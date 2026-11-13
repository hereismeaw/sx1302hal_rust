//! AS923-1 concentrator daemon: Semtech GWMP v2 (PUSH_DATA / PULL_DATA / PULL_RESP / TX_ACK)
//! with a JIT downlink queue and a JSON channel plan.

mod gps;
mod jit;
mod metrics;
mod notify;
mod upstream;
mod utc;

use clap::Parser;
use jit::JitQueue;
use sensecap_board::{reset_carrier, Carrier};
use serde::Deserialize;
use serde_json::json;
use std::collections::BTreeMap;
use sx1302_core::Region;
use sx1302_hal::{
    beacon_antenna, beacon_payload, bw_hz_to_code, count_at, gps_ms, next_beacon_gps_ms, ChannelPlan,
    Concentrator, FskTx, TxGainLut, TxRequest, TxTrigger, AS923_1_BEACON_HZ, BEACON_DELAY_US,
};
use sx1302_transport::SpiConfig;
use std::thread;
use upstream::{Uplink, Upstream, PKT_PULL_ACK, PKT_PULL_RESP};
use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};

#[derive(Parser)]
#[command(name = "sx1302-daemon", about = "Native SX1302 GWMP forwarder (AS923-1)")]
struct Args {
    /// JSON config (`server`, `stat_interval`, `plan`). See configs/sx1302-daemon.json.
    #[arg(long)]
    config: Option<String>,
    /// ChirpStack / Semtech packet-forwarder UDP (`host:port`). Overrides the config.
    #[arg(long)]
    server: Option<String>,
    #[arg(long, default_value = "/dev/spidev0.0")]
    spi: String,
    /// Seconds between `stat` reports. Overrides the config.
    #[arg(long)]
    stat_interval: Option<u64>,
    /// Longest TX air time in ms (AS923 dwell time is 400). 0 = no limit. Overrides the config.
    #[arg(long)]
    dwell_ms: Option<u32>,
    /// TX air time allowed per sliding hour, in percent. 0 = no limit. Overrides the config.
    #[arg(long)]
    duty_cycle_percent: Option<f64>,
    /// Prometheus endpoint, e.g. `0.0.0.0:9302`. Overrides the config.
    #[arg(long)]
    metrics: Option<String>,
}

#[derive(Deserialize, Default)]
struct Config {
    #[serde(default)]
    server: Option<String>,
    /// More network servers. Every uplink and `stat` goes to each of them, and each
    /// may send downlinks (TX_ACK goes back to the one that asked).
    #[serde(default)]
    servers: Vec<String>,
    /// Uplinks kept per server while it does not answer (PUSH_DATA messages,
    /// default 2000, 0 = drop like `lora_pkt_fwd`). Joins are never kept.
    #[serde(default)]
    uplink_buffer: Option<usize>,
    /// Prometheus endpoint (`host:port`). Off when absent.
    #[serde(default)]
    metrics: Option<String>,
    #[serde(default)]
    stat_interval: Option<u64>,
    #[serde(default)]
    plan: Option<ChannelPlan>,
    /// Preset plan when `plan` is absent: `AS923-1`, `US915` or `FCC-800`.
    #[serde(default)]
    region: Option<String>,
    /// 0-based sub-band of the preset: US915 0–7 (default 1), FCC-800 0–4 (default 0).
    #[serde(default)]
    subband: Option<u8>,
    /// Listen to several channel blocks in turn. Off when absent.
    #[serde(default)]
    hop: Option<HopConfig>,
    /// Semtech `tx_gain_lut` (rf_power / pa_gain / pwr_idx). Default = Semtech SX1250 table.
    #[serde(default)]
    tx_gain_lut: Option<TxGainLut>,
    /// AS923-1 Class B beacons (923.4 MHz SF9) plus `tmms` ping slots.
    /// Off unless set: this board has no PPS, so time is the host clock.
    #[serde(default)]
    class_b: bool,
    /// 24-bit NetID written into the beacon (`InfoDesc = 3`) until a GPS fix exists.
    #[serde(default)]
    class_b_net_id: u32,
    /// NMEA UART, 9600 8N1. RAK2287/RAK5146 Pi HAT: `/dev/ttyAMA0` when the ZOE-M8Q is fitted.
    /// 1PPS is already wired to the concentrator. No fix yet: Class B keeps the host clock.
    #[serde(default)]
    gps: Option<String>,
    /// `sensecap-m1` (default), `rak2287`, `rak5146`, or `rak5148`.
    #[serde(default)]
    board: Option<String>,
    /// Refuse any TX longer than this on air (AS923 dwell time is 400). Off when absent or 0.
    #[serde(default)]
    dwell_ms: Option<u32>,
    /// TX air-time limit over a sliding hour, all channels, beacons included (e.g. 1.0).
    /// Off when absent or 0.
    #[serde(default)]
    duty_cycle_percent: Option<f64>,
}

/// One SX1302 demodulates eight 125 kHz channels. Hopping restarts it on the next
/// block every `dwell_s`, so uplinks on the other blocks are missed meanwhile.
/// Each block is the base plan moved by one entry of `shift_hz`, or by
/// `0, step_hz, 2·step_hz, …` (`steps` entries). All eight US915 sub-bands from
/// sub-band 0: `{"dwell_s": 20, "step_hz": 1600000, "steps": 8}`.
#[derive(Deserialize, Default, Clone)]
struct HopConfig {
    /// Seconds on each block (minimum 5).
    dwell_s: u64,
    #[serde(default)]
    shift_hz: Vec<i64>,
    #[serde(default)]
    step_hz: i64,
    #[serde(default)]
    steps: u32,
}

/// The plans to cycle through, each checked against the region of the base plan.
fn hop_plans(base: &ChannelPlan, region: Region, hop: &HopConfig) -> Result<Vec<ChannelPlan>, String> {
    let shifts: Vec<i64> = if hop.shift_hz.is_empty() {
        (0..i64::from(hop.steps)).map(|i| i * hop.step_hz).collect()
    } else {
        hop.shift_hz.clone()
    };
    if shifts.len() < 2 {
        return Err("hop: give at least two blocks (`shift_hz`, or `step_hz` + `steps`)".into());
    }
    shifts
        .iter()
        .map(|d| {
            let p = base.shifted(*d).map_err(|e| format!("hop {d} Hz: {e}"))?;
            p.validate(region).map_err(|e| format!("hop {d} Hz: {e}"))?;
            Ok(p)
        })
        .collect()
}

struct Hop {
    plans: Vec<ChannelPlan>,
    dwell: Duration,
    idx: usize,
    since: Instant,
}

/// No hop until this long after the last uplink: a Class A answer (RX2 of a
/// JoinAccept is 6 s) may still be on its way from the server.
const HOP_QUIET: Duration = Duration::from_secs(7);

struct GpsLock {
    gps_ms: u64,
    count_us: u32,
    lat_deg: f64,
    lon_deg: f64,
    at: Instant,
}

/// Counts for one `stat` interval.
#[derive(Default)]
struct Stats {
    rxnb: u32,
    rxok: u32,
    rxfw: u32,
    dwnb: u32,
    txnb: u32,
    /// Downlinks refused because the JIT window overlapped a queued packet.
    coll: u32,
}

/// Counts since start, for the metrics page.
#[derive(Default)]
struct Totals {
    rx: u64,
    rx_ok: u64,
    rx_fwd: u64,
    dl: u64,
    tx: u64,
    tx_arm_err: u64,
    /// Refused downlinks by reason (`too_late`, `duty_cycle`, `TX_FREQ`, ...).
    refused: BTreeMap<&'static str, u64>,
    beacons: u64,
    hops: u64,
}

/// The server list: `--server` (or `server`) first, then `servers`, without repeats.
fn server_list(first: Option<String>, more: &[String]) -> Vec<String> {
    let mut out: Vec<String> = Vec::new();
    for s in first.into_iter().chain(more.iter().cloned()) {
        let s = s.trim().to_string();
        if !s.is_empty() && !out.contains(&s) {
            out.push(s);
        }
    }
    out
}

/// systemd STATUS link part. The single-server strings are what the soak sampler logs.
fn link_status(ups: &[Upstream]) -> String {
    let up = ups.iter().filter(|u| !u.lost).count();
    let backlog: usize = ups.iter().map(|u| u.spool.backlog_len()).sum();
    let mut s = match (ups.len(), up) {
        (0, _) => "no server".to_string(),
        (n, u) if u == n => "server ok".to_string(),
        (1, _) => "no server ACK > 30 s".to_string(),
        (n, u) => format!("{u}/{n} servers ok"),
    };
    if backlog > 0 {
        s += &format!(", backlog {backlog}");
    }
    s
}

fn main() {
    if let Err(e) = run() {
        eprintln!("sx1302-daemon: {e}");
        std::process::exit(1);
    }
}

fn run() -> Result<(), Box<dyn std::error::Error>> {
    let args = Args::parse();
    let cfg: Config = match &args.config {
        Some(p) => serde_json::from_str(&std::fs::read_to_string(p)?)
            .map_err(|e| format!("config {p}: {e}"))?,
        None => Config::default(),
    };
    let servers = server_list(args.server.or(cfg.server), &cfg.servers);
    let uplink_buffer = cfg.uplink_buffer.unwrap_or(upstream::DEFAULT_BUFFER);
    let metrics_addr = args.metrics.or(cfg.metrics).filter(|a| !a.is_empty());
    let stat_interval = args.stat_interval.or(cfg.stat_interval).unwrap_or(30);
    let plan = match (cfg.plan, cfg.region.as_deref()) {
        (Some(p), _) => p,
        (None, Some(r)) => ChannelPlan::preset(Region::parse(r)?, cfg.subband)?,
        (None, None) => ChannelPlan::as923_1_default(),
    };
    let lut = cfg.tx_gain_lut.unwrap_or_default();
    let region = plan.band()?;
    let mut hop = match &cfg.hop {
        Some(h) => {
            let plans = hop_plans(&plan, region, h)?;
            let dwell = Duration::from_secs(h.dwell_s.max(5));
            eprintln!(
                "hopping over {} blocks in {region}, {} s each (uplinks on the other blocks are missed)",
                plans.len(),
                dwell.as_secs()
            );
            Some(Hop { plans, dwell, idx: 0, since: Instant::now() })
        }
        None => None,
    };
    let plan = hop.as_ref().map_or(plan, |h| h.plans[0].clone());
    let dwell_ms = args.dwell_ms.or(cfg.dwell_ms).filter(|d| *d > 0);
    let duty_cycle_percent = args.duty_cycle_percent.or(cfg.duty_cycle_percent).filter(|d| *d > 0.0);

    let carrier = Carrier::parse(cfg.board.as_deref().unwrap_or(""))?;
    match reset_carrier(carrier) {
        Ok(n) => n.iter().for_each(|l| eprintln!("{l}")),
        Err(e) => return Err(e.into()),
    }
    let c = Concentrator::open(SpiConfig {
        path: args.spi,
        ..Default::default()
    })?;
    // SX1303: ask for the precision timestamp before start (ARB dual demodulation
    // is set while the firmware starts). `start` falls back to legacy on an SX1302.
    c.set_fine_timestamp(carrier == Carrier::Rak5146);
    for line in c.start(&plan)? {
        eprintln!("{line}");
    }
    let eui = c.otp_eui().unwrap_or(0);
    eprintln!("gateway EUI {eui:016x}");
    let mut ups: Vec<Upstream> = Vec::new();
    for server in &servers {
        ups.push(Upstream::connect(server, uplink_buffer).map_err(|e| format!("server {server}: {e}"))?);
        eprintln!("PUSH_DATA/PULL_DATA -> {server}");
    }
    if !ups.is_empty() {
        eprintln!("uplink buffer: {uplink_buffer} messages per server while it does not answer (joins are not kept)");
    }
    let metrics = match &metrics_addr {
        Some(a) => {
            let m = metrics::MetricsServer::bind(a).map_err(|e| format!("metrics {a}: {e}"))?;
            eprintln!("metrics on http://{a}/metrics");
            Some(m)
        }
        None => None,
    };
    let mut tot = Totals::default();
    let mut last_temp: Option<f32> = None;
    let mut st = Stats::default();
    let mut jit = JitQueue::default();
    let mut air = jit::Airtime::new(dwell_ms, duty_cycle_percent);
    eprintln!(
        "TX limits: dwell {}, duty cycle {}",
        dwell_ms.map_or("off".into(), |d| format!("{d} ms")),
        duty_cycle_percent.map_or("off".into(), |d| format!("{d} % per hour"))
    );
    let mut last_rx = Instant::now() - HOP_QUIET;
    let started = Instant::now();
    let mut last_beacon_ms: u64 = 0;
    if cfg.class_b {
        eprintln!("class B on: beacon 923.4 MHz SF9 every 128 s");
    }
    let mut gps_port = match cfg.gps.as_deref().filter(|p| !p.is_empty()) {
        Some(path) => match gps::GpsPort::open(path) {
            Ok(p) => {
                if let Err(e) = c.enable_pps() {
                    eprintln!("gps pps enable: {e}");
                }
                eprintln!("gps NMEA {path} (PPS lock replaces the host clock when a fix arrives)");
                Some(p)
            }
            Err(e) => {
                eprintln!("gps {path}: {e} (host clock)");
                None
            }
        },
        None => None,
    };
    let mut gps_lock: Option<GpsLock> = None;
    let mut last_pps: Option<u32> = None;
    let mut pps_edge: Option<(u32, u64, Instant)> = None;
    let mut last_rtc = Instant::now() - Duration::from_secs(3600);
    let mut token: u16 = 1;
    let mut last_stat = Instant::now();
    // systemd watchdog: ping at half the interval, only after a passing health check.
    let wd_every = notify::watchdog_interval().map(|d| d / 2);
    let mut last_health = Instant::now();
    let mut health_ts = Some(c.health_check(None)?);
    // Test hook: answer TOO_LATE to every other PULL_RESP so the network server
    // falls back to RX2 (exercises RX2 / JoinAccept-RX2 end to end). Never set in production.
    let reject_alternate = std::env::var_os("SX1302_TEST_REJECT_ALTERNATE_DL").is_some();
    let mut dl_seen: u32 = 0;
    if reject_alternate {
        eprintln!("TEST MODE: rejecting every other downlink with TOO_LATE");
    }
    // Board temperature (STTS751 @ I2C-1 0x39) for RSSI temperature compensation.
    let temp_sensor = match sensecap_board::temp::Stts751::open() {
        Ok(s) => Some(s),
        Err(e) => {
            eprintln!("temperature sensor: {e} (RSSI temperature compensation off)");
            None
        }
    };
    let read_temp = |c: &Concentrator| -> Option<f32> {
        let t = temp_sensor.as_ref()?.temperature_c().ok();
        c.set_temperature(t);
        t
    };
    if let Some(t) = read_temp(&c) {
        eprintln!("board temperature {t:.1} °C (STTS751), RSSI tcomp on");
    }
    notify::notify("READY=1\nSTATUS=concentrator started");
    loop {
        if last_health.elapsed() >= wd_every.unwrap_or(Duration::from_secs(10)) {
            // Err here exits -> systemd restarts -> GPIO reset + full start.
            let ts = c.health_check(health_ts).map_err(|e| format!("health check: {e}"))?;
            health_ts = Some(ts);
            jit.expire(ts);
            last_health = Instant::now();
            let temp = read_temp(&c);
            last_temp = temp;
            if wd_every.is_some() {
                let link = link_status(&ups);
                let temp = temp.map_or("n/a".into(), |t| format!("{t:.1} °C"));
                notify::notify(&format!(
                    "WATCHDOG=1\nSTATUS={link}, jit queue {}, board {temp}",
                    jit.len()
                ));
            }
        }
        let dump = c.rx_poll()?;
        if !dump.packets.is_empty() {
            st.rxnb += dump.packets.len() as u32;
            st.rxok += dump.packets.iter().filter(|p| p.stat == 1).count() as u32;
            tot.rx += dump.packets.len() as u64;
            tot.rx_ok += dump.packets.iter().filter(|p| p.stat == 1).count() as u64;
            // Like lora_pkt_fwd: forward CRC-OK only. The multi-SF chain and the LoRa
            // service modem can both demodulate one frame: keep the best-SNR copy.
            // SX1303 dual demodulation also gives two copies: like `lgw_receive`,
            // keep the one that carries the fine timestamp.
            let mut ok: Vec<_> = dump.packets.iter().filter(|p| p.stat == 1).collect();
            ok.sort_by(|a, b| {
                b.ftime_ns
                    .is_some()
                    .cmp(&a.ftime_ns.is_some())
                    .then(b.snr.total_cmp(&a.snr))
            });
            let mut seen: Vec<(u32, &[u8])> = Vec::new();
            ok.retain(|p| {
                let dup = seen.iter().any(|(t, d)| {
                    p.count_us.wrapping_sub(*t).min(t.wrapping_sub(p.count_us)) < 1_000
                        && *d == p.payload.as_slice()
                });
                if !dup {
                    seen.push((p.count_us, &p.payload));
                }
                !dup
            });
            for p in &ok {
                if let Some(note) = sx1302_hal::PhyHdr::parse(&p.payload).and_then(|h| h.note()) {
                    eprintln!("{note}");
                }
            }
            let rxpk: Vec<_> = ok.iter().map(|p| p.to_rxpk()).collect();
            if !rxpk.is_empty() {
                last_rx = Instant::now();
                eprintln!("{}", json!({ "rxpk": rxpk }));
                if !ups.is_empty() {
                    let u = Uplink {
                        keep: ok.iter().map(|p| upstream::replayable_phy(&p.payload)).collect(),
                        rxpk,
                        unix_ms: unix_now_ms(),
                    };
                    st.rxfw += u.rxpk.len() as u32;
                    tot.rx_fwd += u.rxpk.len() as u64;
                    for up in ups.iter_mut() {
                        up.send_rxpk(token, eui, u.clone());
                    }
                    token = token.wrapping_add(1);
                }
            }
        }

        if gps_port.is_some() {
            if let Ok(Some(pps)) = c.pps_count_us() {
                if last_pps != Some(pps) {
                    last_pps = Some(pps);
                    if let Ok(conc) = c.now_us() {
                        let host = SystemTime::now()
                            .duration_since(UNIX_EPOCH)
                            .map(|d| d.as_millis() as u64)
                            .unwrap_or(0);
                        if let Some(at_edge) = gps::host_ms_at_pps(host, conc, pps) {
                            pps_edge = Some((pps, at_edge, Instant::now()));
                        }
                    }
                }
            }
            if let Some(port) = gps_port.as_mut() {
                for rmc in port.poll() {
                    if let Some((cnt, host_at_edge, when)) = pps_edge {
                        if when.elapsed() < Duration::from_millis(1200) {
                            let fresh = gps_lock.is_none();
                            gps_lock = Some(GpsLock {
                                gps_ms: rmc.gps_ms / 1000 * 1000,
                                count_us: cnt,
                                lat_deg: rmc.lat_deg,
                                lon_deg: rmc.lon_deg,
                                at: Instant::now(),
                            });
                            if fresh {
                                eprintln!("gps lock {} us at concentrator {cnt}", rmc.gps_ms);
                            }
                            let delta = host_at_edge as i64 - rmc.unix_ms as i64;
                            // 20 ms is above one SPI read of the counter and below a useful step.
                            if delta.unsigned_abs() > 20 {
                                let Some(conc) = c.now_us().ok() else { continue };
                                let Some(target) = gps::unix_ms_now(rmc.unix_ms, conc, cnt) else { continue };
                                let write_rtc = delta.unsigned_abs() >= 1000 || last_rtc.elapsed() > Duration::from_secs(600);
                                match gps::step_host_and_rtc(target, write_rtc) {
                                    Ok(()) => {
                                        eprintln!("host clock stepped {delta} ms from GPS");
                                        if write_rtc {
                                            last_rtc = Instant::now();
                                        }
                                    }
                                    Err(e) => eprintln!("host clock step: {e}"),
                                }
                            }
                        }
                    }
                }
            }
        }
        if let Some(lock) = &gps_lock {
            if lock.at.elapsed() > Duration::from_secs(10) {
                eprintln!("gps lock stale, host clock");
                gps_lock = None;
            }
        }

        if cfg.class_b {
            if let Ok(now) = c.now_us() {
                let unix_ms = SystemTime::now()
                    .duration_since(UNIX_EPOCH)
                    .map(|d| d.as_millis() as u64)
                    .unwrap_or(0);
                let (gps, origin) = match gps_lock.as_ref() {
                    Some(lock) => (lock.gps_ms, lock.count_us),
                    None => (gps_ms(unix_ms), now),
                };
                let boundary = next_beacon_gps_ms(gps);
                if boundary != last_beacon_ms {
                    let at = count_at(origin, gps, boundary).wrapping_add(BEACON_DELAY_US);
                    let ahead = at.wrapping_sub(now) as i32;
                    // Queue inside the last 9 s so the clock mapping is fresh. The JIT queue
                    // arms 100 ms before emission and takes anything >= MIN_AHEAD_US out,
                    // so a late loop still sends the beacon. Later than that: TOO_LATE, once.
                    if ahead < 9_000_000 {
                        let payload = if let Some(lock) = gps_lock.as_ref() {
                            beacon_antenna((boundary / 1000) as u32, lock.lat_deg, lock.lon_deg)
                        } else {
                            beacon_payload(
                                (boundary / 1000) as u32,
                                cfg.class_b_net_id,
                                (eui & 0xff_ffff) as u32,
                            )
                        };
                        let Ok(mut req) = TxRequest::as923_1_sf7(payload.to_vec()) else {
                            eprintln!("class B beacon: bad frequency");
                            last_beacon_ms = boundary;
                            continue;
                        };
                        req.freq_hz = AS923_1_BEACON_HZ;
                        req.sf = 9;
                        req.bw = bw_hz_to_code(125_000).unwrap_or(0x04);
                        req.cr = 1;
                        req.preamble = 10;
                        req.implicit = true;
                        req.invert_pol = false;
                        req.set_power(&lut, 14);
                        req.trigger = TxTrigger::Timestamped(at);
                        // One attempt per beacon: a refusal cannot clear before the slot.
                        last_beacon_ms = boundary;
                        let mono_ms = started.elapsed().as_millis() as u64;
                        match submit(&mut jit, &mut air, req, Some(at), now, jit::MAX_AHEAD_CLASS_B_US, mono_ms) {
                            Ok(()) => {
                                tot.beacons += 1;
                                eprintln!(
                                    "class B beacon queued gps_ms={boundary} tmst={at} (+{} ms)",
                                    ahead / 1000
                                )
                            }
                            Err(e) => {
                                if e == jit::JitError::Collision {
                                    st.coll += 1;
                                }
                                *tot.refused.entry(e.label()).or_default() += 1;
                                eprintln!("class B beacon gps_ms={boundary} skipped: {}", e.reason());
                            }
                        }
                    }
                }
            }
        }

        // JIT: arm whatever is due.
        if jit.len() > 0 {
            let now = c.now_us()?;
            while let Some(item) = jit.pop_due(now) {
                let t0 = Instant::now();
                match c.send(&item.req) {
                    Ok(msg) => {
                        st.txnb += 1;
                        tot.tx += 1;
                        eprintln!("{msg} [armed in {} µs, queue {}]", t0.elapsed().as_micros(), jit.len());
                    }
                    Err(e) => {
                        tot.tx_arm_err += 1;
                        eprintln!("jit arm: {e}")
                    }
                }
            }
        }

        // Hop to the next block: same path as a service restart (GPIO reset + start),
        // so the counter restarts. Wait until nothing is queued or on the air and the
        // channel has been quiet; after two dwell times hop even if uplinks keep coming.
        if let Some(h) = hop.as_mut() {
            let waited = h.since.elapsed();
            if waited >= h.dwell
                && jit.len() == 0
                && (last_rx.elapsed() >= HOP_QUIET || waited >= h.dwell * 2)
                && !jit.on_air(c.now_us()?)
            {
                let next = (h.idx + 1) % h.plans.len();
                let t0 = Instant::now();
                reset_carrier(carrier)?;
                c.start(&h.plans[next])?;
                if gps_port.is_some() {
                    if let Err(e) = c.enable_pps() {
                        eprintln!("gps pps enable: {e}");
                    }
                }
                gps_lock = None;
                last_pps = None;
                pps_edge = None;
                health_ts = None;
                h.idx = next;
                h.since = Instant::now();
                tot.hops += 1;
                let [a, b] = h.plans[next].radio_hz;
                eprintln!(
                    "hop {}/{}: radios {a}/{b} Hz (restart took {} ms)",
                    next + 1,
                    h.plans.len(),
                    t0.elapsed().as_millis()
                );
            }
        }

        if !ups.is_empty() && last_stat.elapsed() > Duration::from_secs(stat_interval) {
            let temp = read_temp(&c);
            let time = gwmp_time();
            let time_src = if gps_lock.is_some() { "pps" } else { "host" };
            for (i, up) in ups.iter_mut().enumerate() {
                let stat = stat_json(&time, &st, up.ackr(), temp, time_src);
                // One log line per stat interval (the first server's ackr) keeps the journal as before.
                if i == 0 {
                    eprintln!("{stat}");
                }
                up.send_stat(token, eui, &stat.to_string());
            }
            st = Stats::default();
            token = token.wrapping_add(1);
            last_stat = Instant::now();
        }
        let mut buf = [0u8; 65536];
        for up in ups.iter_mut() {
            up.tick(&mut token, eui);
            while let Some(n) = up.recv(&mut buf) {
                match buf[3] {
                    PKT_PULL_ACK if !up.pull_acked => {
                        eprintln!("PULL_ACK from {} (downlink path open)", up.addr);
                        up.pull_acked = true;
                    }
                    PKT_PULL_RESP => {
                        st.dwnb += 1;
                        tot.dl += 1;
                        let resp_token = u16::from_be_bytes([buf[1], buf[2]]);
                        let json = std::str::from_utf8(&buf[4..n]).unwrap_or("");
                        eprintln!("PULL_RESP {json}");
                        dl_seen += 1;
                        let now_us = c.now_us().unwrap_or(0);
                        let unix_ms = SystemTime::now()
                            .duration_since(UNIX_EPOCH)
                            .map(|d| d.as_millis() as u64)
                            .unwrap_or(0);
                        let (origin_gps, origin_cnt) = match gps_lock.as_ref() {
                            Some(lock) => (lock.gps_ms, lock.count_us),
                            None => (gps_ms(unix_ms), now_us),
                        };
                        let err = match parse_txpk(json, &lut, origin_cnt, origin_gps) {
                            Ok(_) if reject_alternate && dl_seen % 2 == 1 => {
                                eprintln!("TEST MODE: rejecting this downlink (TOO_LATE)");
                                "TOO_LATE"
                            }
                            Ok((req, start, horizon)) => {
                                let now = c.now_us()?;
                                let mono_ms = started.elapsed().as_millis() as u64;
                                match submit(&mut jit, &mut air, req, start, now, horizon, mono_ms) {
                                    Ok(()) => {
                                        eprintln!(
                                            "queued tmst={} (+{} ms), queue {}",
                                            start.map_or("imme".into(), |t| t.to_string()),
                                            start.map_or(0, |t| t.wrapping_sub(now) as i32 / 1000),
                                            jit.len()
                                        );
                                        "NONE"
                                    }
                                    Err(e) => {
                                        if e == jit::JitError::Collision {
                                            st.coll += 1;
                                        }
                                        *tot.refused.entry(e.label()).or_default() += 1;
                                        eprintln!("jit: {}", e.reason());
                                        e.gwmp()
                                    }
                                }
                            }
                            Err(e) => {
                                *tot.refused.entry(e).or_default() += 1;
                                eprintln!("txpk: {e}");
                                e
                            }
                        };
                        up.tx_ack(resp_token, eui, err);
                    }
                    _ => {}
                }
            }
        }
        if let Some(m) = &metrics {
            m.poll(|| {
                render_metrics(
                    &tot,
                    &ups,
                    jit.len(),
                    last_temp,
                    gps_lock.is_some(),
                    started.elapsed().as_secs(),
                    hop.as_ref().map(|h| h.idx),
                )
            });
        }
        thread::sleep(Duration::from_millis(5));
    }
}

/// Air-time limits first, then the JIT queue. `mono_ms` is host-monotonic "now".
fn submit(
    jit: &mut JitQueue,
    air: &mut jit::Airtime,
    req: TxRequest,
    start_us: Option<u32>,
    now: u32,
    horizon: u32,
    mono_ms: u64,
) -> Result<(), jit::JitError> {
    let toa_us = req.time_on_air_us();
    let ahead_ms = start_us.map_or(0, |t| i64::from(t.wrapping_sub(now) as i32) / 1000);
    let at_ms = mono_ms.saturating_add_signed(ahead_ms);
    air.check(at_ms, toa_us)?;
    jit.enqueue_within(req, start_us, now, horizon)?;
    air.record(at_ms, toa_us);
    Ok(())
}

fn unix_now_ms() -> u64 {
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map(|d| d.as_millis() as u64)
        .unwrap_or(0)
}

/// The Prometheus page. Counters are since daemon start.
fn render_metrics(
    tot: &Totals,
    ups: &[Upstream],
    jit_len: usize,
    temp: Option<f32>,
    pps: bool,
    uptime_s: u64,
    hop_idx: Option<usize>,
) -> String {
    let mut p = metrics::Page::default();
    p.gauge("uptime_seconds", "Seconds since the daemon started.", uptime_s as f64)
        .counter("rx_packets_total", "Frames read from the RX FIFO, CRC good or bad.", tot.rx)
        .counter("rx_crc_ok_total", "Frames with a good CRC.", tot.rx_ok)
        .counter("rx_forwarded_total", "rxpk sent upstream after dropping duplicate copies.", tot.rx_fwd)
        .counter("downlink_requests_total", "PULL_RESP received from all servers.", tot.dl)
        .counter("tx_total", "Packets armed on the concentrator.", tot.tx)
        .counter("tx_arm_errors_total", "Packets the concentrator refused to arm.", tot.tx_arm_err)
        .counter("beacons_total", "Class B beacons queued.", tot.beacons)
        .counter("hops_total", "Channel-block hops.", tot.hops)
        .gauge("jit_queue_length", "Downlinks waiting in the JIT queue.", jit_len as f64)
        .gauge("time_source_pps", "1 while a GPS PPS lock sets the time, 0 on the host clock.", f64::from(u8::from(pps)));
    let refused: Vec<(String, f64)> =
        tot.refused.iter().map(|(r, n)| (format!("reason=\"{}\"", metrics::label(r)), *n as f64)).collect();
    let refused: Vec<(&str, f64)> = refused.iter().map(|(l, v)| (l.as_str(), *v)).collect();
    p.family("tx_refused_total", "counter", "Downlinks refused, by reason.", &refused);
    if let Some(t) = temp {
        p.gauge("board_temperature_celsius", "STTS751 board temperature.", f64::from(t));
    }
    if let Some(i) = hop_idx {
        p.gauge("hop_block", "Channel block in use (0-based).", i as f64);
    }
    let labels: Vec<String> = ups.iter().map(|u| format!("server=\"{}\"", metrics::label(&u.addr))).collect();
    let mut per = |name: &str, kind: &str, help: &str, f: &dyn Fn(&Upstream) -> f64| {
        let s: Vec<(&str, f64)> = labels.iter().zip(ups).map(|(l, u)| (l.as_str(), f(u))).collect();
        p.family(name, kind, help, &s);
    };
    per("server_up", "gauge", "1 while the server answers.", &|u| f64::from(u8::from(!u.lost)));
    per("server_ack_ratio_percent", "gauge", "PUSH_ACK per PUSH_DATA since start.", &|u| u.ackr());
    per("server_push_sent_total", "counter", "PUSH_DATA sent.", &|u| u.push_sent as f64);
    per("server_push_ack_total", "counter", "PUSH_ACK received.", &|u| u.push_ack as f64);
    per("server_outages_total", "counter", "Times the server stopped answering for 30 s.", &|u| u.outages as f64);
    per("uplink_backlog", "gauge", "PUSH_DATA messages waiting for the server.", &|u| u.spool.backlog_len() as f64);
    per("uplink_replayed_total", "counter", "rxpk sent late from the backlog.", &|u| u.spool.replayed as f64);
    per(
        "uplink_dropped_total",
        "counter",
        "rxpk lost: backlog full or off, or a join that cannot wait.",
        &|u| u.spool.dropped as f64,
    );
    p.finish()
}

/// GWMP `txpk` -> request, emission time (`None` = immediate), and the JIT horizon.
/// `tmms` is GPS milliseconds (Class B ping slot).
/// `count_us` / `origin_gps_ms` are one paired sample (host clock, or a PPS lock).
fn parse_txpk(
    json: &str,
    lut: &TxGainLut,
    count_us: u32,
    origin_gps_ms: u64,
) -> Result<(TxRequest, Option<u32>, u32), &'static str> {
    let v: serde_json::Value = serde_json::from_str(json).map_err(|_| "COLLISION_PACKET")?;
    let tx = v.get("txpk").ok_or("COLLISION_PACKET")?;
    let freq = tx.get("freq").and_then(|x| x.as_f64()).ok_or("TX_FREQ")?;
    let freq_hz = (freq * 1_000_000.0).round() as u32;
    let region = if Region::As923_1.tx_contains(freq_hz) {
        Region::As923_1
    } else if Region::Us915.tx_contains(freq_hz) {
        Region::Us915
    } else if Region::Fcc800.tx_contains(freq_hz) {
        Region::Fcc800
    } else if Region::Fcc800.contains(freq_hz) {
        eprintln!("txpk {freq_hz} Hz is the FCC-800 device side (809–817); the gateway transmits 45 MHz higher");
        return Err("TX_FREQ");
    } else if Region::Ism2400.contains(freq_hz) {
        eprintln!("txpk {freq_hz} Hz is ISM2400; SX1250 cannot transmit 2.4 GHz");
        return Err("TX_FREQ");
    } else {
        return Err("TX_FREQ");
    };
    let payload = b64_decode(tx.get("data").and_then(|x| x.as_str()).unwrap_or(""))
        .map_err(|_| "COLLISION_PACKET")?;
    let imme = tx.get("imme").and_then(|x| x.as_bool()).unwrap_or(false);
    let (start, horizon) = if imme {
        (None, jit::MAX_AHEAD_US)
    } else if let Some(tmms) = tx.get("tmms").and_then(|x| x.as_u64()) {
        let at = count_at(count_us, origin_gps_ms, tmms);
        (Some(at), jit::MAX_AHEAD_CLASS_B_US)
    } else if let Some(tmst) = tx.get("tmst").and_then(|x| x.as_u64()) {
        (Some(tmst as u32), jit::MAX_AHEAD_US)
    } else {
        return Err("GPS_UNLOCKED");
    };
    let modu = tx.get("modu").and_then(|x| x.as_str()).unwrap_or("LORA");
    let mut req = TxRequest::as923_1_sf7(payload).map_err(|_| "TX_FREQ")?;
    req.region = region;
    req.freq_hz = freq_hz;
    // GWMP `powe` (dBm, conducted) -> nearest LUT entry not above it.
    let powe = tx.get("powe").and_then(|x| x.as_i64()).unwrap_or(14).clamp(-128, 127) as i8;
    req.set_power(lut, powe);
    req.trigger = match start {
        None => TxTrigger::Immediate,
        Some(t) => TxTrigger::Timestamped(t),
    };
    match modu {
        "LORA" => {
            let datr = tx.get("datr").and_then(|x| x.as_str()).unwrap_or("SF7BW125");
            let (sf, bw_khz) = parse_datr(datr).ok_or("COLLISION_PACKET")?;
            req.sf = sf;
            req.bw = bw_hz_to_code(bw_khz * 1000).ok_or("COLLISION_PACKET")?;
            req.cr = match tx.get("codr").and_then(|x| x.as_str()).unwrap_or("4/5") {
                "4/6" => 2,
                "4/7" => 3,
                "4/8" => 4,
                _ => 1,
            };
            req.invert_pol = tx.get("ipol").and_then(|x| x.as_bool()).unwrap_or(false);
            req.preamble = tx.get("prea").and_then(|x| x.as_u64()).unwrap_or(8) as u16;
        }
        "FSK" => {
            let datarate = tx.get("datr").and_then(|x| x.as_u64()).ok_or("COLLISION_PACKET")? as u32;
            let fdev_hz = tx.get("fdev").and_then(|x| x.as_u64()).unwrap_or(25_000) as u32;
            req.fsk = Some(FskTx {
                datarate,
                fdev_hz,
                sync_word: 0xC1_94_C1,
                sync_word_size: 3,
            });
            req.preamble = tx.get("prea").and_then(|x| x.as_u64()).unwrap_or(5) as u16;
        }
        _ => return Err("COLLISION_PACKET"),
    }
    Ok((req, start, horizon))
}

fn parse_datr(d: &str) -> Option<(u8, u32)> {
    let rest = d.strip_prefix("SF")?;
    let (sf, bw) = rest.split_once("BW")?;
    Some((sf.parse().ok()?, bw.parse().ok()?))
}

fn b64_decode(s: &str) -> Result<Vec<u8>, &'static str> {
    fn val(c: u8) -> Result<u8, &'static str> {
        Ok(match c {
            b'A'..=b'Z' => c - b'A',
            b'a'..=b'z' => c - b'a' + 26,
            b'0'..=b'9' => c - b'0' + 52,
            b'+' => 62,
            b'/' => 63,
            _ => return Err("bad base64"),
        })
    }
    let bytes: Vec<u8> = s.bytes().filter(|c| *c != b'=' && !c.is_ascii_whitespace()).collect();
    let mut out = Vec::new();
    for chunk in bytes.chunks(4) {
        if chunk.len() < 2 {
            break;
        }
        let a = val(chunk[0])?;
        let b = val(chunk[1])?;
        out.push((a << 2) | (b >> 4));
        if chunk.len() > 2 {
            let c = val(chunk[2])?;
            out.push((b << 4) | (c >> 2));
            if chunk.len() > 3 {
                let d = val(chunk[3])?;
                out.push((c << 6) | d);
            }
        }
    }
    Ok(out)
}

/// GWMP `stat`. `temp` is the STTS751 reading. `time_src` is `pps` after a
/// PPS+RMC lock and `host` otherwise. `coll` counts JIT collisions.
fn stat_json(time: &str, st: &Stats, ackr: f64, temp: Option<f32>, time_src: &str) -> serde_json::Value {
    json!({
        "stat": {
            "time": time,
            "rxnb": st.rxnb,
            "rxok": st.rxok,
            "rxfw": st.rxfw,
            "ackr": ackr,
            "dwnb": st.dwnb,
            "txnb": st.txnb,
            "temp": temp,
            "time_src": time_src,
            "coll": st.coll
        }
    })
}

/// GWMP `stat.time`: "YYYY-MM-DD hh:mm:ss GMT".
fn gwmp_time() -> String {
    let secs = std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .map(|d| d.as_secs())
        .unwrap_or(0);
    utc::format_utc(secs)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn servers_and_status() {
        let l = server_list(Some("a:1700".into()), &["b:1700".into(), "a:1700".into(), " ".into()]);
        assert_eq!(l, ["a:1700", "b:1700"]);
        assert!(server_list(None, &[]).is_empty());
        assert_eq!(link_status(&[]), "no server");
        let mut a = Upstream::connect("127.0.0.1:9", 4).unwrap();
        assert_eq!(link_status(std::slice::from_ref(&a)), "server ok");
        a.lost = true;
        assert_eq!(link_status(std::slice::from_ref(&a)), "no server ACK > 30 s");
        let b = Upstream::connect("127.0.0.1:9", 4).unwrap();
        assert_eq!(link_status(&[a, b]), "1/2 servers ok");
    }

    #[test]
    fn metrics_page_lists_servers() {
        let mut tot = Totals { rx: 5, ..Default::default() };
        tot.refused.insert("duty_cycle", 2);
        let up = Upstream::connect("127.0.0.1:9", 4).unwrap();
        let s = render_metrics(&tot, &[up], 1, Some(37.5), false, 60, None);
        assert!(s.contains("sx1302_rx_packets_total 5\n"));
        assert!(s.contains("sx1302_tx_refused_total{reason=\"duty_cycle\"} 2\n"));
        assert!(s.contains("sx1302_server_up{server=\"127.0.0.1:9\"} 1\n"));
        assert!(s.contains("sx1302_board_temperature_celsius 37.5\n"));
        assert!(!s.contains("hop_block"));
    }

    #[test]
    fn datr() {
        assert_eq!(parse_datr("SF10BW125"), Some((10, 125)));
        assert_eq!(parse_datr("SF7BW250"), Some((7, 250)));
        assert_eq!(parse_datr("4/5"), None);
    }

    #[test]
    fn b64() {
        assert_eq!(b64_decode("R1cwMQ==").unwrap(), b"GW01");
    }

    #[test]
    fn txpk_lora_and_fsk() {
        let lut = TxGainLut::default();
        let (r, t, h) = parse_txpk(
            r#"{"txpk":{"imme":false,"tmst":5000000,"freq":923.2,"powe":16,"modu":"LORA","datr":"SF10BW125","codr":"4/5","ipol":true,"data":"AQID"}}"#,
            &lut,
            0,
            0,
        )
        .unwrap();
        assert_eq!((r.sf, r.invert_pol, t), (10, true, Some(5_000_000)));
        assert_eq!(h, jit::MAX_AHEAD_US);
        assert_eq!((r.pa_gain, r.power_idx), (0, 20)); // 16 dBm entry
        let (r, t, h) = parse_txpk(
            r#"{"txpk":{"imme":false,"tmms":2000,"freq":923.4,"powe":14,"modu":"LORA","datr":"SF9BW125","codr":"4/5","ipol":true,"data":"AQID"}}"#,
            &lut,
            1_000_000,
            0,
        )
        .unwrap();
        assert_eq!(h, jit::MAX_AHEAD_CLASS_B_US);
        assert!(!r.implicit);
        assert_eq!(t, Some(1_000_000 + 2_000_000));
        let (r, t, _) = parse_txpk(
            r#"{"txpk":{"imme":true,"freq":922.3,"powe":30,"modu":"FSK","datr":50000,"fdev":25000,"data":"AQID"}}"#,
            &lut,
            0,
            0,
        )
        .unwrap();
        assert_eq!(t, None);
        assert_eq!(r.fsk.unwrap().datarate, 50_000);
        assert_eq!((r.pa_gain, r.power_idx), (1, 14)); // capped at 27 dBm
        assert_eq!(
            parse_txpk(
                r#"{"txpk":{"tmst":1,"freq":868.1,"modu":"LORA","datr":"SF7BW125","data":"AQ=="}}"#,
                &lut,
                0,
                0,
            )
            .unwrap_err(),
            "TX_FREQ"
        );
        let (r, _, _) = parse_txpk(
            r#"{"txpk":{"imme":true,"freq":903.9,"modu":"LORA","datr":"SF7BW125","data":"AQ=="}}"#,
            &lut,
            0,
            0,
        )
        .unwrap();
        assert_eq!(r.region, Region::Us915);
        // FCC-800: gateway side is uplink + 45 MHz
        let (r, _, _) = parse_txpk(
            r#"{"txpk":{"imme":true,"freq":854.1,"modu":"LORA","datr":"SF7BW125","data":"AQ=="}}"#,
            &lut,
            0,
            0,
        )
        .unwrap();
        assert_eq!((r.region, r.freq_hz), (Region::Fcc800, 854_100_000));
        assert_eq!(
            parse_txpk(
                r#"{"txpk":{"imme":true,"freq":813.0,"modu":"LORA","datr":"SF7BW125","data":"AQ=="}}"#,
                &lut,
                0,
                0,
            )
            .unwrap_err(),
            "TX_FREQ"
        );
    }

    #[test]
    fn stat_reports_temp_clock_and_collisions() {
        let mut st = Stats::default();
        st.coll = 2;
        let v = stat_json("2026-01-01 00:00:00 GMT", &st, 100.0, Some(31.5), "host");
        let s = &v["stat"];
        assert_eq!(s["temp"], 31.5);
        assert_eq!(s["time_src"], "host");
        assert_eq!(s["coll"], 2);
        assert!(stat_json("", &Stats::default(), 0.0, None, "pps")["stat"]["temp"].is_null());
    }

    #[test]
    fn submit_charges_only_accepted_packets() {
        let mut jit = JitQueue::default();
        // 0.01 % of an hour = 360 ms
        let mut air = jit::Airtime::new(Some(400), Some(0.01));
        let mk = |sf: u8| {
            let mut r = TxRequest::as923_1_sf7(vec![0; 12]).unwrap();
            r.sf = sf;
            r
        };
        // SF12 breaks the dwell limit and is not charged
        assert_eq!(
            submit(&mut jit, &mut air, mk(12), Some(1_000_000), 0, jit::MAX_AHEAD_US, 0),
            Err(jit::JitError::Dwell)
        );
        // SF10 fits once; the second one is over budget, not a collision
        submit(&mut jit, &mut air, mk(10), Some(1_000_000), 0, jit::MAX_AHEAD_US, 0).unwrap();
        assert_eq!(
            submit(&mut jit, &mut air, mk(10), Some(3_000_000), 0, jit::MAX_AHEAD_US, 0),
            Err(jit::JitError::DutyCycle)
        );
        assert_eq!(jit.len(), 1);
        // a late packet is refused by the queue and leaves the budget alone
        let mut air = jit::Airtime::new(None, Some(0.01));
        assert_eq!(
            submit(&mut jit, &mut air, mk(10), Some(5_000), 0, jit::MAX_AHEAD_US, 0),
            Err(jit::JitError::TooLate)
        );
        submit(&mut jit, &mut air, mk(10), Some(3_000_000), 0, jit::MAX_AHEAD_US, 0).unwrap();
    }

    #[test]
    fn hop_blocks_from_config() {
        let cfg: Config = serde_json::from_str(
            r#"{"region":"US915","subband":0,"hop":{"dwell_s":20,"step_hz":1600000,"steps":8}}"#,
        )
        .unwrap();
        let base = ChannelPlan::preset(Region::parse(cfg.region.as_deref().unwrap()).unwrap(), cfg.subband).unwrap();
        let plans = hop_plans(&base, Region::Us915, &cfg.hop.unwrap()).unwrap();
        assert_eq!(plans.len(), 8);
        assert_eq!(plans[7], ChannelPlan::us915_subband(7).unwrap());

        // AS923-1: the default block and the one 1.6 MHz below
        let a = ChannelPlan::as923_1_default();
        let hop = |shift_hz: Vec<i64>| HopConfig { dwell_s: 10, shift_hz, ..Default::default() };
        let plans = hop_plans(&a, Region::As923_1, &hop(vec![0, -1_600_000])).unwrap();
        assert_eq!(plans[1].multisf[0].freq_hz, 921_000_000);
        // a block that leaves 920–925 MHz is refused, although it is inside US915
        assert!(hop_plans(&a, Region::As923_1, &hop(vec![0, 1_600_000])).is_err());
        assert!(hop_plans(&a, Region::As923_1, &hop(vec![0])).is_err());

        // FCC-800: five sub-bands, a sixth does not fit
        let f = ChannelPlan::fcc800_subband(0).unwrap();
        let steps = |steps: u32| HopConfig { dwell_s: 10, step_hz: 1_600_000, steps, ..Default::default() };
        assert_eq!(hop_plans(&f, Region::Fcc800, &steps(5)).unwrap().len(), 5);
        assert!(hop_plans(&f, Region::Fcc800, &steps(6)).is_err());
    }

    #[test]
    fn config_parses() {
        let s = std::fs::read_to_string(concat!(
            env!("CARGO_MANIFEST_DIR"),
            "/../../configs/sx1302-daemon.json"
        ))
        .unwrap();
        let c: Config = serde_json::from_str(&s).unwrap();
        c.plan.unwrap().validate(Region::As923_1).unwrap();
    }
}
