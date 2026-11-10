//! AS923-1 concentrator daemon: Semtech GWMP v2 (PUSH_DATA / PULL_DATA / PULL_RESP / TX_ACK)
//! with a JIT downlink queue and a JSON channel plan.

mod gps;
mod jit;
mod notify;

use clap::Parser;
use jit::JitQueue;
use sensecap_board::{reset_carrier, Carrier};
use serde::Deserialize;
use serde_json::json;
use sx1302_core::Region;
use sx1302_hal::{
    beacon_antenna, beacon_payload, bw_hz_to_code, count_at, gps_ms, next_beacon_gps_ms, ChannelPlan,
    Concentrator, FskTx, TxGainLut, TxRequest, TxTrigger, AS923_1_BEACON_HZ, BEACON_DELAY_US,
};
use sx1302_transport::SpiConfig;
use std::net::UdpSocket;
use std::thread;
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
}

#[derive(Deserialize, Default)]
struct Config {
    #[serde(default)]
    server: Option<String>,
    #[serde(default)]
    stat_interval: Option<u64>,
    #[serde(default)]
    plan: Option<ChannelPlan>,
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
}

struct GpsLock {
    gps_ms: u64,
    count_us: u32,
    lat_deg: f64,
    lon_deg: f64,
    at: Instant,
}

const PKT_PUSH_DATA: u8 = 0x00;
const PKT_PUSH_ACK: u8 = 0x01;
const PKT_PULL_DATA: u8 = 0x02;
const PKT_PULL_RESP: u8 = 0x03;
const PKT_PULL_ACK: u8 = 0x04;
const PKT_TX_ACK: u8 = 0x05;
const PROTO: u8 = 0x02;

#[derive(Default)]
struct Stats {
    rxnb: u32,
    rxok: u32,
    rxfw: u32,
    push_sent: u32,
    push_ack: u32,
    dwnb: u32,
    txnb: u32,
    /// Downlinks refused because the JIT window overlapped a queued packet.
    coll: u32,
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
    let server = args.server.or(cfg.server).unwrap_or_default();
    let stat_interval = args.stat_interval.or(cfg.stat_interval).unwrap_or(30);
    let plan = cfg.plan.unwrap_or_else(ChannelPlan::as923_1_default);
    let lut = cfg.tx_gain_lut.unwrap_or_default();
    plan.band()?;

    let carrier = Carrier::parse(cfg.board.as_deref().unwrap_or(""))?;
    match reset_carrier(carrier) {
        Ok(n) => n.iter().for_each(|l| eprintln!("{l}")),
        Err(e) => return Err(e.into()),
    }
    let c = Concentrator::open(SpiConfig {
        path: args.spi,
        ..Default::default()
    })?;
    for line in c.start(&plan)? {
        eprintln!("{line}");
    }
    if carrier == Carrier::Rak5146 {
        match c.configure_timestamp(true) {
            Ok(n) => n.iter().for_each(|l| eprintln!("{l}")),
            Err(e) => eprintln!("fine timestamp: {e}"),
        }
    }
    let eui = c.otp_eui().unwrap_or(0);
    eprintln!("gateway EUI {eui:016x}");
    let sock = if server.is_empty() {
        None
    } else {
        let s = UdpSocket::bind("0.0.0.0:0")?;
        s.set_nonblocking(true)?;
        s.connect(&server)?;
        eprintln!("PUSH_DATA/PULL_DATA -> {server}");
        Some(s)
    };
    let mut st = Stats::default();
    let mut jit = JitQueue::default();
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
    let mut last_pull = Instant::now() - Duration::from_secs(10);
    let mut pull_acked = false;
    // systemd watchdog: ping at half the interval, only after a passing health check.
    let wd_every = notify::watchdog_interval().map(|d| d / 2);
    let mut last_health = Instant::now();
    let mut health_ts = Some(c.health_check(None)?);
    let mut last_server_ack = Instant::now();
    let mut server_lost = false;
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
            health_ts = Some(c.health_check(health_ts).map_err(|e| format!("health check: {e}"))?);
            last_health = Instant::now();
            let temp = read_temp(&c);
            if wd_every.is_some() {
                let link = if sock.is_none() || last_server_ack.elapsed() < Duration::from_secs(30) {
                    "server ok"
                } else {
                    "no server ACK > 30 s"
                };
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
            // Like lora_pkt_fwd: forward CRC-OK only. The multi-SF chain and the LoRa
            // service modem can both demodulate one frame: keep the best-SNR copy.
            let mut ok: Vec<_> = dump.packets.iter().filter(|p| p.stat == 1).collect();
            ok.sort_by(|a, b| b.snr.total_cmp(&a.snr));
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
                let body = json!({ "rxpk": rxpk });
                println!("{body}");
                if let Some(s) = &sock {
                    soft(push(s, token, eui, &body.to_string()), "PUSH_DATA rxpk");
                    st.rxfw += rxpk.len() as u32;
                    st.push_sent += 1;
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
                    // Arm window is 100 ms; the TX timer rejects > 10 s at arm time,
                    // so queue once the beacon is a few seconds out.
                    if (2_000_000..9_000_000).contains(&ahead) {
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
                        match jit.enqueue_within(req, Some(at), now, jit::MAX_AHEAD_CLASS_B_US) {
                            Ok(()) => {
                                last_beacon_ms = boundary;
                                eprintln!("class B beacon queued gps_ms={boundary} tmst={at}");
                            }
                            Err(e) => {
                                if e == jit::JitError::Collision {
                                    st.coll += 1;
                                }
                                eprintln!("class B beacon: {}", e.gwmp());
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
                        eprintln!("{msg} [armed in {} µs, queue {}]", t0.elapsed().as_micros(), jit.len());
                    }
                    Err(e) => eprintln!("jit arm: {e}"),
                }
            }
        }

        if let Some(s) = &sock {
            if last_stat.elapsed() > Duration::from_secs(stat_interval) {
                let ackr = if st.push_sent == 0 {
                    100.0
                } else {
                    (100.0 * f64::from(st.push_ack) / f64::from(st.push_sent)).min(100.0)
                };
                let stat = stat_json(
                    &gwmp_time(),
                    &st,
                    (ackr * 10.0).round() / 10.0,
                    read_temp(&c),
                    if gps_lock.is_some() { "pps" } else { "host" },
                );
                eprintln!("{stat}");
                soft(push(s, token, eui, &stat.to_string()), "PUSH_DATA stat");
                st = Stats {
                    push_sent: st.push_sent + 1,
                    push_ack: st.push_ack,
                    ..Stats::default()
                };
                token = token.wrapping_add(1);
                last_stat = Instant::now();
            }
            if last_pull.elapsed() > Duration::from_secs(5) {
                soft(s.send(&gwmp_hdr(PKT_PULL_DATA, token, eui)).map(|_| ()), "PULL_DATA");
                token = token.wrapping_add(1);
                last_pull = Instant::now();
            }
            if !server_lost && last_server_ack.elapsed() > Duration::from_secs(30) {
                eprintln!("server {server}: no ACK for 30 s (link down?), still receiving RF");
                server_lost = true;
            }
            let mut buf = [0u8; 65536];
            while let Ok(n) = s.recv(&mut buf) {
                if n < 4 || buf[0] != PROTO {
                    continue;
                }
                if matches!(buf[3], PKT_PUSH_ACK | PKT_PULL_ACK | PKT_PULL_RESP) {
                    if server_lost {
                        eprintln!(
                            "server {server} back after {} s",
                            last_server_ack.elapsed().as_secs()
                        );
                        server_lost = false;
                    }
                    last_server_ack = Instant::now();
                }
                match buf[3] {
                    PKT_PUSH_ACK => st.push_ack += 1,
                    PKT_PULL_ACK => {
                        if !pull_acked {
                            eprintln!("PULL_ACK from {server} (downlink path open)");
                            pull_acked = true;
                        }
                    }
                    PKT_PULL_RESP => {
                        st.dwnb += 1;
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
                                match jit.enqueue_within(req, start, now, horizon) {
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
                                        eprintln!("jit: {}", e.gwmp());
                                        e.gwmp()
                                    }
                                }
                            }
                            Err(e) => {
                                eprintln!("txpk: {e}");
                                e
                            }
                        };
                        soft(tx_ack(s, resp_token, eui, err), "TX_ACK");
                    }
                    _ => {}
                }
            }
        }
        thread::sleep(Duration::from_millis(5));
    }
}

/// Network errors (ECONNREFUSED from a dead server, ENETUNREACH while the link is
/// down, ...) must not kill the concentrator: log at most every 30 s and carry on.
fn soft(r: std::io::Result<()>, what: &str) {
    static LAST: std::sync::Mutex<Option<Instant>> = std::sync::Mutex::new(None);
    if let Err(e) = r {
        let mut last = LAST.lock().unwrap();
        if last.is_none_or(|t| t.elapsed() > Duration::from_secs(30)) {
            eprintln!("{what}: {e} (continuing; packets dropped until the server is reachable)");
            *last = Some(Instant::now());
        }
    }
}

fn gwmp_hdr(ident: u8, token: u16, eui: u64) -> Vec<u8> {
    let mut pkt = Vec::with_capacity(12);
    pkt.push(PROTO);
    pkt.extend_from_slice(&token.to_be_bytes());
    pkt.push(ident);
    pkt.extend_from_slice(&eui.to_be_bytes());
    pkt
}

/// TX_ACK must echo the PULL_RESP token (PROTOCOL.md §6.2).
fn tx_ack(sock: &UdpSocket, token: u16, eui: u64, error: &str) -> std::io::Result<()> {
    let mut pkt = gwmp_hdr(PKT_TX_ACK, token, eui);
    pkt.extend_from_slice(json!({ "txpk_ack": { "error": error } }).to_string().as_bytes());
    sock.send(&pkt)?;
    Ok(())
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
    let region = if Region::As923_1.contains(freq_hz) {
        Region::As923_1
    } else if Region::Us915.contains(freq_hz) {
        Region::Us915
    } else if Region::Fcc800.contains(freq_hz) {
        // shortcut: licensed 809–817 MHz, no Alliance raster, no SX1250 image-cal row.
        eprintln!("txpk {freq_hz} Hz is FCC-800 (809–817); SX1250 image cal has no row there");
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

fn push(sock: &UdpSocket, token: u16, eui: u64, json: &str) -> std::io::Result<()> {
    let mut pkt = gwmp_hdr(PKT_PUSH_DATA, token, eui);
    pkt.extend_from_slice(json.as_bytes());
    sock.send(&pkt)?;
    Ok(())
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
    format_utc(secs)
}

fn format_utc(secs: u64) -> String {
    let days = (secs / 86_400) as i64;
    let tod = secs % 86_400;
    // Howard Hinnant civil_from_days
    let z = days + 719_468;
    let era = z.div_euclid(146_097);
    let doe = z - era * 146_097;
    let yoe = (doe - doe / 1460 + doe / 36_524 - doe / 146_096) / 365;
    let doy = doe - (365 * yoe + yoe / 4 - yoe / 100);
    let mp = (5 * doy + 2) / 153;
    let d = doy - (153 * mp + 2) / 5 + 1;
    let m = if mp < 10 { mp + 3 } else { mp - 9 };
    let y = yoe + era * 400 + i64::from(m <= 2);
    format!(
        "{y:04}-{m:02}-{d:02} {:02}:{:02}:{:02} GMT",
        tod / 3600,
        (tod / 60) % 60,
        tod % 60
    )
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn utc_format() {
        assert_eq!(format_utc(0), "1970-01-01 00:00:00 GMT");
        assert_eq!(format_utc(1_709_210_096), "2024-02-29 12:34:56 GMT");
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
