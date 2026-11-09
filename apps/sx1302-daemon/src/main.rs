//! AS923-1 concentrator daemon: Semtech GWMP v2 (PUSH_DATA / PULL_DATA / PULL_RESP / TX_ACK)
//! with a JIT downlink queue and a JSON channel plan.

mod jit;

use clap::Parser;
use jit::JitQueue;
use sensecap_board::reset_sensecap_m1;
use serde::Deserialize;
use serde_json::json;
use sx1302_core::Region;
use sx1302_hal::{bw_hz_to_code, ChannelPlan, Concentrator, FskTx, TxRequest, TxTrigger};
use sx1302_transport::SpiConfig;
use std::net::UdpSocket;
use std::thread;
use std::time::{Duration, Instant};

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
    plan.validate(Region::As923_1)?;

    match reset_sensecap_m1() {
        Ok(n) => n.iter().for_each(|l| eprintln!("{l}")),
        Err(e) => eprintln!("gpio: {e}"),
    }
    let c = Concentrator::open(SpiConfig {
        path: args.spi,
        ..Default::default()
    })?;
    for line in c.start(&plan)? {
        eprintln!("{line}");
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
    let mut token: u16 = 1;
    let mut last_stat = Instant::now();
    let mut last_pull = Instant::now() - Duration::from_secs(10);
    let mut pull_acked = false;
    loop {
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
            let rxpk: Vec<_> = ok.iter().map(|p| p.to_rxpk()).collect();
            if !rxpk.is_empty() {
                let body = json!({ "rxpk": rxpk });
                println!("{body}");
                if let Some(s) = &sock {
                    push(s, token, eui, &body.to_string())?;
                    st.rxfw += rxpk.len() as u32;
                    st.push_sent += 1;
                    token = token.wrapping_add(1);
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
                let stat = json!({
                    "stat": {
                        "time": gwmp_time(),
                        "rxnb": st.rxnb,
                        "rxok": st.rxok,
                        "rxfw": st.rxfw,
                        "ackr": (ackr * 10.0).round() / 10.0,
                        "dwnb": st.dwnb,
                        "txnb": st.txnb
                    }
                });
                eprintln!("{stat}");
                push(s, token, eui, &stat.to_string())?;
                st = Stats {
                    push_sent: st.push_sent + 1,
                    push_ack: st.push_ack,
                    ..Stats::default()
                };
                token = token.wrapping_add(1);
                last_stat = Instant::now();
            }
            if last_pull.elapsed() > Duration::from_secs(5) {
                s.send(&gwmp_hdr(PKT_PULL_DATA, token, eui))?;
                token = token.wrapping_add(1);
                last_pull = Instant::now();
            }
            let mut buf = [0u8; 65536];
            while let Ok(n) = s.recv(&mut buf) {
                if n < 4 || buf[0] != PROTO {
                    continue;
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
                        let err = match parse_txpk(json) {
                            Ok((req, start)) => {
                                let now = c.now_us()?;
                                match jit.enqueue(req, start, now) {
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
                        tx_ack(s, resp_token, eui, err)?;
                    }
                    _ => {}
                }
            }
        }
        thread::sleep(Duration::from_millis(5));
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

/// GWMP `txpk` -> request + emission time (`None` = immediate). Errors are GWMP codes.
fn parse_txpk(json: &str) -> Result<(TxRequest, Option<u32>), &'static str> {
    let v: serde_json::Value = serde_json::from_str(json).map_err(|_| "COLLISION_PACKET")?;
    let tx = v.get("txpk").ok_or("COLLISION_PACKET")?;
    let freq = tx.get("freq").and_then(|x| x.as_f64()).ok_or("TX_FREQ")?;
    let freq_hz = (freq * 1_000_000.0).round() as u32;
    if !Region::As923_1.contains(freq_hz) {
        return Err("TX_FREQ");
    }
    let payload = b64_decode(tx.get("data").and_then(|x| x.as_str()).unwrap_or(""))
        .map_err(|_| "COLLISION_PACKET")?;
    let imme = tx.get("imme").and_then(|x| x.as_bool()).unwrap_or(false);
    let start = if imme {
        None
    } else {
        // GPS `tmms` scheduling needs PPS (not on this carrier).
        Some(tx.get("tmst").and_then(|x| x.as_u64()).ok_or("GPS_UNLOCKED")? as u32)
    };
    let modu = tx.get("modu").and_then(|x| x.as_str()).unwrap_or("LORA");
    let mut req = TxRequest::as923_1_sf7(payload).map_err(|_| "TX_FREQ")?;
    req.freq_hz = freq_hz;
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
    Ok((req, start))
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
        let (r, t) = parse_txpk(
            r#"{"txpk":{"imme":false,"tmst":5000000,"freq":923.2,"modu":"LORA","datr":"SF10BW125","codr":"4/5","ipol":true,"data":"AQID"}}"#,
        )
        .unwrap();
        assert_eq!((r.sf, r.invert_pol, t), (10, true, Some(5_000_000)));
        let (r, t) = parse_txpk(
            r#"{"txpk":{"imme":true,"freq":922.3,"modu":"FSK","datr":50000,"fdev":25000,"data":"AQID"}}"#,
        )
        .unwrap();
        assert_eq!(t, None);
        assert_eq!(r.fsk.unwrap().datarate, 50_000);
        assert_eq!(
            parse_txpk(r#"{"txpk":{"tmst":1,"freq":868.1,"modu":"LORA","datr":"SF7BW125","data":"AQ=="}}"#)
                .unwrap_err(),
            "TX_FREQ"
        );
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
