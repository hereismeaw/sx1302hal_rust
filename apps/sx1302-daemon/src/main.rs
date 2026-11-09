//! AS923-1 concentrator daemon: Semtech GWMP v2 (PUSH_DATA / PULL_DATA / PULL_RESP / TX_ACK).

use clap::Parser;
use sensecap_board::reset_sensecap_m1;
use serde_json::json;
use sx1302_core::Region;
use sx1302_hal::{Concentrator, TxRequest, TxTrigger, BW_125KHZ};
use sx1302_transport::SpiConfig;
use std::net::UdpSocket;
use std::thread;
use std::time::{Duration, Instant};

#[derive(Parser)]
#[command(name = "sx1302-daemon", about = "Native SX1302 GWMP forwarder (AS923-1)")]
struct Args {
    /// ChirpStack / Semtech packet-forwarder UDP (`host:port`). Empty = stdout JSON only.
    #[arg(long, default_value = "")]
    server: String,
    #[arg(long, default_value = "/dev/spidev0.0")]
    spi: String,
    /// Seconds between `stat` reports.
    #[arg(long, default_value_t = 30)]
    stat_interval: u64,
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
    match reset_sensecap_m1() {
        Ok(n) => n.iter().for_each(|l| eprintln!("{l}")),
        Err(e) => eprintln!("gpio: {e}"),
    }
    let c = Concentrator::open(SpiConfig {
        path: args.spi,
        ..Default::default()
    })?;
    for line in c.start_as923_1()? {
        eprintln!("{line}");
    }
    let eui = c.otp_eui().unwrap_or(0);
    eprintln!("gateway EUI {eui:016x}");
    let sock = if args.server.is_empty() {
        None
    } else {
        let s = UdpSocket::bind("0.0.0.0:0")?;
        s.set_nonblocking(true)?;
        s.connect(&args.server)?;
        eprintln!("PUSH_DATA/PULL_DATA -> {}", args.server);
        Some(s)
    };
    let mut st = Stats::default();
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
                    p.count_us.abs_diff(*t) < 1_000 && *d == p.payload.as_slice()
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
        if let Some(s) = &sock {
            if last_stat.elapsed() > Duration::from_secs(args.stat_interval) {
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
                st.push_sent += 1;
                token = token.wrapping_add(1);
                st.rxnb = 0;
                st.rxok = 0;
                st.rxfw = 0;
                st.dwnb = 0;
                st.txnb = 0;
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
                            eprintln!("PULL_ACK from {} (downlink path open)", args.server);
                            pull_acked = true;
                        }
                    }
                    PKT_PULL_RESP => {
                        st.dwnb += 1;
                        let resp_token = u16::from_be_bytes([buf[1], buf[2]]);
                        let json = std::str::from_utf8(&buf[4..n]).unwrap_or("");
                        eprintln!("PULL_RESP {json}");
                        let err = match handle_txpk(&c, json) {
                            Ok(msg) => {
                                eprintln!("{msg}");
                                st.txnb += 1;
                                "NONE"
                            }
                            Err(e) => {
                                eprintln!("txpk: {e}");
                                gwmp_error(&e.to_string())
                            }
                        };
                        tx_ack(s, resp_token, eui, err)?;
                    }
                    _ => {}
                }
            }
        }
        thread::sleep(Duration::from_millis(10));
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

fn gwmp_error(msg: &str) -> &'static str {
    if msg.contains("TOO_LATE") {
        "TOO_LATE"
    } else if msg.contains("TOO_EARLY") {
        "TOO_EARLY"
    } else if msg.contains("busy") {
        "COLLISION_PACKET"
    } else if msg.contains("band") || msg.contains("Frequency") {
        "TX_FREQ"
    } else {
        "COLLISION_PACKET"
    }
}

fn handle_txpk(c: &Concentrator, json: &str) -> Result<String, Box<dyn std::error::Error>> {
    let v: serde_json::Value = serde_json::from_str(json)?;
    let tx = v.get("txpk").ok_or("no txpk")?;
    if tx.get("modu").and_then(|x| x.as_str()).unwrap_or("LORA") != "LORA" {
        return Err("only LORA modulation".into());
    }
    let freq = tx.get("freq").and_then(|x| x.as_f64()).ok_or("no freq")?;
    let freq_hz = (freq * 1_000_000.0).round() as u32;
    let datr = tx.get("datr").and_then(|x| x.as_str()).unwrap_or("SF7BW125");
    let (sf, bw) = parse_datr(datr).ok_or("bad datr")?;
    if bw != 125 {
        return Err(format!("only BW125 supported ({datr})").into());
    }
    let b64 = tx.get("data").and_then(|x| x.as_str()).unwrap_or("");
    let payload = b64_decode(b64)?;
    let imme = tx.get("imme").and_then(|x| x.as_bool()).unwrap_or(false);
    let trigger = if imme {
        TxTrigger::Immediate
    } else {
        let t = tx.get("tmst").and_then(|x| x.as_u64()).ok_or("no tmst (GPS time not supported)")?;
        TxTrigger::Timestamped(t as u32)
    };
    let req = TxRequest {
        region: Region::As923_1,
        freq_hz,
        sf,
        bw: BW_125KHZ,
        cr: 1,
        payload,
        power_idx: 22,
        invert_pol: tx.get("ipol").and_then(|x| x.as_bool()).unwrap_or(false),
        preamble: tx.get("prea").and_then(|x| x.as_u64()).unwrap_or(8) as u16,
        trigger,
    };
    Ok(c.send_lora(&req)?)
}

fn parse_datr(d: &str) -> Option<(u8, u32)> {
    let rest = d.strip_prefix("SF")?;
    let (sf, bw) = rest.split_once("BW")?;
    Some((sf.parse().ok()?, bw.parse().ok()?))
}

fn b64_decode(s: &str) -> Result<Vec<u8>, Box<dyn std::error::Error>> {
    fn val(c: u8) -> Result<u8, Box<dyn std::error::Error>> {
        Ok(match c {
            b'A'..=b'Z' => c - b'A',
            b'a'..=b'z' => c - b'a' + 26,
            b'0'..=b'9' => c - b'0' + 52,
            b'+' => 62,
            b'/' => 63,
            _ => return Err("bad base64".into()),
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
}
