//! AS923-1 concentrator daemon: Semtech GWMP PUSH_DATA to a packet-forwarder UDP port.

use clap::Parser;
use sensecap_board::reset_sensecap_m1;
use serde_json::json;
use sx1302_core::Region;
use sx1302_hal::{Concentrator, TxRequest, BW_125KHZ};
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
}

const PKT_PUSH_DATA: u8 = 0x00;
const PKT_PULL_DATA: u8 = 0x02;
const PKT_PULL_RESP: u8 = 0x03;
const PKT_TX_ACK: u8 = 0x05;
const PROTO: u8 = 0x02;

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
    let mut token: u16 = 1;
    let mut last_stat = Instant::now();
    let mut last_pull = Instant::now() - Duration::from_secs(10);
    loop {
        let dump = c.rx_poll()?;
        if !dump.packets.is_empty() {
            let rxpk: Vec<_> = dump.packets.iter().map(|p| p.to_rxpk()).collect();
            let body = json!({ "rxpk": rxpk });
            println!("{body}");
            if let Some(s) = &sock {
                push(s, token, eui, &body.to_string())?;
                token = token.wrapping_add(1);
            }
        }
        if last_stat.elapsed() > Duration::from_secs(30) {
            let stat = json!({
                "stat": {
                    "time": chrono_like(),
                    "rxnb": 0,
                    "rxok": 0,
                    "rxfw": 0,
                    "ackr": 100.0,
                    "dwnb": 0,
                    "txnb": 0
                }
            });
            if let Some(s) = &sock {
                push(s, token, eui, &stat.to_string())?;
                token = token.wrapping_add(1);
            }
            last_stat = Instant::now();
        }
        if let Some(s) = &sock {
            if last_pull.elapsed() > Duration::from_secs(8) {
                pull(s, token, eui)?;
                token = token.wrapping_add(1);
                last_pull = Instant::now();
            }
            let mut buf = [0u8; 4096];
            match s.recv(&mut buf) {
                Ok(n) if n >= 4 && buf[3] == PKT_PULL_RESP => {
                    if let Some(json) = std::str::from_utf8(&buf[4..n]).ok() {
                        match handle_txpk(&c, json) {
                            Ok(msg) => {
                                eprintln!("{msg}");
                                tx_ack(s, token, eui, true)?;
                            }
                            Err(e) => {
                                eprintln!("txpk: {e}");
                                tx_ack(s, token, eui, false)?;
                            }
                        }
                        token = token.wrapping_add(1);
                    }
                }
                Ok(_) | Err(_) => {}
            }
        }
        thread::sleep(Duration::from_millis(20));
    }
}

fn gwmp_hdr(ident: u8, token: u16, eui: u64) -> Vec<u8> {
    let mut pkt = Vec::with_capacity(12);
    pkt.push(PROTO);
    pkt.push((token >> 8) as u8);
    pkt.push(token as u8);
    pkt.push(ident);
    pkt.extend_from_slice(&eui.to_be_bytes());
    pkt
}

fn pull(sock: &UdpSocket, token: u16, eui: u64) -> std::io::Result<()> {
    sock.send(&gwmp_hdr(PKT_PULL_DATA, token, eui))?;
    Ok(())
}

fn tx_ack(sock: &UdpSocket, token: u16, eui: u64, ok: bool) -> std::io::Result<()> {
    let mut pkt = gwmp_hdr(PKT_TX_ACK, token, eui);
    let body = if ok {
        "{}".to_string()
    } else {
        r#"{"txpk_ack":{"error":"NONE"}}"#.to_string()
    };
    pkt.extend_from_slice(body.as_bytes());
    sock.send(&pkt)?;
    Ok(())
}

fn handle_txpk(c: &Concentrator, json: &str) -> Result<String, Box<dyn std::error::Error>> {
    let v: serde_json::Value = serde_json::from_str(json)?;
    let tx = v.get("txpk").ok_or("no txpk")?;
    let freq = tx.get("freq").and_then(|x| x.as_f64()).unwrap_or(923.2);
    let freq_hz = (freq * 1_000_000.0).round() as u32;
    let datr = tx.get("datr").and_then(|x| x.as_str()).unwrap_or("SF7BW125");
    let sf = datr
        .trim_start_matches("SF")
        .split('B')
        .next()
        .and_then(|s| s.parse::<u8>().ok())
        .unwrap_or(7);
    let b64 = tx.get("data").and_then(|x| x.as_str()).unwrap_or("");
    let payload = b64_decode(b64)?;
    let req = TxRequest {
        region: Region::As923_1,
        freq_hz,
        sf,
        bw: BW_125KHZ,
        cr: 1,
        payload,
        power_idx: 22,
    };
    Ok(c.send_lora(&req)?)
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
    let mut pkt = Vec::with_capacity(12 + json.len());
    pkt.push(PROTO);
    pkt.push((token >> 8) as u8);
    pkt.push(token as u8);
    pkt.push(PKT_PUSH_DATA);
    pkt.extend_from_slice(&eui.to_be_bytes());
    pkt.extend_from_slice(json.as_bytes());
    sock.send(&pkt)?;
    Ok(())
}

fn chrono_like() -> String {
    // UTC compact without extra crate: UNIX is enough for logs; GWMP prefers ISO8601.
    let secs = std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .map(|d| d.as_secs())
        .unwrap_or(0);
    format!("{secs}")
}
