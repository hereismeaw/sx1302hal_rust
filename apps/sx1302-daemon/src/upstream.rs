//! One network server: its UDP socket, the link state, and a spool of uplinks
//! the server has not acknowledged yet.
//!
//! A PUSH_DATA with `rxpk` stays in flight until its PUSH_ACK arrives. Without an
//! ACK after [`ACK_TIMEOUT`], or while the link is down, it goes to the backlog.
//! The backlog is sent again, oldest first, once the server answers again. Each
//! replayed `rxpk` gets a `time` field from when it was received, so the server
//! does not file it under the moment it arrived.
//!
//! Only data uplinks are kept. A JoinRequest or Rejoin-Request that arrives late
//! can make the network server start a session the device never sees, so those
//! are dropped like before.

use crate::utc::format_iso;
use serde_json::{json, Value};
use std::collections::VecDeque;
use std::io;
use std::net::UdpSocket;
use std::time::{Duration, Instant};

pub const PKT_PUSH_DATA: u8 = 0x00;
pub const PKT_PUSH_ACK: u8 = 0x01;
pub const PKT_PULL_DATA: u8 = 0x02;
pub const PKT_PULL_RESP: u8 = 0x03;
pub const PKT_PULL_ACK: u8 = 0x04;
pub const PKT_TX_ACK: u8 = 0x05;
pub const PROTO: u8 = 0x02;

/// An `rxpk` PUSH_DATA without PUSH_ACK after this long goes to the backlog.
pub const ACK_TIMEOUT: Duration = Duration::from_secs(5);
/// No ACK of any kind for this long: the link is down (the soak sampler greps the log line).
pub const LINK_TIMEOUT: Duration = Duration::from_secs(30);
const PULL_EVERY: Duration = Duration::from_secs(5);
/// Replay pace: one PUSH_DATA every 50 ms, at most 8 waiting for their ACK.
const REPLAY_EVERY: Duration = Duration::from_millis(50);
const REPLAY_INFLIGHT: usize = 8;
/// Default backlog size in PUSH_DATA messages.
pub const DEFAULT_BUFFER: usize = 2000;

/// LoRaWAN MType of a PHYPayload: 010 unconfirmed up, 100 confirmed up, 111 proprietary.
pub fn replayable_phy(payload: &[u8]) -> bool {
    matches!(payload.first().map(|b| b >> 5), Some(0b010 | 0b100 | 0b111))
}

/// The `rxpk` objects of one PUSH_DATA, with when they were received.
#[derive(Clone, Debug, PartialEq)]
pub struct Uplink {
    pub rxpk: Vec<Value>,
    /// Per `rxpk`: may it be sent late (see [`replayable_phy`]).
    pub keep: Vec<bool>,
    pub unix_ms: u64,
}

impl Uplink {
    /// The part that may go out late, or `None` when nothing is left.
    fn replayable(self) -> Option<Uplink> {
        let (rxpk, keep): (Vec<_>, Vec<_>) =
            self.rxpk.into_iter().zip(self.keep).filter(|(_, k)| *k).unzip();
        (!rxpk.is_empty()).then_some(Uplink { rxpk, keep, unix_ms: self.unix_ms })
    }

    /// PUSH_DATA body for a late send: every `rxpk` carries its receive `time`.
    fn late_body(&self) -> Value {
        let time = format_iso(self.unix_ms);
        let rxpk: Vec<Value> = self
            .rxpk
            .iter()
            .map(|r| {
                let mut r = r.clone();
                if let Some(o) = r.as_object_mut() {
                    o.entry("time").or_insert_with(|| json!(time));
                }
                r
            })
            .collect();
        json!({ "rxpk": rxpk })
    }
}

#[derive(Debug, Default)]
pub struct Spool {
    inflight: VecDeque<(u16, Instant, Uplink)>,
    backlog: VecDeque<Uplink>,
    cap: usize,
    /// Uplinks lost: backlog full, buffer off, or a join that cannot wait.
    pub dropped: u64,
    pub replayed: u64,
}

impl Spool {
    pub fn new(cap: usize) -> Self {
        Spool { cap, ..Default::default() }
    }

    pub fn backlog_len(&self) -> usize {
        self.backlog.len()
    }

    /// Keep `u` for later. The oldest entry goes when the backlog is full.
    pub fn defer(&mut self, u: Uplink) {
        let n = u.rxpk.len() as u64;
        if self.cap == 0 {
            self.dropped += n;
            return;
        }
        let Some(u) = u.replayable() else {
            self.dropped += n;
            return;
        };
        self.dropped += n - u.rxpk.len() as u64;
        self.backlog.push_back(u);
        self.trim();
    }

    fn trim(&mut self) {
        while self.backlog.len() > self.cap {
            if let Some(u) = self.backlog.pop_front() {
                self.dropped += u.rxpk.len() as u64;
            }
        }
    }

    pub fn sent(&mut self, token: u16, u: Uplink, now: Instant) {
        self.inflight.push_back((token, now, u));
    }

    pub fn acked(&mut self, token: u16) -> bool {
        match self.inflight.iter().position(|(t, _, _)| *t == token) {
            Some(i) => {
                self.inflight.remove(i);
                true
            }
            None => false,
        }
    }

    /// Move unacknowledged uplinks to the backlog, which stays in receive order.
    pub fn expire(&mut self, now: Instant) -> usize {
        let mut moved = 0;
        while self
            .inflight
            .front()
            .is_some_and(|(_, t, _)| now.duration_since(*t) >= ACK_TIMEOUT)
        {
            if let Some((_, _, u)) = self.inflight.pop_front() {
                self.defer(u);
                moved += 1;
            }
        }
        if moved > 0 {
            self.backlog.make_contiguous().sort_by_key(|u| u.unix_ms);
        }
        moved
    }

    /// Next backlog entry to replay, unless too many are still waiting for an ACK.
    fn next_replay(&mut self) -> Option<Uplink> {
        if self.inflight.len() >= REPLAY_INFLIGHT {
            return None;
        }
        self.backlog.pop_front()
    }

    fn put_back(&mut self, u: Uplink) {
        self.backlog.push_front(u);
    }
}

pub struct Upstream {
    pub addr: String,
    /// `None` until the name resolves (a Tailscale or DNS name may not exist yet at boot).
    sock: Option<UdpSocket>,
    last_connect: Instant,
    pub last_ack: Instant,
    pub lost: bool,
    pub pull_acked: bool,
    last_pull: Instant,
    last_replay: Instant,
    last_warn: Option<Instant>,
    pub spool: Spool,
    pub push_sent: u64,
    pub push_ack: u64,
    /// Times the link went down.
    pub outages: u64,
}

impl Upstream {
    /// Never fails on the address: a name that does not resolve yet is tried again
    /// every [`LINK_TIMEOUT`], and uplinks are kept meanwhile.
    pub fn connect(addr: &str, buffer: usize) -> io::Result<Self> {
        let now = Instant::now();
        let mut u = Upstream {
            addr: addr.to_string(),
            sock: None,
            last_connect: now,
            last_ack: now,
            lost: false,
            pull_acked: false,
            last_pull: now - PULL_EVERY * 2,
            last_replay: now,
            last_warn: None,
            spool: Spool::new(buffer),
            push_sent: 0,
            push_ack: 0,
            outages: 0,
        };
        if let Err(e) = u.open() {
            eprintln!("server {addr}: {e} (retrying every {} s, uplinks kept)", LINK_TIMEOUT.as_secs());
        }
        Ok(u)
    }

    fn open(&mut self) -> io::Result<()> {
        self.last_connect = Instant::now();
        let sock = UdpSocket::bind("0.0.0.0:0")?;
        sock.set_nonblocking(true)?;
        sock.connect(&self.addr)?;
        self.sock = Some(sock);
        Ok(())
    }

    /// ENETUNREACH while the cable is out, ECONNREFUSED from a dead server, ...:
    /// log at most every 30 s and carry on.
    fn warn(&mut self, what: &str, e: &io::Error) {
        if self.last_warn.is_none_or(|t| t.elapsed() > LINK_TIMEOUT) {
            let backlog = self.spool.backlog_len();
            eprintln!(
                "{} {what}: {e} (continuing; uplinks kept for later, backlog {backlog})",
                self.addr
            );
            self.last_warn = Some(Instant::now());
        }
    }

    fn send(&self, ident: u8, token: u16, eui: u64, body: Option<&str>) -> io::Result<()> {
        let Some(sock) = &self.sock else {
            return Err(io::Error::new(io::ErrorKind::NotConnected, "address not resolved yet"));
        };
        let mut pkt = hdr(ident, token, eui);
        if let Some(b) = body {
            pkt.extend_from_slice(b.as_bytes());
        }
        sock.send(&pkt).map(|_| ())
    }

    /// Forward a new uplink, or keep it while the link is down or older ones wait.
    pub fn send_rxpk(&mut self, token: u16, eui: u64, u: Uplink) {
        if self.lost || self.spool.backlog_len() > 0 {
            self.spool.defer(u);
            return;
        }
        let body = json!({ "rxpk": u.rxpk }).to_string();
        match self.send(PKT_PUSH_DATA, token, eui, Some(&body)) {
            Ok(()) => {
                self.push_sent += 1;
                self.spool.sent(token, u, Instant::now());
            }
            Err(e) => {
                self.warn("PUSH_DATA rxpk", &e);
                self.spool.defer(u);
            }
        }
    }

    pub fn send_stat(&mut self, token: u16, eui: u64, body: &str) {
        match self.send(PKT_PUSH_DATA, token, eui, Some(body)) {
            Ok(()) => self.push_sent += 1,
            Err(e) => self.warn("PUSH_DATA stat", &e),
        }
    }

    pub fn tx_ack(&mut self, token: u16, eui: u64, error: &str) {
        let body = json!({ "txpk_ack": { "error": error } }).to_string();
        if let Err(e) = self.send(PKT_TX_ACK, token, eui, Some(&body)) {
            self.warn("TX_ACK", &e);
        }
    }

    /// Up-time PUSH_ACK ratio in percent, one decimal.
    pub fn ackr(&self) -> f64 {
        if self.push_sent == 0 {
            return 100.0;
        }
        let r = (100.0 * self.push_ack as f64 / self.push_sent as f64).min(100.0);
        (r * 10.0).round() / 10.0
    }

    /// PULL_DATA keep-alive, link-down detection, ACK timeouts and replay.
    pub fn tick(&mut self, token: &mut u16, eui: u64) {
        let now = Instant::now();
        if self.sock.is_none() && now.duration_since(self.last_connect) >= LINK_TIMEOUT {
            match self.open() {
                Ok(()) => eprintln!("server {}: address resolved", self.addr),
                Err(e) => self.warn("resolve", &e),
            }
        }
        if now.duration_since(self.last_pull) > PULL_EVERY {
            if let Err(e) = self.send(PKT_PULL_DATA, *token, eui, None) {
                self.warn("PULL_DATA", &e);
            }
            *token = token.wrapping_add(1);
            self.last_pull = now;
        }
        if !self.lost && now.duration_since(self.last_ack) > LINK_TIMEOUT {
            eprintln!("server {}: no ACK for 30 s (link down?), still receiving RF", self.addr);
            self.lost = true;
            self.outages += 1;
        }
        self.spool.expire(now);
        if self.lost || now.duration_since(self.last_replay) < REPLAY_EVERY {
            return;
        }
        let Some(u) = self.spool.next_replay() else { return };
        self.last_replay = now;
        let body = u.late_body().to_string();
        match self.send(PKT_PUSH_DATA, *token, eui, Some(&body)) {
            Ok(()) => {
                self.push_sent += 1;
                self.spool.replayed += u.rxpk.len() as u64;
                self.spool.sent(*token, u, now);
                if self.spool.backlog_len() == 0 {
                    eprintln!("server {}: backlog sent", self.addr);
                }
            }
            Err(e) => {
                self.warn("PUSH_DATA replay", &e);
                self.spool.put_back(u);
            }
        }
        *token = token.wrapping_add(1);
    }

    /// Next datagram from the server. ACK bookkeeping is done here; the caller
    /// handles PULL_ACK and PULL_RESP. Returns the length of a GWMP v2 packet.
    pub fn recv(&mut self, buf: &mut [u8]) -> Option<usize> {
        loop {
            let n = self.sock.as_ref()?.recv(buf).ok()?;
            if n < 4 || buf[0] != PROTO {
                continue;
            }
            if matches!(buf[3], PKT_PUSH_ACK | PKT_PULL_ACK | PKT_PULL_RESP) {
                if self.lost {
                    eprintln!(
                        "server {} back after {} s, backlog {}",
                        self.addr,
                        self.last_ack.elapsed().as_secs(),
                        self.spool.backlog_len()
                    );
                    self.lost = false;
                }
                self.last_ack = Instant::now();
            }
            if buf[3] == PKT_PUSH_ACK {
                self.push_ack += 1;
                self.spool.acked(u16::from_be_bytes([buf[1], buf[2]]));
            }
            return Some(n);
        }
    }
}

pub fn hdr(ident: u8, token: u16, eui: u64) -> Vec<u8> {
    let mut pkt = Vec::with_capacity(12);
    pkt.push(PROTO);
    pkt.extend_from_slice(&token.to_be_bytes());
    pkt.push(ident);
    pkt.extend_from_slice(&eui.to_be_bytes());
    pkt
}

#[cfg(test)]
mod tests {
    use super::*;

    fn up(ms: u64) -> Uplink {
        Uplink { rxpk: vec![json!({ "tmst": ms })], keep: vec![true], unix_ms: ms }
    }

    #[test]
    fn mtype_filter() {
        assert!(replayable_phy(&[0x40])); // unconfirmed up
        assert!(replayable_phy(&[0x80])); // confirmed up
        assert!(replayable_phy(&[0xE0])); // proprietary
        assert!(!replayable_phy(&[0x00])); // JoinRequest
        assert!(!replayable_phy(&[0xC0])); // Rejoin-Request
        assert!(!replayable_phy(&[]));
    }

    #[test]
    fn ack_clears_and_timeout_defers_in_order() {
        let mut s = Spool::new(10);
        let t0 = Instant::now();
        s.sent(1, up(100), t0);
        s.sent(2, up(200), t0);
        assert!(s.acked(1));
        assert!(!s.acked(1));
        // the link drops: a newer uplink is deferred before the old one times out
        s.defer(up(300));
        assert_eq!(s.expire(t0 + ACK_TIMEOUT), 1);
        let order: Vec<u64> = s.backlog.iter().map(|u| u.unix_ms).collect();
        assert_eq!(order, [200, 300]);
        assert_eq!(s.next_replay().unwrap().unix_ms, 200);
    }

    #[test]
    fn full_backlog_drops_oldest_and_joins_are_not_kept() {
        let mut s = Spool::new(2);
        for ms in [1, 2, 3] {
            s.defer(up(ms));
        }
        assert_eq!(s.backlog.iter().map(|u| u.unix_ms).collect::<Vec<_>>(), [2, 3]);
        assert_eq!(s.dropped, 1);
        let mixed = Uplink {
            rxpk: vec![json!({ "n": "join" }), json!({ "n": "data" })],
            keep: vec![false, true],
            unix_ms: 4,
        };
        s.defer(mixed);
        assert_eq!(s.dropped, 3); // the join, and frame 2 pushed out
        assert_eq!(s.backlog.back().unwrap().rxpk, [json!({ "n": "data" })]);
        let mut off = Spool::new(0);
        off.defer(up(5));
        assert_eq!((off.backlog_len(), off.dropped), (0, 1));
    }

    #[test]
    fn replay_is_paced_by_inflight() {
        let mut s = Spool::new(100);
        let t0 = Instant::now();
        for ms in 0..20 {
            s.defer(up(ms));
        }
        let mut n = 0;
        while let Some(u) = s.next_replay() {
            s.sent(n, u, t0);
            n += 1;
        }
        assert_eq!(n as usize, REPLAY_INFLIGHT);
        s.acked(0);
        assert!(s.next_replay().is_some());
    }

    #[test]
    fn late_body_carries_receive_time() {
        let u = Uplink {
            rxpk: vec![json!({ "tmst": 1 }), json!({ "tmst": 2, "time": "kept" })],
            keep: vec![true, true],
            unix_ms: 1_709_210_096_789,
        };
        let b = u.late_body();
        assert_eq!(b["rxpk"][0]["time"], "2024-02-29T12:34:56.789000Z");
        assert_eq!(b["rxpk"][1]["time"], "kept");
    }

    #[test]
    fn unresolved_name_keeps_uplinks() {
        let mut u = Upstream::connect("no-such-host.invalid:1700", 10).unwrap();
        assert!(u.sock.is_none());
        u.send_rxpk(1, 1, up(1));
        assert_eq!(u.spool.backlog_len(), 1);
        assert!(u.recv(&mut [0u8; 64]).is_none());
    }

    #[test]
    fn link_state_follows_acks() {
        let srv = UdpSocket::bind("127.0.0.1:0").unwrap();
        let addr = srv.local_addr().unwrap().to_string();
        let mut u = Upstream::connect(&addr, 10).unwrap();
        u.lost = true;
        u.send_rxpk(7, 1, up(1));
        assert_eq!(u.spool.backlog_len(), 1); // kept while down
        // server answers a PULL_DATA -> back, then the backlog is replayed
        let mut token = 9;
        u.last_replay = Instant::now() - REPLAY_EVERY;
        u.tick(&mut token, 1);
        let mut b = [0u8; 512];
        let (_, from) = srv.recv_from(&mut b).unwrap(); // PULL_DATA
        srv.send_to(&[PROTO, 0, 9, PKT_PULL_ACK], from).unwrap();
        std::thread::sleep(Duration::from_millis(20));
        assert!(u.recv(&mut b).is_some());
        assert!(!u.lost);
        u.last_replay = Instant::now() - REPLAY_EVERY;
        u.tick(&mut token, 1);
        let n = srv.recv(&mut b).unwrap();
        assert_eq!(b[3], PKT_PUSH_DATA);
        let v: Value = serde_json::from_slice(&b[12..n]).unwrap();
        assert!(v["rxpk"][0]["time"].is_string());
        assert_eq!(u.spool.backlog_len(), 0);
        // its PUSH_ACK clears it
        srv.send_to(&[PROTO, b[1], b[2], PKT_PUSH_ACK], from).unwrap();
        std::thread::sleep(Duration::from_millis(20));
        assert!(u.recv(&mut b).is_some());
        assert_eq!(u.spool.inflight.len(), 0);
    }
}
