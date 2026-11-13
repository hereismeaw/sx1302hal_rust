//! Just-in-time TX queue (like `lora_pkt_fwd` `jitqueue.c`).
//!
//! Downlinks are accepted (TX_ACK) when queued and armed on the concentrator
//! `LEAD_US` before their emission time, so several JoinAccepts / RX1 / RX2
//! downlinks can be pending while the single TX_TOP_A chain is busy.

use sx1302_hal::TxRequest;

/// Arm the concentrator this long before the emission time.
pub const LEAD_US: u32 = 100_000;
/// Refuse packets that cannot be programmed in time.
pub const MIN_AHEAD_US: u32 = 20_000;
/// Refuse Class A packets scheduled too far ahead (RX2 JoinAccept is 6 s).
pub const MAX_AHEAD_US: u32 = 30_000_000;
/// Class B ping slots and beacons can be queued up to one beacon period ahead.
pub const MAX_AHEAD_CLASS_B_US: u32 = 130_000_000;
/// Spacing kept after a packet's air time before the next one may be armed.
pub const GUARD_US: u32 = 5_000;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum JitError {
    TooLate,
    TooEarly,
    Collision,
    /// Air time above the configured dwell limit.
    Dwell,
    /// The sliding one-hour air-time budget is used up.
    DutyCycle,
}

impl JitError {
    /// GWMP `txpk_ack.error`. Semtech's PROTOCOL.md has no duty-cycle code, but
    /// ChirpStack maps the name onto `gw.TxAckStatus`, which has `DUTY_CYCLE_OVERFLOW`.
    /// Nothing there means dwell time, so a dwell refusal stays `COLLISION_PACKET`.
    /// [`JitError::reason`] is what the log shows.
    pub fn gwmp(self) -> &'static str {
        match self {
            JitError::TooLate => "TOO_LATE",
            JitError::TooEarly => "TOO_EARLY",
            JitError::DutyCycle => "DUTY_CYCLE_OVERFLOW",
            JitError::Collision | JitError::Dwell => "COLLISION_PACKET",
        }
    }

    pub fn reason(self) -> &'static str {
        match self {
            JitError::Dwell => "dwell time exceeded",
            JitError::DutyCycle => "duty cycle budget used up",
            other => other.gwmp(),
        }
    }
}

/// Length of the sliding duty-cycle window.
pub const DUTY_WINDOW_MS: u64 = 3_600_000;

/// Optional air-time limits: a per-packet dwell time and a duty cycle over the
/// last hour, all channels together. Both are off unless the config sets them.
/// Times are host-monotonic milliseconds (the `tmst` counter wraps every 71.6 min).
#[derive(Debug, Default)]
pub struct Airtime {
    dwell_us: Option<u32>,
    budget_us: Option<u64>,
    sent: Vec<(u64, u32)>,
}

impl Airtime {
    pub fn new(dwell_ms: Option<u32>, duty_cycle_percent: Option<f64>) -> Self {
        Self {
            dwell_us: dwell_ms.map(|m| m.saturating_mul(1000)),
            budget_us: duty_cycle_percent
                .map(|p| (p.clamp(0.0, 100.0) / 100.0 * (DUTY_WINDOW_MS * 1000) as f64) as u64),
            sent: Vec::new(),
        }
    }

    /// Would a packet of `toa_us` starting at `at_ms` stay inside the limits?
    pub fn check(&self, at_ms: u64, toa_us: u32) -> Result<(), JitError> {
        if self.dwell_us.is_some_and(|d| toa_us > d) {
            return Err(JitError::Dwell);
        }
        if let Some(budget) = self.budget_us {
            let used: u64 = self
                .sent
                .iter()
                .filter(|(t, _)| t + DUTY_WINDOW_MS > at_ms)
                .map(|(_, toa)| u64::from(*toa))
                .sum();
            if used + u64::from(toa_us) > budget {
                return Err(JitError::DutyCycle);
            }
        }
        Ok(())
    }

    /// Charge an accepted packet. It stays charged even if arming fails later.
    pub fn record(&mut self, at_ms: u64, toa_us: u32) {
        if self.budget_us.is_some() {
            self.sent.retain(|(t, _)| t + DUTY_WINDOW_MS > at_ms);
            self.sent.push((at_ms, toa_us));
        }
    }
}

#[derive(Debug, Clone)]
pub struct Item {
    /// Emission time (µs, `tmst` domain). For immediate packets: enqueue time.
    pub start_us: u32,
    pub immediate: bool,
    pub toa_us: u32,
    pub req: TxRequest,
}

impl Item {
    /// Busy window [arm, end) relative to `now`, in signed µs.
    fn window(&self, now: u32) -> (i64, i64) {
        let start = i64::from(self.start_us.wrapping_sub(now) as i32);
        let arm = if self.immediate { start } else { start - i64::from(LEAD_US) };
        (arm, start + i64::from(self.toa_us) + i64::from(GUARD_US))
    }
}

#[derive(Debug, Default)]
pub struct JitQueue {
    items: Vec<Item>,
    /// End (air time + guard) of the last packet handed to the concentrator.
    busy_until: Option<u32>,
}

impl JitQueue {
    pub fn len(&self) -> usize {
        self.items.len()
    }

    /// Forget the last armed packet once its air time is over. Call at least every
    /// few minutes so the mark cannot survive a `tmst` wrap.
    pub fn expire(&mut self, now: u32) {
        if self.busy_until.is_some_and(|b| b.wrapping_sub(now) as i32 <= 0) {
            self.busy_until = None;
        }
    }

    /// True while the packet armed last may still be on the air.
    pub fn on_air(&mut self, now: u32) -> bool {
        self.expire(now);
        self.busy_until.is_some()
    }

    /// `start_us = None` = immediate. Class A horizon.
    pub fn enqueue(&mut self, req: TxRequest, start_us: Option<u32>, now: u32) -> Result<(), JitError> {
        self.enqueue_within(req, start_us, now, MAX_AHEAD_US)
    }

    /// `max_ahead_us` is [`MAX_AHEAD_CLASS_B_US`] for `tmms` and beacons.
    pub fn enqueue_within(
        &mut self,
        req: TxRequest,
        start_us: Option<u32>,
        now: u32,
        max_ahead_us: u32,
    ) -> Result<(), JitError> {
        let toa_us = req.time_on_air_us();
        let item = match start_us {
            None => Item { start_us: now, immediate: true, toa_us, req },
            Some(t) => {
                let ahead = t.wrapping_sub(now) as i32;
                if ahead < MIN_AHEAD_US as i32 {
                    return Err(JitError::TooLate);
                }
                if ahead > max_ahead_us as i32 {
                    return Err(JitError::TooEarly);
                }
                Item { start_us: t, immediate: false, toa_us, req }
            }
        };
        let (a0, a1) = item.window(now);
        self.expire(now);
        // The packet armed last is no longer in `items` but may still be on the air.
        if self.busy_until.is_some_and(|b| a0 < i64::from(b.wrapping_sub(now) as i32)) {
            return Err(JitError::Collision);
        }
        for other in &self.items {
            let (b0, b1) = other.window(now);
            if a0 < b1 && b0 < a1 {
                return Err(JitError::Collision);
            }
        }
        self.items.push(item);
        Ok(())
    }

    /// Pop the packet that must be armed now (immediate, or within `LEAD_US`).
    pub fn pop_due(&mut self, now: u32) -> Option<Item> {
        let (idx, _) = self
            .items
            .iter()
            .enumerate()
            .map(|(i, it)| (i, it.window(now).0))
            .filter(|(_, arm)| *arm <= 0)
            .min_by_key(|(_, arm)| *arm)?;
        let item = self.items.swap_remove(idx);
        self.busy_until = Some(
            item.start_us
                .wrapping_add(item.toa_us)
                .wrapping_add(GUARD_US),
        );
        Some(item)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn req(sf: u8, len: usize) -> TxRequest {
        let mut r = TxRequest::as923_1_sf7(vec![0; len]).unwrap();
        r.sf = sf;
        r
    }

    #[test]
    fn late_early() {
        let mut q = JitQueue::default();
        let now = 1_000_000;
        assert_eq!(q.enqueue(req(7, 12), Some(now + 5_000), now), Err(JitError::TooLate));
        assert_eq!(q.enqueue(req(7, 12), Some(now + 40_000_000), now), Err(JitError::TooEarly));
        assert!(q.enqueue(req(7, 12), Some(now + 5_000_000), now).is_ok());
    }

    #[test]
    fn collision_and_spacing() {
        let mut q = JitQueue::default();
        let now = 0;
        // SF12 ~1.15 s on air at t=1 s
        q.enqueue(req(12, 12), Some(1_000_000), now).unwrap();
        // overlaps its air time
        assert_eq!(q.enqueue(req(7, 12), Some(1_500_000), now), Err(JitError::Collision));
        // arm window (start-100 ms) overlaps the SF12 tail
        assert_eq!(q.enqueue(req(7, 12), Some(2_200_000), now), Err(JitError::Collision));
        // clear of it
        q.enqueue(req(7, 12), Some(2_300_000), now).unwrap();
        assert_eq!(q.len(), 2);
    }

    #[test]
    fn pops_in_time_order_and_wraps() {
        let mut q = JitQueue::default();
        let now = u32::MAX - 500_000; // tmst wraps during the test
        q.enqueue(req(7, 12), Some(now.wrapping_add(2_000_000)), now).unwrap();
        q.enqueue(req(7, 12), Some(now.wrapping_add(1_000_000)), now).unwrap();
        assert!(q.pop_due(now).is_none());
        let t = now.wrapping_add(950_000);
        let first = q.pop_due(t).unwrap();
        assert_eq!(first.start_us, now.wrapping_add(1_000_000));
        assert!(q.pop_due(t).is_none());
        assert!(q.pop_due(now.wrapping_add(1_950_000)).is_some());
    }

    #[test]
    fn armed_packet_still_blocks_the_air() {
        let mut q = JitQueue::default();
        // SF12 ~1.15 s on air at t=1 s, armed at 0.9 s
        q.enqueue(req(12, 12), Some(1_000_000), 0).unwrap();
        assert!(q.pop_due(900_000).is_some());
        assert_eq!(q.len(), 0);
        assert_eq!(q.enqueue(req(7, 12), Some(1_500_000), 1_000_000), Err(JitError::Collision));
        assert_eq!(q.enqueue(req(7, 4), None, 1_200_000), Err(JitError::Collision));
        assert!(q.on_air(2_000_000));
        // after air time + guard
        assert!(!q.on_air(2_400_000));
        q.enqueue(req(7, 4), None, 2_400_000).unwrap();
    }

    #[test]
    fn dwell_and_duty_cycle() {
        let off = Airtime::default();
        assert!(off.check(0, 3_000_000).is_ok());

        let toa7 = req(7, 12).time_on_air_us(); // ~41 ms
        let toa12 = req(12, 12).time_on_air_us(); // ~1.15 s
        let dwell = Airtime::new(Some(400), None);
        assert!(dwell.check(0, toa7).is_ok());
        assert_eq!(dwell.check(0, toa12), Err(JitError::Dwell));

        // 0.1 % of an hour = 3.6 s: three SF12 frames fit, the fourth does not.
        let mut duty = Airtime::new(None, Some(0.1));
        for i in 0..3 {
            duty.check(i * 10_000, toa12).unwrap();
            duty.record(i * 10_000, toa12);
        }
        assert_eq!(duty.check(30_000, toa12), Err(JitError::DutyCycle));
        assert!(duty.check(30_000, toa7).is_ok());
        // the first frame leaves the window one hour after it was sent
        assert_eq!(duty.check(DUTY_WINDOW_MS - 1, toa12), Err(JitError::DutyCycle));
        assert!(duty.check(DUTY_WINDOW_MS, toa12).is_ok());
        assert_eq!(JitError::DutyCycle.gwmp(), "DUTY_CYCLE_OVERFLOW");
        assert_eq!(JitError::Dwell.gwmp(), "COLLISION_PACKET");
    }

    #[test]
    fn immediate_is_due_now() {
        let mut q = JitQueue::default();
        q.enqueue(req(7, 4), None, 42).unwrap();
        assert!(q.pop_due(42).unwrap().immediate);
    }
}
