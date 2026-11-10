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
}

impl JitError {
    /// GWMP `txpk_ack.error`.
    pub fn gwmp(self) -> &'static str {
        match self {
            JitError::TooLate => "TOO_LATE",
            JitError::TooEarly => "TOO_EARLY",
            JitError::Collision => "COLLISION_PACKET",
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
}

impl JitQueue {
    pub fn len(&self) -> usize {
        self.items.len()
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
        Some(self.items.swap_remove(idx))
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
    fn immediate_is_due_now() {
        let mut q = JitQueue::default();
        q.enqueue(req(7, 4), None, 42).unwrap();
        assert!(q.pop_due(42).unwrap().immediate);
    }
}
