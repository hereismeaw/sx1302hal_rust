//! SX1302 32 MHz timestamp / PPS (`loragw_sx1302_timestamp.c`).
//! Wall-clock is never substituted for hardware counters.
//! Precision/fine timestamp is SX1303-only.

use serde::{Deserialize, Serialize};

/// Semtech `CHIP_MODEL_ID_SX1303`.
pub const CHIP_MODEL_ID_SX1303: u8 = 0x01;

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub struct ConcentratorTs(pub u32);

impl ConcentratorTs {
    pub fn wrapping_delta(self, later: ConcentratorTs) -> u32 {
        later.0.wrapping_sub(self.0)
    }
}

/// 27-bit 1 MHz wrap tracker (`timestamp_counter_t`).
#[derive(Debug, Default, Clone, Copy)]
pub struct TimestampExpand {
    pub inst_ref: u32,
    pub inst_wrap: u8,
    pub pps_ref: u32,
    pub pps_wrap: u8,
}

impl TimestampExpand {
    pub fn update(&mut self, pps_1mhz: u32, inst_1mhz: u32) {
        if pps_1mhz < self.pps_ref {
            self.pps_wrap = (self.pps_wrap + 1) % 32;
        }
        if inst_1mhz < self.inst_ref {
            self.inst_wrap = (self.inst_wrap + 1) % 32;
        }
        self.pps_ref = pps_1mhz;
        self.inst_ref = inst_1mhz;
    }

    pub fn expand_inst(&self, cnt_us: u32) -> u32 {
        (u32::from(self.inst_wrap) << 27) | cnt_us
    }

    pub fn expand_pps(&self, cnt_us: u32) -> u32 {
        (u32::from(self.pps_wrap) << 27) | cnt_us
    }
}

/// Scale raw 32 MHz 27-bit counters to 1 MHz (divide by 32).
pub fn scale_32mhz_to_1mhz(raw: u32) -> u32 {
    raw / 32
}

pub fn fine_timestamp_supported(model_id: u8) -> bool {
    model_id == CHIP_MODEL_ID_SX1303
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn rollover() {
        let a = ConcentratorTs(0xffff_fff0);
        let b = ConcentratorTs(0x10);
        assert_eq!(a.wrapping_delta(b), 0x20);
    }

    #[test]
    fn wrap_expand() {
        let mut t = TimestampExpand::default();
        t.update(10, 10);
        t.update(5, 5);
        assert_eq!(t.inst_wrap, 1);
        assert_eq!(t.expand_inst(5), (1 << 27) | 5);
    }

    #[test]
    fn sx1302_has_no_ftime() {
        assert!(!fine_timestamp_supported(0x00));
        assert!(fine_timestamp_supported(CHIP_MODEL_ID_SX1303));
    }
}
