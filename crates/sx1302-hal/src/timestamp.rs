//! Hardware PPS / freerun counters (`sx1302_timestamp_counter`).

use crate::Concentrator;
use serde::{Deserialize, Serialize};
use sx1302_core::{HalError, Result};
use sx1302_registers::SX1302_REG_TIMESTAMP_BASE_ADDR;
use sx1302_timestamp::{fine_timestamp_supported, scale_32mhz_to_1mhz, TimestampExpand};

const TS_PPS: u16 = SX1302_REG_TIMESTAMP_BASE_ADDR + 1;
const RX_TOP: u16 = sx1302_registers::SX1302_REG_RX_TOP_BASE_ADDR;

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TimestampDump {
    pub pps_raw_32mhz: u32,
    pub inst_raw_32mhz: u32,
    pub pps_us: u32,
    pub inst_us: u32,
    pub fine_supported: bool,
    pub notes: Vec<String>,
}

impl Concentrator {
    /// Latch end-of-packet (legacy) timestamps. Fine timestamp refused on SX1302.
    pub fn configure_timestamp(&self, fine: bool) -> Result<Vec<String>> {
        let mut notes = Vec::new();
        let port = self.bus.sx1302();
        let model = self
            .read_otp_byte(&port, sx1302_core::SX1302_OTP_MODEL_ID_ADDR)
            .unwrap_or(0);
        if fine && !fine_timestamp_supported(model) {
            return Err(HalError::Other(format!(
                "fine timestamp requires SX1303 (OTP model 0x{model:02x})"
            )));
        }
        if fine {
            port.rmw(RX_TOP + 192, 2, 1, 0)?; // LEGACY_TIMESTAMP off
            port.write_u8(RX_TOP + 193, 32)?; // MAX_TS_METRICS
            notes.push("precision timestamp (SX1303)".into());
        } else {
            port.rmw(RX_TOP + 192, 2, 1, 1)?; // LEGACY_TIMESTAMP
            notes.push("legacy end-of-packet timestamp".into());
        }
        Ok(notes)
    }

    /// Free-running 32 MHz counter (`TIMESTAMP_INST`), same domain as RX packet timestamps.
    pub fn inst_raw_32mhz(&self) -> Result<u32> {
        let port = self.bus.sx1302();
        let mut a = [0u8; 4];
        let mut b = [0u8; 4];
        port.read_burst(TS_PPS + 4, &mut a)?;
        port.read_burst(TS_PPS + 4, &mut b)?;
        let (a, b) = (u32::from_be_bytes(a), u32::from_be_bytes(b));
        // reread guards against a carry between byte reads
        Ok(if b.wrapping_sub(a) < 32 * 10_000 { b } else { a })
    }

    pub fn read_timestamp(&self) -> Result<TimestampDump> {
        let port = self.bus.sx1302();
        let mut a = [0u8; 8];
        port.read_burst(TS_PPS, &mut a)?;
        let mut b = [0u8; 8];
        port.read_burst(TS_PPS, &mut b)?;
        let buff = if a[0] != b[0] || a[4] != b[4] {
            let mut c = [0u8; 8];
            port.read_burst(TS_PPS, &mut c)?;
            c
        } else {
            a
        };
        let pps_raw = u32::from_be_bytes([buff[0], buff[1], buff[2], buff[3]]);
        let inst_raw = u32::from_be_bytes([buff[4], buff[5], buff[6], buff[7]]);
        let pps_us = scale_32mhz_to_1mhz(pps_raw);
        let inst_us = scale_32mhz_to_1mhz(inst_raw);
        let mut exp = TimestampExpand::default();
        exp.update(pps_us, inst_us);
        let model = self
            .read_otp_byte(&port, sx1302_core::SX1302_OTP_MODEL_ID_ADDR)
            .unwrap_or(0);
        let mut notes = vec![
            "PPS/inst are 32 MHz hardware counters scaled /32 to 1 µs".into(),
            "GPS_EN left 0 (no GPS on this carrier)".into(),
        ];
        if pps_raw == 0 {
            notes.push("PPS counter is 0 (no 1PPS edge yet)".into());
        }
        Ok(TimestampDump {
            pps_raw_32mhz: pps_raw,
            inst_raw_32mhz: inst_raw,
            pps_us: exp.expand_pps(pps_us),
            inst_us: exp.expand_inst(inst_us),
            fine_supported: fine_timestamp_supported(model),
            notes,
        })
    }
}
