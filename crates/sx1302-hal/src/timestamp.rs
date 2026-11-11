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

/// Expands the 27-bit µs counter (32 MHz / 32) to the 32-bit GWMP `tmst` domain
/// (`timestamp_counter_expand` / `timestamp_pkt_expand`). `tmst·32 mod 2^32` stays
/// equal to the raw 32 MHz counter, so TX trigger math is unchanged.
#[derive(Debug, Default, Clone, Copy)]
pub(crate) struct TsWrap {
    last_us27: u32,
    wraps: u32,
    primed: bool,
}

const US27_MASK: u32 = (1 << 27) - 1;

impl TsWrap {
    /// Feed a fresh instantaneous counter reading (raw 32 MHz); returns expanded µs.
    pub(crate) fn update(&mut self, inst_raw: u32) -> u32 {
        let us27 = inst_raw / 32;
        if self.primed && us27 < self.last_us27 {
            self.wraps = self.wraps.wrapping_add(1);
        }
        self.primed = true;
        self.last_us27 = us27;
        (self.wraps << 27) | us27
    }

    /// Expand a packet timestamp latched near the last `update` (before it, or just
    /// after it when a packet completes between the counter read and the FIFO read).
    /// Uses the shortest modular distance in the 27-bit domain.
    pub(crate) fn expand_pkt(&self, pkt_raw: u32) -> u32 {
        let us27 = (pkt_raw / 32) & US27_MASK;
        let last = (self.wraps << 27) | self.last_us27;
        let d = us27.wrapping_sub(self.last_us27) & US27_MASK;
        if d < (1 << 26) {
            last.wrapping_add(d) // at/after the reference reading
        } else {
            last.wrapping_sub((1 << 27) - d) // before it (possibly before a wrap)
        }
    }
}

/// `MAX_TIMESTAMP_PPS_HISTORY`.
const PPS_HISTORY: usize = 16;
/// One second of the 32 MHz counter.
const TICKS_PER_S: u32 = 32_000_000;

/// `timestamp_pps_history_s`: ring of the last PPS latches (raw 32 MHz counter).
#[derive(Debug, Default, Clone)]
pub struct PpsHistory {
    history: [u32; PPS_HISTORY],
    idx: usize,
    size: usize,
}

/// `sx1302_dc_notch_delay`: extra delay (32 MHz cycles) for channels near DC.
pub fn dc_notch_delay(if_freq_khz: f64) -> f64 {
    if !(-75.0..=75.0).contains(&if_freq_khz) {
        0.0
    } else {
        1.7e-6 * if_freq_khz.powi(4) + 2.4e-6 * if_freq_khz.powi(3)
            - 0.0101 * if_freq_khz.powi(2)
            - 0.01275 * if_freq_khz
            + 10.2922
    }
}

impl PpsHistory {
    /// `timestamp_pps_history_save`: keep the latch only when it changed.
    pub fn save(&mut self, pps_reg: u32) {
        if pps_reg != self.history[self.idx] || self.size == 0 {
            if self.size > 0 {
                self.idx += 1;
            }
            if self.idx == PPS_HISTORY {
                self.idx = 0;
            }
            self.history[self.idx] = pps_reg;
            if self.size < PPS_HISTORY {
                self.size += 1;
            }
        }
    }

    /// `precise_timestamp_calculate`: nanoseconds between the PPS edge before the
    /// packet and its end of preamble. `ts_metrics` holds two entries per metric,
    /// `timestamp_cnt` is the raw 32 MHz latch from the FIFO, `pps_reg` the PPS
    /// register read just now. `None` until the history is full, or when no
    /// reference edge is found.
    pub fn precise_timestamp_ns(
        &mut self,
        pps_reg: u32,
        ts_metrics: &[i8],
        timestamp_cnt: u32,
        sf: u8,
        if_freq_hz: i32,
        pkt_freq_error: f64,
    ) -> Option<u32> {
        if self.size < PPS_HISTORY || !(5..=12).contains(&sf) {
            return None;
        }
        // End of header -> end of preamble (32e6 / 125e3 = 256 cycles per chip).
        let chips = 1u32 << sf;
        let offset = 256 * chips * (8 + 4 + if sf == 5 || sf == 6 { 2 } else { 0 })
            + 256 * (chips / 4 - 1);
        let offset = (f64::from(offset) + f64::from(offset) * pkt_freq_error + 0.5) as u32;
        // 2138 cycles: decimation / filtering group delay difference to the v2 gateway.
        let cnt = timestamp_cnt.wrapping_sub(offset).wrapping_add(2138);

        let nb = ts_metrics.len() / 2;
        let clipped = nb.min(match sf {
            12 => 4,
            11 => 8,
            10 => 16,
            _ => 32,
        });
        if clipped == 0 {
            return None;
        }
        let mut acc = 0i32;
        let mut sum = 0i32;
        for m in &ts_metrics[..2 * clipped] {
            acc += i32::from(*m);
            sum += acc;
        }
        let mean = sum as f32 / (2 * clipped) as f32;

        self.save(pps_reg);
        let (pps, diff) = if cnt.wrapping_sub(pps_reg) > TICKS_PER_S {
            // The register already holds an edge after the packet: rewind.
            let i = (0..self.size)
                .find(|i| cnt.wrapping_sub(self.history[*i]) < TICKS_PER_S)?;
            let next = if i == PPS_HISTORY - 1 { 0 } else { i + 1 };
            (self.history[i], self.history[next].wrapping_sub(self.history[i]))
        } else {
            let prev = if self.idx == 0 { PPS_HISTORY - 1 } else { self.idx - 1 };
            (pps_reg, self.history[self.idx].wrapping_sub(self.history[prev]))
        };
        let xtal_correct = 32e6 / f64::from(diff);
        if !(0.8..=1.2).contains(&xtal_correct) {
            return None;
        }
        let mut ftime = f64::from(cnt.wrapping_sub(pps)) + f64::from(mean);
        ftime += dc_notch_delay(f64::from(if_freq_hz) / 1e3);
        ftime *= 31.25; // 32 MHz cycles -> ns
        ftime *= xtal_correct;
        if !(0.0..=1e9).contains(&ftime) {
            return None;
        }
        Some(ftime as u32)
    }
}

impl Concentrator {
    /// Ask for the SX1303 precision timestamp. Call before [`Concentrator::start`]:
    /// the ARB firmware takes its dual-demodulation setting only while it starts.
    pub fn set_fine_timestamp(&self, on: bool) {
        self.fine.store(on, std::sync::atomic::Ordering::Relaxed);
    }

    /// True when `start` put the chip in precision-timestamp mode.
    pub fn fine_timestamp(&self) -> bool {
        self.fine.load(std::sync::atomic::Ordering::Relaxed)
    }

    /// `timestamp_counter_mode` plus the SX1303 check of `sx1302_init`. Falls back
    /// to the legacy latch on an SX1302. Returns the mode that was programmed.
    pub(crate) fn timestamp_mode(
        &self,
        port: &sx1302_transport::ConcentratorPort<'_>,
        notes: &mut Vec<String>,
    ) -> Result<bool> {
        let mut fine = self.fine_timestamp();
        if fine {
            let model = self
                .read_otp_byte(port, sx1302_core::SX1302_OTP_MODEL_ID_ADDR)
                .unwrap_or(0);
            if !fine_timestamp_supported(model) {
                notes.push(format!(
                    "fine timestamp needs an SX1303 (OTP model 0x{model:02x}); using the legacy timestamp"
                ));
                fine = false;
                self.set_fine_timestamp(false);
            }
        }
        if fine {
            port.rmw(RX_TOP + 192, 2, 1, 0)?; // LEGACY_TIMESTAMP off: latch end of header
            port.write_u8(RX_TOP + 193, 32)?; // MAX_TS_METRICS
            port.rmw(RX_TOP + 179, 3, 1, 1)?; // TIMESTAMP_ENABLE (multi-SF modems)
            port.rmw(RX_TOP + 179, 0, 3, 0)?; // TIMESTAMP_NB_SYMB
            self.enable_pps()?; // the fine timestamp counts from the PPS latch
            notes.push("precision timestamp (SX1303): max_ts_metrics 32, nb_symbols 0, PPS latch on".into());
        } else {
            port.rmw(RX_TOP + 192, 2, 1, 1)?; // LEGACY_TIMESTAMP
        }
        *self.pps_hist.lock().unwrap() = PpsHistory::default();
        Ok(fine)
    }

    /// Raw PPS latch (32 MHz), with the MSB re-read of `timestamp_counter_get`.
    pub(crate) fn pps_raw_32mhz(&self) -> Result<u32> {
        let port = self.bus.sx1302();
        let mut a = [0u8; 4];
        let mut b = [0u8; 4];
        port.read_burst(TS_PPS, &mut a)?;
        port.read_burst(TS_PPS, &mut b)?;
        if a[0] != b[0] {
            port.read_burst(TS_PPS, &mut b)?;
        }
        Ok(u32::from_be_bytes(b))
    }

    /// Fine timestamp of one packet; also keeps the PPS history current.
    pub(crate) fn ftime_ns(
        &self,
        pps_reg: u32,
        ts_metrics: &[i8],
        ts_raw: u32,
        sf: u8,
        if_freq_hz: i32,
        pkt_freq_error: f64,
    ) -> Option<u32> {
        self.pps_hist.lock().unwrap().precise_timestamp_ns(
            pps_reg,
            ts_metrics,
            ts_raw,
            sf,
            if_freq_hz,
            pkt_freq_error,
        )
    }

    pub(crate) fn pps_history_save(&self, pps_reg: u32) {
        self.pps_hist.lock().unwrap().save(pps_reg);
    }

    /// Current concentrator time in the 32-bit `tmst` domain (µs). Must be called at
    /// least every ~134 s (the daemon polls continuously).
    pub fn now_us(&self) -> Result<u32> {
        let raw = self.inst_raw_32mhz()?;
        Ok(self.ts.lock().unwrap().update(raw))
    }

    /// Liveness probe for a watchdog: SPI answers with `COMMON_VERSION` 0x10 and the
    /// 32 MHz counter has advanced since `prev_us` (a stopped radio clock freezes it).
    /// Returns the current `tmst`.
    pub fn health_check(&self, prev_us: Option<u32>) -> Result<u32> {
        let v = self
            .bus
            .sx1302()
            .read_u8(sx1302_core::SX1302_REG_COMMON_VERSION)?;
        if v != sx1302_core::SX1302_VERSION_EXPECTED {
            return Err(HalError::Other(format!("COMMON_VERSION 0x{v:02x}")));
        }
        let now = self.now_us()?;
        if let Some(p) = prev_us {
            if now == p {
                return Err(HalError::Other("concentrator counter not advancing".into()));
            }
        }
        Ok(now)
    }

    pub(crate) fn expand_pkt_ts(&self, pkt_raw: u32) -> u32 {
        self.ts.lock().unwrap().expand_pkt(pkt_raw)
    }

    /// Switch the latch mode on a running concentrator. Prefer
    /// [`Concentrator::set_fine_timestamp`] before `start`: this call cannot turn
    /// on the ARB dual demodulation, so no packet carries fine-timestamp metrics.
    pub fn configure_timestamp(&self, fine: bool) -> Result<Vec<String>> {
        let mut notes = Vec::new();
        let port = self.bus.sx1302();
        self.set_fine_timestamp(fine);
        if !self.timestamp_mode(&port, &mut notes)? {
            if fine {
                return Err(HalError::Other(notes.join("; ")));
            }
            notes.push("legacy end-of-packet timestamp".into());
        }
        Ok(notes)
    }

    /// Latch the SX1302 PPS input. Leave this off until a GPS 1PPS wire is connected.
    pub fn enable_pps(&self) -> Result<()> {
        self.bus.sx1302().rmw(sx1302_registers::SX1302_REG_TIMESTAMP_BASE_ADDR, 0, 1, 1)?;
        Ok(())
    }

    /// Concentrator µs latched by the last PPS edge, in the `tmst` domain.
    /// `None` while the counter is still zero (no edge yet).
    pub fn pps_count_us(&self) -> Result<Option<u32>> {
        let _ = self.now_us()?;
        let port = self.bus.sx1302();
        let mut a = [0u8; 4];
        let mut b = [0u8; 4];
        port.read_burst(TS_PPS, &mut a)?;
        port.read_burst(TS_PPS, &mut b)?;
        let raw = if a[0] == b[0] {
            u32::from_be_bytes(b)
        } else {
            let mut c = [0u8; 4];
            port.read_burst(TS_PPS, &mut c)?;
            u32::from_be_bytes(c)
        };
        if raw == 0 {
            return Ok(None);
        }
        Ok(Some(self.expand_pkt_ts(raw)))
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

#[cfg(test)]
mod tests {
    use super::{dc_notch_delay, PpsHistory, TsWrap};

    /// 16 PPS edges `period` cycles apart; returns the history and the last edge.
    fn history(period: u32) -> (PpsHistory, u32) {
        let mut h = PpsHistory::default();
        let mut pps = 1_000_000u32;
        for _ in 0..16 {
            pps = pps.wrapping_add(period);
            h.save(pps);
            h.save(pps); // unchanged latch is stored once
        }
        (h, pps)
    }

    #[test]
    fn fine_timestamp_from_current_pps() {
        let (mut h, pps) = history(32_000_000);
        // SF7: 256·128·12 + 256·31 = 401152 cycles from end of preamble to end of header.
        let ts_raw = pps + 3_200_000 + 401_152 - 2138;
        // cumulative sum of [4, -2] is [4, 2], mean 3; IF 400 kHz has no notch delay.
        let ns = h.precise_timestamp_ns(pps, &[4, -2], ts_raw, 7, 400_000, 0.0);
        // (3_200_000 + 3) · 31.25 ns = 100_000_093.75
        assert_eq!(ns, Some(100_000_093));
        // SF12 uses the first 4 metrics only: the tail must not move the result.
        let mut m = vec![1i8; 8];
        let off12 = 256 * 4096 * 12 + 256 * 1023;
        let ts12 = pps + 3_200_000 + off12 - 2138;
        let a = h.precise_timestamp_ns(pps, &m, ts12, 12, 400_000, 0.0);
        m.extend([100i8; 8]);
        assert_eq!(h.precise_timestamp_ns(pps, &m, ts12, 12, 400_000, 0.0), a);
        // cumulative 1..=8, mean 4.5
        assert_eq!(a, Some(((3_200_000.0 + 4.5) * 31.25) as u32));
    }

    #[test]
    fn fine_timestamp_rewinds_and_corrects_xtal() {
        // Crystal 10 ppm fast: 32_000_320 cycles between edges.
        let (mut h, last) = history(32_000_320);
        let before = last - 32_000_320;
        let ts_raw = before + 16_000_000 + 401_152 - 2138;
        // The register already shows the edge after the packet.
        let ns = h.precise_timestamp_ns(last, &[0, 0], ts_raw, 7, 400_000, 0.0).unwrap();
        let want = (16_000_000.0 * 31.25 * (32e6 / 32_000_320.0)) as u32;
        assert_eq!(ns, want);
        assert!(ns < 500_000_000 && ns > 499_990_000);
        // A packet older than the whole history has no reference edge.
        let old = last.wrapping_sub(20 * 32_000_320);
        assert_eq!(h.precise_timestamp_ns(last, &[0, 0], old, 7, 400_000, 0.0), None);
    }

    #[test]
    fn fine_timestamp_needs_full_history_and_metrics() {
        let mut h = PpsHistory::default();
        for i in 1..=15u32 {
            h.save(i * 32_000_000);
        }
        assert_eq!(h.precise_timestamp_ns(15 * 32_000_000, &[0, 0], 15 * 32_000_000 + 500_000, 7, 0, 0.0), None);
        let (mut h, pps) = history(32_000_000);
        assert_eq!(h.precise_timestamp_ns(pps, &[], pps + 500_000, 7, 0, 0.0), None);
        // No PPS wire: the latch stays 0 and the history never fills.
        let mut h = PpsHistory::default();
        for _ in 0..100 {
            h.save(0);
        }
        assert_eq!(h.precise_timestamp_ns(0, &[0, 0], 500_000, 7, 0, 0.0), None);
    }

    #[test]
    fn notch_delay_only_near_dc() {
        assert_eq!(dc_notch_delay(0.0), 10.2922);
        assert_eq!(dc_notch_delay(100.0), 0.0);
        assert_eq!(dc_notch_delay(-75.1), 0.0);
        // 1.7e-6·50^4 + 2.4e-6·50^3 − 0.0101·50^2 − 0.01275·50 + 10.2922
        assert!((dc_notch_delay(50.0) - (10.625 + 0.3 - 25.25 - 0.6375 + 10.2922)).abs() < 1e-9);
    }

    #[test]
    fn expands_past_27_bits() {
        let mut w = TsWrap::default();
        let near_wrap = ((1u32 << 27) - 1_000) * 32;
        assert_eq!(w.update(near_wrap), (1 << 27) - 1_000);
        // counter wraps: raw 32 MHz goes past 2^32
        let after = 500u32 * 32;
        assert_eq!(w.update(after), (1 << 27) + 500);
        // packet latched 2000 µs earlier (before the wrap)
        assert_eq!(w.expand_pkt(near_wrap - 1_000 * 32), (1 << 27) - 2_000);
        // packet latched after the wrap
        assert_eq!(w.expand_pkt(200 * 32), (1 << 27) + 200);
        // packet completed just after the counter read (race seen on hardware)
        let mut w2 = TsWrap::default();
        assert_eq!(w2.update(3_000_000 * 32), 3_000_000);
        assert_eq!(w2.expand_pkt(3_000_400 * 32), 3_000_400);
        // tmst·32 == raw counter mod 2^32 (TX trigger domain unchanged)
        assert_eq!(((1u32 << 27) + 500).wrapping_mul(32), after);
    }
}
