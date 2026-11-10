//! RX FIFO poll and TX_TOP send (Semtech `sx1302_parse` / `sx1302_send`, SHA 4b42025).

use crate::plan::{bw_code_to_hz, ChainKind, ChainMap, BW_125KHZ, BW_250KHZ};
use crate::Concentrator;
use serde::{Deserialize, Serialize};
use sx1302_core::{HalError, Region, Result};
use sx1302_registers::{sx1302_freq_to_reg, SX1302_REG_RX_TOP_BASE_ADDR, TX_BUF_A};
use sx1302_transport::ConcentratorPort;

const TX_A: u16 = 0x5200;
const RX_NB_MSB: u16 = SX1302_REG_RX_TOP_BASE_ADDR + 200;
const RX_NB_LSB: u16 = SX1302_REG_RX_TOP_BASE_ADDR + 201;
const RX_RAM: u16 = 0x4000;
/// AGC mailbox WR_DATA BYTE0 (BYTEn = this − n).
const AGC_MB_WR0: u16 = 0x578C;

/// Semtech `loragw_sx1302_rx.c` packet framing.
const PKT_SYNC0: u8 = 0xA5;
const PKT_SYNC1: u8 = 0xC0;
const PKT_HEAD: usize = 9;
const PKT_TAIL: usize = 14;
/// Default SX1250 RSSI offset from Semtech `global_conf.json.sx1250.*`.
pub const RSSI_OFFSET_SX1250: f32 = -215.4;
/// `TX_START_DELAY_DEFAULT` (µs).
const TX_START_DELAY_DEFAULT_US: u32 = 1500;

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct RxDump {
    pub nb_bytes: u16,
    pub hex: String,
    pub packets: Vec<RxPacket>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub rssi_bb_a: Option<u8>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub rssi_dec_a: Option<u8>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub rssi_dec_min: Option<u8>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub rssi_dec_max: Option<u8>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum RxModulation {
    Lora,
    Fsk,
}

/// Parsed concentrator RX packet (`lgw_pkt_rx_s` subset).
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct RxPacket {
    pub freq_hz: u32,
    pub if_chain: u8,
    pub rf_chain: u8,
    pub modem_id: u8,
    pub modulation: RxModulation,
    /// LoRa SF (0 for FSK).
    pub sf: u8,
    /// Semtech BW code (LoRa) — FSK reports 125 kHz.
    pub bw: u8,
    /// FSK bit/s (0 for LoRa).
    pub datarate: u32,
    pub cr: u8,
    pub size: u8,
    /// GWMP `stat`: 1 CRC OK, -1 CRC fail, 0 no CRC.
    pub stat: i8,
    pub rssi_chan: f32,
    pub rssi_sig: f32,
    pub snr: f32,
    /// GWMP `tmst`: 32-bit µs, end of packet (decode latency compensated).
    pub count_us: u32,
    /// Raw 32 MHz latch from the FIFO.
    pub ts_raw: u32,
    /// Correction already applied to `count_us` (µs, negative = earlier).
    pub ts_correction_us: i32,
    pub freq_offset: i32,
    pub payload_hex: String,
    pub payload: Vec<u8>,
}

/// GWMP `txpk` FSK parameters.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub struct FskTx {
    pub datarate: u32,
    /// Frequency deviation (Hz).
    pub fdev_hz: u32,
    #[serde(default = "default_fsk_sync")]
    pub sync_word: u64,
    #[serde(default = "default_fsk_sync_size")]
    pub sync_word_size: u8,
}

fn default_fsk_sync() -> u64 {
    0xC1_94_C1
}
fn default_fsk_sync_size() -> u8 {
    3
}

/// One `tx_gain_lut` entry (`struct lgw_tx_gain_s`, SX1250 fields).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub struct TxGain {
    /// Conducted RF power at the connector (dBm) this entry was calibrated for.
    pub rf_power: i8,
    /// External PA: 0 = off, >0 = on (only 1 bit drives the PA on SX1250 boards).
    pub pa_gain: u8,
    /// SX1250 power index (0..=31).
    pub pwr_idx: u8,
}

/// TX gain LUT. Default = Semtech `global_conf.json.sx1250.*` (WM1302 / Corecell).
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct TxGainLut(pub Vec<TxGain>);

impl Default for TxGainLut {
    fn default() -> Self {
        const T: [(i8, u8, u8); 16] = [
            (12, 0, 15),
            (13, 0, 16),
            (14, 0, 17),
            (15, 0, 19),
            (16, 0, 20),
            (17, 0, 22),
            (18, 1, 1),
            (19, 1, 2),
            (20, 1, 3),
            (21, 1, 4),
            (22, 1, 5),
            (23, 1, 6),
            (24, 1, 7),
            (25, 1, 9),
            (26, 1, 11),
            (27, 1, 14),
        ];
        Self(
            T.iter()
                .map(|&(rf_power, pa_gain, pwr_idx)| TxGain { rf_power, pa_gain, pwr_idx })
                .collect(),
        )
    }
}

impl TxGainLut {
    /// `sx1302_send`: highest entry with `rf_power <= requested`, else entry 0.
    pub fn select(&self, rf_power_dbm: i8) -> TxGain {
        let mut lut = self.0.clone();
        lut.sort_by_key(|g| g.rf_power);
        lut.iter()
            .rev()
            .find(|g| g.rf_power <= rf_power_dbm)
            .or(lut.first())
            .copied()
            .unwrap_or(TxGain { rf_power: 14, pa_gain: 0, pwr_idx: 17 })
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TxRequest {
    pub region: Region,
    pub freq_hz: u32,
    pub sf: u8,
    pub bw: u8,
    pub cr: u8,
    pub payload: Vec<u8>,
    /// SX1250 power index (from [`TxGainLut::select`]).
    pub power_idx: u8,
    /// External PA on (`pa_gain > 0`).
    #[serde(default)]
    pub pa_gain: u8,
    /// LoRaWAN downlink = true (GWMP `ipol`). Class B beacons are non-inverted.
    #[serde(default)]
    pub invert_pol: bool,
    /// Implicit header, LoRa CRC off. Class B beacons only.
    #[serde(default)]
    pub implicit: bool,
    /// LoRa symbols / FSK bytes.
    #[serde(default = "default_preamble")]
    pub preamble: u16,
    #[serde(default)]
    pub trigger: TxTrigger,
    /// `Some` = FSK (sf/bw/cr/invert_pol ignored).
    #[serde(default)]
    pub fsk: Option<FskTx>,
}

fn default_preamble() -> u16 {
    8
}

/// `lgw_pkt_tx_s.tx_mode`: IMMEDIATE or TIMESTAMPED (32-bit µs, GWMP `tmst`).
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq, Serialize, Deserialize)]
pub enum TxTrigger {
    #[default]
    Immediate,
    Timestamped(u32),
}

impl TxRequest {
    pub fn as923_1_sf7(payload: Vec<u8>) -> Result<Self> {
        let region = Region::As923_1;
        let freq_hz = region.default_tx_hz();
        if !region.contains(freq_hz) {
            return Err(HalError::FrequencyOutOfBand { freq_hz, region });
        }
        Ok(Self {
            region,
            freq_hz,
            sf: 7,
            bw: BW_125KHZ,
            cr: 1,
            payload,
            // 14 dBm from the default LUT (PA off)
            power_idx: 17,
            pa_gain: 0,
            invert_pol: false,
            implicit: false,
            preamble: 8,
            trigger: TxTrigger::Immediate,
            fsk: None,
        })
    }

    /// Apply a LUT entry for `rf_power_dbm`; returns the entry used.
    pub fn set_power(&mut self, lut: &TxGainLut, rf_power_dbm: i8) -> TxGain {
        let g = lut.select(rf_power_dbm);
        self.power_idx = g.pwr_idx;
        self.pa_gain = g.pa_gain;
        g
    }

    /// Air time (µs) of this request.
    pub fn time_on_air_us(&self) -> u32 {
        match &self.fsk {
            Some(f) => {
                let bytes = u64::from(self.preamble)
                    + u64::from(f.sync_word_size)
                    + 1
                    + self.payload.len() as u64
                    + 2;
                (bytes * 8 * 1_000_000 / u64::from(f.datarate.max(1))) as u32
            }
            None => time_on_air_us(
                self.sf,
                bw_code_to_hz(self.bw).unwrap_or(125_000),
                self.cr,
                self.preamble,
                self.payload.len(),
                !self.implicit,
                self.implicit,
            ),
        }
    }
}

/// `SET_PPM_ON`: low-datarate optimisation.
fn ppm_on(bw: u8, sf: u8) -> bool {
    (bw == BW_125KHZ && (sf == 11 || sf == 12)) || (bw == BW_250KHZ && sf == 12)
}

/// LoRa time on air (µs), Semtech `lgw_time_on_air` / SX127x datasheet formula.
pub fn time_on_air_us(
    sf: u8,
    bw_hz: u32,
    cr: u8,
    preamble: u16,
    payload_len: usize,
    crc: bool,
    implicit_header: bool,
) -> u32 {
    let sf_f = f64::from(sf);
    let t_sym = f64::from(1u32 << sf) / f64::from(bw_hz) * 1e6;
    let de = if sf >= 11 && bw_hz == 125_000 || sf == 12 && bw_hz == 250_000 {
        1.0
    } else {
        0.0
    };
    let extra = if sf <= 6 { 6.25 } else { 4.25 };
    let t_pre = (f64::from(preamble) + extra) * t_sym;
    let h = if implicit_header { 1.0 } else { 0.0 };
    let crc = if crc { 1.0 } else { 0.0 };
    let num = 8.0 * payload_len as f64 - 4.0 * sf_f + 28.0 + 16.0 * crc - 20.0 * h;
    let den = 4.0 * (sf_f - 2.0 * de);
    let n_payload = 8.0 + ((num / den).ceil() * f64::from(cr + 4)).max(0.0);
    (t_pre + n_payload * t_sym).round() as u32
}

/// `legacy_timestamp_correction` (`loragw_sx1302_timestamp.c`): µs to add to the
/// legacy end-of-packet latch to compensate modem decode latency (always <= 0).
/// `dft_peak_auto` matches `RX_DFT_PEAK_MODE_AUTO`.
pub fn legacy_timestamp_correction_us(
    bw: u8,
    sf: u8,
    cr: u8,
    crc_en: bool,
    payload_length: u8,
    dft_peak_auto: bool,
) -> i32 {
    let bw_pow: u64 = match bw {
        0x04 => 1,
        0x05 => 2,
        0x06 => 4,
        _ => return 0,
    };
    if !(5..=12).contains(&sf) {
        return 0;
    }
    let sf64 = u64::from(sf);
    let crc = u64::from(crc_en);
    let ppm = u64::from(ppm_on(bw, sf));
    let mut dft_peak_en = u64::from(dft_peak_auto);
    let mut cr_local = u64::from(cr);
    // all delays below in ps, like the C code (clk_period 250e3 / bw_pow)
    let clk_period: u64 = 250_000 / bw_pow;
    let nb_nibble = (u64::from(payload_length) + 2 * crc) * 2 + 5;
    let nb_nibble_in_hdr = if sf == 5 || sf == 6 { sf64 } else { sf64 - 2 };
    let blk = sf64 - 2 * ppm;
    let mut nb_nibble_in_last_block = nb_nibble
        .wrapping_sub(nb_nibble_in_hdr)
        .wrapping_sub(blk * (nb_nibble.wrapping_sub(nb_nibble_in_hdr) / blk));
    if nb_nibble_in_last_block == 0 {
        nb_nibble_in_last_block = blk;
    }
    let nb_iter = (sf64 + 1) / 2;
    let fits = (2 * (i64::from(payload_length) + 2 * crc as i64) - (i64::from(sf) - 7)) <= 0
        || (payload_length == 0 && !crc_en);
    if fits {
        dft_peak_en = 0;
        cr_local = 4;
        nb_nibble_in_last_block = if sf > 6 { sf64 - 2 } else { sf64 };
    }
    let filtering_delay = 16_000_000 / bw_pow + 2_031_250;
    let demap_delay = if fits {
        clk_period + (1 << sf) * clk_period * 3 / 4 + 3 * clk_period + (sf64 - 2) * clk_period
    } else {
        // C: (1 - ppm / 4) is integer arithmetic -> 1 for ppm in {0,1}
        clk_period + (1 << sf) * clk_period + 3 * clk_period + (sf64 - 2 * ppm) * clk_period
    };
    let fft_delay_state3 =
        clk_period * (((1 << sf) - 6) + 2 * ((1 << sf) * (nb_iter - 1) + 6)) + 4 * clk_period;
    let fft_delay = if dft_peak_en == 1 {
        (5 - 2 * ppm) * ((1 << sf) * clk_period + 7 * clk_period) + 2 * clk_period
    } else {
        (1 << sf) * 2 * clk_period + 3 * clk_period
    };
    let decode_delay = 5 * clk_period
        + (9 * clk_period + clk_period * cr_local) * nb_nibble_in_last_block
        + 3 * clk_period;
    let total =
        (filtering_delay + fft_delay_state3 + fft_delay + demap_delay + decode_delay + 500_000)
            / 1_000_000;
    -(total as i32)
}

/// FSK correction (`sx1302_parse`): `680000 / datarate - 20` µs.
fn fsk_timestamp_correction_us(datarate: u32) -> i32 {
    (680_000 / datarate.max(1)) as i32 - 20
}

fn fsk_rssi_linearize(raw: f32) -> f32 {
    let x = f64::from(raw);
    (90.636423 + 0.420835 * x + 0.007129 * x.powi(2) - 0.000026 * x.powi(3)) as f32
}

/// Parse Semtech RX FIFO bytes into packets. `count_us` is the unexpanded 27-bit
/// value plus the decode-latency correction. Public for tests.
pub fn parse_rx_buffer(buf: &[u8], chains: &ChainMap) -> Vec<RxPacket> {
    let mut out = Vec::new();
    let mut idx = 0usize;
    while idx + 2 <= buf.len() {
        if buf[idx] != PKT_SYNC0 || buf[idx + 1] != PKT_SYNC1 {
            idx += 1;
            continue;
        }
        if idx + PKT_HEAD > buf.len() {
            break;
        }
        let size = buf[idx + 2] as usize;
        if idx + PKT_HEAD + size + PKT_TAIL > buf.len() {
            break;
        }
        let num_ts = buf[idx + size + 21] as usize;
        let total = PKT_HEAD + size + PKT_TAIL + 2 * num_ts;
        if idx + total > buf.len() {
            break;
        }
        let pkt = &buf[idx..idx + total];
        let checksum_rcv = pkt[total - 1];
        let checksum_calc = pkt[..total - 1].iter().fold(0u8, |a, b| a.wrapping_add(*b));
        if checksum_rcv != checksum_calc {
            idx += 1;
            continue;
        }
        idx += total;

        let chan = pkt[3];
        let flags = pkt[4];
        let crc_en = flags & 1 != 0;
        let cr = (flags >> 1) & 7;
        let sf = flags >> 4;
        let modem = pkt[5];
        let mut foff = i32::from(pkt[6]) | (i32::from(pkt[7]) << 8) | (i32::from(pkt[8] & 0x0F) << 16);
        if foff >= (1 << 19) {
            foff -= 1 << 20;
        }
        let payload = pkt[PKT_HEAD..PKT_HEAD + size].to_vec();
        let t = size;
        let crc_err = pkt[t + 9] & 1 != 0;
        let sync_err = (pkt[t + 9] >> 2) & 1 != 0;
        let hdr_err = (pkt[t + 9] >> 3) & 1 != 0;
        let snr = pkt[t + 10] as i8;
        let rssi_chan = pkt[t + 11];
        let rssi_sig = pkt[t + 12];
        let ts_raw = u32::from_le_bytes([pkt[t + 15], pkt[t + 16], pkt[t + 17], pkt[t + 18]]);
        let Some(kind) = chains.get(chan as usize).copied().flatten() else {
            continue; // chain not in the active plan
        };
        let stat = if hdr_err || sync_err {
            -1
        } else if !crc_en {
            0
        } else if crc_err {
            -1
        } else {
            1
        };
        let (freq_hz, modulation, bw, datarate, corr, rssi_c, rssi_s, foff_lsb) = match kind {
            ChainKind::LoraMulti { freq_hz } => (
                freq_hz,
                RxModulation::Lora,
                BW_125KHZ,
                0,
                legacy_timestamp_correction_us(BW_125KHZ, sf, cr, crc_en, size as u8, true),
                f32::from(rssi_chan) + RSSI_OFFSET_SX1250,
                f32::from(rssi_sig) + RSSI_OFFSET_SX1250,
                0.119_209_29f32,
            ),
            ChainKind::LoraStd { freq_hz, bw } => (
                freq_hz,
                RxModulation::Lora,
                bw,
                0,
                legacy_timestamp_correction_us(bw, sf, cr, crc_en, size as u8, true),
                f32::from(rssi_chan) + RSSI_OFFSET_SX1250,
                f32::from(rssi_sig) + RSSI_OFFSET_SX1250,
                match bw {
                    BW_250KHZ => 0.238_418_58,
                    0x06 => 0.476_837_16,
                    _ => 0.119_209_29,
                },
            ),
            ChainKind::Fsk { freq_hz, datarate } => (
                freq_hz,
                RxModulation::Fsk,
                BW_125KHZ,
                datarate,
                fsk_timestamp_correction_us(datarate),
                fsk_rssi_linearize(f32::from(rssi_chan)) + RSSI_OFFSET_SX1250,
                -128.0,
                0.0,
            ),
        };
        let is_fsk = modulation == RxModulation::Fsk;
        out.push(RxPacket {
            freq_hz,
            if_chain: chan,
            rf_chain: 0,
            modem_id: modem,
            modulation,
            sf: if is_fsk { 0 } else { sf },
            bw,
            datarate,
            cr: if is_fsk { 0 } else { cr },
            size: size as u8,
            stat,
            rssi_chan: rssi_c,
            rssi_sig: rssi_s,
            snr: if is_fsk { 0.0 } else { f32::from(snr) / 4.0 },
            count_us: (ts_raw / 32).wrapping_add_signed(corr),
            ts_raw,
            ts_correction_us: corr,
            freq_offset: (foff as f32 * foff_lsb) as i32,
            payload_hex: payload.iter().map(|b| format!("{b:02x}")).collect(),
            payload,
        });
    }
    out
}

fn b64(data: &[u8]) -> String {
    const T: &[u8] = b"ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
    let mut out = String::new();
    for chunk in data.chunks(3) {
        let b0 = chunk[0];
        let b1 = chunk.get(1).copied().unwrap_or(0);
        let b2 = chunk.get(2).copied().unwrap_or(0);
        out.push(T[(b0 >> 2) as usize] as char);
        out.push(T[(((b0 & 3) << 4) | (b1 >> 4)) as usize] as char);
        out.push(if chunk.len() > 1 {
            T[(((b1 & 0x0F) << 2) | (b2 >> 6)) as usize] as char
        } else {
            '='
        });
        out.push(if chunk.len() > 2 { T[(b2 & 0x3F) as usize] as char } else { '=' });
    }
    out
}

impl RxPacket {
    /// Semtech GWMP `rxpk` object (PROTOCOL.md).
    pub fn to_rxpk(&self) -> serde_json::Value {
        let mut v = serde_json::json!({
            "tmst": self.count_us,
            "freq": (self.freq_hz as f64) / 1_000_000.0,
            "chan": self.if_chain,
            "rfch": self.rf_chain,
            "mid": self.modem_id,
            "stat": self.stat,
            "rssi": self.rssi_chan.round() as i32,
            "size": self.size,
            "data": b64(&self.payload),
        });
        let o = v.as_object_mut().unwrap();
        match self.modulation {
            RxModulation::Fsk => {
                o.insert("modu".into(), "FSK".into());
                o.insert("datr".into(), self.datarate.into());
            }
            RxModulation::Lora => {
                let cr = match self.cr {
                    2 => "4/6",
                    3 => "4/7",
                    4 => "4/8",
                    _ => "4/5",
                };
                let bw_khz = bw_code_to_hz(self.bw).unwrap_or(125_000) / 1000;
                o.insert("modu".into(), "LORA".into());
                o.insert("datr".into(), format!("SF{}BW{bw_khz}", self.sf).into());
                o.insert("codr".into(), cr.into());
                o.insert("rssis".into(), (self.rssi_sig.round() as i32).into());
                o.insert(
                    "lsnr".into(),
                    ((f64::from(self.snr) * 10.0).round() / 10.0).into(),
                );
                o.insert("foff".into(), self.freq_offset.into());
            }
        }
        v
    }
}

impl Concentrator {
    pub fn rx_poll(&self) -> Result<RxDump> {
        let port = self.bus.sx1302();
        // Refresh the wrap tracker before reading packets latched just before now.
        self.now_us()?;
        let rssi_bb_a = port.read_u8(0x5705).ok();
        let rssi_dec_a = port.read_u8(0x5704).ok();
        let n1 = rx_nb(&port)?;
        let n2 = rx_nb(&port)?;
        let nb = n1.max(n2);
        if nb == 0 {
            return Ok(RxDump {
                nb_bytes: 0,
                hex: String::new(),
                packets: vec![],
                rssi_bb_a,
                rssi_dec_a,
                rssi_dec_min: rssi_dec_a,
                rssi_dec_max: rssi_dec_a,
            });
        }
        let n = (nb as usize).min(4096);
        let mut buf = vec![0u8; n];
        port.read_burst(RX_RAM, &mut buf)?;
        let temperature = *self.temperature.lock().unwrap();
        let (chains, multisf_radio, rssi_adj) = {
            let plan = self.plan.lock().unwrap();
            let radios: Vec<u8> = plan.multisf.iter().map(|c| c.radio).collect();
            let std_radio = plan.lora_std.map(|s| s.radio).unwrap_or(0);
            let fsk_radio = plan.fsk.map(|f| f.radio).unwrap_or(0);
            // loragw_hal.c: rssi += rssi_offset + tcomp(T) per rf_chain
            let adj = |r: usize| {
                plan.rssi_offset[r] - RSSI_OFFSET_SX1250
                    + temperature.map_or(0.0, |t| plan.rssi_tcomp[r].offset_db(t))
            };
            (plan.chain_map(), (radios, std_radio, fsk_radio), [adj(0), adj(1)])
        };
        let packets = parse_rx_buffer(&buf, &chains)
            .into_iter()
            .filter(|p| p.size > 0)
            .map(|mut p| {
                p.count_us = self.expand_pkt_ts(p.ts_raw).wrapping_add_signed(p.ts_correction_us);
                p.rf_chain = match p.if_chain {
                    0..=7 => multisf_radio.0.get(p.if_chain as usize).copied().unwrap_or(0),
                    8 => multisf_radio.1,
                    _ => multisf_radio.2,
                };
                let a = rssi_adj[usize::from(p.rf_chain & 1)];
                p.rssi_chan += a;
                if p.modulation == RxModulation::Lora {
                    p.rssi_sig += a;
                }
                p
            })
            .collect();
        Ok(RxDump {
            nb_bytes: nb,
            hex: buf.iter().map(|b| format!("{b:02x}")).collect(),
            packets,
            rssi_bb_a,
            rssi_dec_a,
            rssi_dec_min: rssi_dec_a,
            rssi_dec_max: rssi_dec_a,
        })
    }

    pub fn rx_packets(&self) -> Result<Vec<RxPacket>> {
        Ok(self.rx_poll()?.packets)
    }
}

fn rx_nb(port: &ConcentratorPort<'_>) -> Result<u16> {
    let msb = port.read_u8(RX_NB_MSB)? & 0x1F;
    let lsb = port.read_u8(RX_NB_LSB)?;
    Ok((u16::from(msb) << 8) | u16::from(lsb))
}

/// SX1250 PLL step: f · 2^25 / 32 MHz (`SX1250_FREQ_TO_REG`).
fn sx1250_freq_to_reg(f: u32) -> u32 {
    ((u64::from(f) << 25) / 32_000_000) as u32
}

impl Concentrator {
    /// TX_TOP_A FSM status (`TX_FSM_STATUS`): 0x80 = idle after a TX.
    pub fn tx_status(&self) -> Result<u8> {
        self.bus.sx1302().read_u8(TX_A + 17)
    }

    /// Back-compat name for [`Concentrator::send`].
    pub fn send_lora(&self, req: &TxRequest) -> Result<String> {
        self.send(req)
    }

    /// Program TX_TOP_A and arm it (immediate or timestamped). Never blocks.
    pub fn send(&self, req: &TxRequest) -> Result<String> {
        if !req.region.contains(req.freq_hz) {
            return Err(HalError::FrequencyOutOfBand {
                freq_hz: req.freq_hz,
                region: req.region,
            });
        }
        if req.region == Region::Fcc800 {
            // shortcut: Semtech sx1250_calibrate covers 779–787, not 809–817.
            return Err(HalError::Other(
                "FCC-800 is 809–817 MHz; SX1250 CALIBRATE_IMAGE has no row for that window".into(),
            ));
        }
        if req.region == Region::Ism2400 {
            // shortcut: SX1250 PLL is sub-GHz. ISM2400 needs an SX1280 (RAK5148's MCU protocol is not driven here).
            return Err(HalError::Other(
                "ISM2400 2.4 GHz is SX1280 LoRa at 812 kHz; this SX1250 frontend cannot tune it".into(),
            ));
        }
        if req.payload.is_empty() || req.payload.len() > 255 {
            return Err(HalError::Other("payload 1..=255".into()));
        }
        if req.fsk.is_none() && !(5..=12).contains(&req.sf) {
            return Err(HalError::Other(format!("SF{} unsupported", req.sf)));
        }
        let port = self.bus.sx1302();
        let fsm0 = port.read_u8(TX_A + 17)?;
        if fsm0 != 0x80 && fsm0 != 0x00 {
            return Err(HalError::Other(format!("TX busy fsm={fsm0:#04x}")));
        }
        let freq_reg = sx1302_freq_to_reg(req.freq_hz);
        port.write_u8(TX_A + 37, ((freq_reg >> 16) & 0xFF) as u8)?;
        port.write_u8(TX_A + 38, ((freq_reg >> 8) & 0xFF) as u8)?;
        port.write_u8(TX_A + 39, (freq_reg & 0xFF) as u8)?;
        if req.power_idx > 31 {
            return Err(HalError::Other(format!("pwr_idx {} > 31", req.power_idx)));
        }
        // SX1250: power = (pa_en << 6) | pwr_idx
        let power = (u8::from(req.pa_gain > 0) << 6) | req.power_idx;
        port.write_u8(TX_A + 13, power)?; // AGC_TX_PWR

        let tx_start_delay: u32 = if let Some(f) = &req.fsk {
            self.program_fsk(&port, req, f)?;
            0 // Semtech: start delay only applied to LoRa
        } else {
            self.program_lora(&port, req)?
        };

        // AGC owns radio SPI and PA/LNA LUT (HOST_RADIO_CTRL=0, FORCE_HOST_FE=0)
        port.rmw(0x5601, 3, 1, 0)?;
        port.rmw(0x5780, 3, 1, 0)?;
        let what = match &req.fsk {
            Some(f) => format!("FSK {} b/s", f.datarate),
            None => format!("SF{}", req.sf),
        };
        match req.trigger {
            TxTrigger::Immediate => {
                port.rmw(TX_A, 0, 1, 0)?;
                port.rmw(TX_A, 0, 1, 1)?; // TX_TRIG_IMMEDIATE
                Ok(format!(
                    "TX_TOP_A immediate {} Hz {what} {} bytes",
                    req.freq_hz,
                    req.payload.len()
                ))
            }
            TxTrigger::Timestamped(count_us) => {
                // timer = count_us*32 - tx_start_delay (32 MHz domain, wraps).
                let target = count_us.wrapping_mul(32).wrapping_sub(tx_start_delay);
                let now = self.inst_raw_32mhz()?;
                let ahead = target.wrapping_sub(now) as i32;
                if ahead < 32 * 1_000 {
                    return Err(HalError::Other(format!(
                        "TOO_LATE: {} µs before trigger",
                        ahead / 32
                    )));
                }
                // JoinAccept RX1/RX2 = 5/6 s; the 32 MHz timer covers ~134 s.
                if ahead > 32 * 10_000_000 {
                    return Err(HalError::Other(format!("TOO_EARLY: {} µs ahead", ahead / 32)));
                }
                port.write_u8(TX_A + 1, (target >> 24) as u8)?;
                port.write_u8(TX_A + 2, (target >> 16) as u8)?;
                port.write_u8(TX_A + 3, (target >> 8) as u8)?;
                port.write_u8(TX_A + 4, target as u8)?;
                port.rmw(TX_A, 1, 1, 0)?;
                port.rmw(TX_A, 1, 1, 1)?; // TX_TRIG_DELAYED
                Ok(format!(
                    "TX_TOP_A scheduled tmst={count_us} (+{} ms) {} Hz {what} {} bytes ipol={} pa={} idx={}",
                    ahead / 32_000,
                    req.freq_hz,
                    req.payload.len(),
                    req.invert_pol,
                    u8::from(req.pa_gain > 0),
                    req.power_idx
                ))
            }
        }
    }

    /// LoRa branch of `sx1302_send`; returns the programmed TX_START_DELAY (32 MHz ticks).
    fn program_lora(&self, port: &ConcentratorPort<'_>, req: &TxRequest) -> Result<u32> {
        let bw_hz = bw_code_to_hz(req.bw)
            .ok_or_else(|| HalError::Other(format!("bw code {:#x}", req.bw)))?;
        port.write_u8(TX_A + 9, 0)?; // MODULATION_TYPE LoRa
        port.rmw(TX_A + 32, 0, 2, 1)?; // TX_IF_SRC LoRa
        port.write_u8(TX_A + 12, req.bw)?; // AGC_TX_BW
        let fdev = sx1302_freq_to_reg(bw_hz / 2);
        port.write_u8(TX_A + 40, ((fdev >> 8) & 0x0F) as u8)?;
        port.write_u8(TX_A + 41, (fdev & 0xFF) as u8)?;
        port.rmw(TX_A + 96, 4, 4, req.bw)?; // MODEM_BW
        port.rmw(TX_A + 96, 0, 4, req.sf)?; // MODEM_SF
        port.rmw(TX_A + 97, 0, 3, req.cr)?; // CODING_RATE
        port.write_u8(TX_A + 99, req.payload.len() as u8)?;
        let preamble = req.preamble.max(6);
        port.write_u8(TX_A + 102, (preamble & 0xFF) as u8)?;
        port.write_u8(TX_A + 103, (preamble >> 8) as u8)?;
        port.rmw(TX_A + 98, 6, 1, 1)?; // MODEM_EN
        port.rmw(TX_A + 98, 4, 2, 2)?; // CADRXTX TX
        port.rmw(TX_A + 98, 1, 1, u8::from(req.implicit))?;
        port.rmw(TX_A + 98, 0, 1, u8::from(!req.implicit))?; // LoRa CRC off on a beacon
        port.rmw(TX_A + 101, 7, 1, 1)?; // MODEM_START
        port.rmw(TX_A + 105, 0, 1, 0)?; // not continuous
        port.rmw(TX_A + 105, 1, 1, u8::from(req.invert_pol))?; // CHIRP_INVERT (ipol)
        let lowpass: u8 = if req.sf < 10 { 6 } else { 7 };
        port.rmw(TX_A + 105, 4, 3, lowpass)?; // CHIRP_LOWPASS
        port.rmw(TX_A + 98, 7, 1, u8::from(req.sf == 5 || req.sf == 6))?; // FINE_SYNCH_EN
        port.rmw(TX_A + 97, 6, 2, 0)?; // PPM_OFFSET_HDR_CTRL = 0 (Semtech)
        port.rmw(TX_A + 97, 4, 2, u8::from(ppm_on(req.bw, req.sf)))?; // PPM_OFFSET
        port.rmw(TX_A + 109, 0, 5, 6)?; // public sync 0x34
        port.rmw(TX_A + 110, 0, 5, 8)?;
        // sx1302_tx_configure
        port.rmw(TX_A + 32, 2, 1, 1)?; // TX_IF_DST SX1250
        port.rmw(TX_A + 32, 3, 1, 1)?; // TX_MODE modulation
        port.rmw(TX_A + 32, 4, 1, 0)?; // TX_CLK_EDGE rising
        port.rmw(TX_A + 33, 0, 1, 1)?; // PLL_DIV_CTRL_AGC
        // sx1302_tx_set_start_delay (SX1250)
        let radio_bw_delay: u32 = match req.bw {
            0x05 => 24,
            0x06 => 21,
            _ => 19,
        };
        let filter_delay = ((1u32 << lowpass) - 1) * 1_000_000 / bw_hz;
        let modem_delay = 8 * (32_000_000 / (32 * bw_hz));
        let delay = TX_START_DELAY_DEFAULT_US * 32 - (radio_bw_delay + filter_delay + modem_delay);
        port.write_u8(TX_A + 5, (delay >> 8) as u8)?;
        port.write_u8(TX_A + 6, delay as u8)?;
        port.write_u8(TX_A + 7, 1)?;
        port.write_burst(TX_BUF_A, &req.payload)?;
        port.write_u8(TX_A + 7, 0)?;
        Ok(delay)
    }

    /// FSK branch of `sx1302_send`.
    fn program_fsk(&self, port: &ConcentratorPort<'_>, req: &TxRequest, f: &FskTx) -> Result<()> {
        if !(500..=250_000).contains(&f.datarate) || !(1..=8).contains(&f.sync_word_size) {
            return Err(HalError::Other("FSK datarate/sync size".into()));
        }
        port.write_u8(TX_A + 9, 1)?; // MODULATION_TYPE FSK
        port.rmw(TX_A + 32, 0, 2, 2)?; // TX_IF_SRC FSK
        port.write_u8(TX_A + 12, 0x80 | BW_125KHZ)?; // AGC_TX_BW (FSK flag)
        let fdev = sx1302_freq_to_reg(f.fdev_hz);
        port.write_u8(TX_A + 40, ((fdev >> 8) & 0x0F) as u8)?;
        port.write_u8(TX_A + 41, (fdev & 0xFF) as u8)?;
        // AGC firmware configures SX1250 FSK fdev from mailbox BYTE2..0
        let fdev_radio = sx1250_freq_to_reg(f.fdev_hz);
        port.write_u8(AGC_MB_WR0 - 2, (fdev_radio >> 16) as u8)?;
        port.write_u8(AGC_MB_WR0 - 1, (fdev_radio >> 8) as u8)?;
        port.write_u8(AGC_MB_WR0, fdev_radio as u8)?;
        // FSK_CFG_0: variable length, CRC on (CCITT), whitening
        port.write_u8(TX_A + 0x41, 0b0000_1011)?;
        // FSK_MOD: gaussian BT 0.5, ref pattern on, size-1, preamble seq 0
        port.write_u8(TX_A + 0x46, 1 | (2 << 1) | (1 << 3) | ((f.sync_word_size - 1) << 5))?;
        let sync = f.sync_word << (8 * (8 - u32::from(f.sync_word_size)));
        for i in 0..8u16 {
            port.write_u8(TX_A + 0x4E - i, (sync >> (8 * i)) as u8)?; // BYTE0 @ 0x524E
        }
        let br = 32_000_000 / f.datarate;
        port.write_u8(TX_A + 0x44, (br >> 8) as u8)?;
        port.write_u8(TX_A + 0x45, br as u8)?;
        let pre = req.preamble.max(3);
        port.write_u8(TX_A + 0x42, (pre >> 8) as u8)?;
        port.write_u8(TX_A + 0x43, pre as u8)?;
        port.write_u8(TX_A + 0x40, req.payload.len() as u8)?; // FSK_PKT_LEN
        port.rmw(TX_A + 32, 2, 1, 1)?;
        port.rmw(TX_A + 32, 3, 1, 1)?;
        port.rmw(TX_A + 32, 4, 1, 0)?;
        port.rmw(TX_A + 33, 0, 1, 1)?;
        port.write_u8(TX_A + 7, 1)?;
        let mut buf = Vec::with_capacity(req.payload.len() + 1);
        buf.push(req.payload.len() as u8); // variable-length size byte
        buf.extend_from_slice(&req.payload);
        port.write_burst(TX_BUF_A, &buf)?;
        port.write_u8(TX_A + 7, 0)?;
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::plan::ChannelPlan;

    fn frame(chan: u8, flags: u8, payload: &[u8], ts: u32) -> Vec<u8> {
        let size = payload.len();
        let mut pkt = vec![0u8; PKT_HEAD + size + PKT_TAIL];
        pkt[0] = 0xA5;
        pkt[1] = 0xC0;
        pkt[2] = size as u8;
        pkt[3] = chan;
        pkt[4] = flags;
        pkt[PKT_HEAD..PKT_HEAD + size].copy_from_slice(payload);
        pkt[size + 10] = 20; // snr*4
        pkt[size + 11] = 80;
        pkt[size + 12] = 70;
        pkt[size + 15..size + 19].copy_from_slice(&ts.to_le_bytes());
        let n = pkt.len();
        pkt[n - 1] = pkt[..n - 1].iter().fold(0u8, |a, x| a.wrapping_add(*x));
        pkt
    }

    #[test]
    fn parse_lora_multisf_uses_plan_freq_and_correction() {
        let plan = ChannelPlan::as923_1_default();
        let buf = frame(3, 0x73, &[0xAA, 0xBB, 0xCC], 32_000 * 32);
        let pkts = parse_rx_buffer(&buf, &plan.chain_map());
        assert_eq!(pkts.len(), 1);
        let p = &pkts[0];
        assert_eq!((p.size, p.sf, p.if_chain, p.stat), (3, 7, 3, 1));
        assert_eq!(p.payload, vec![0xAA, 0xBB, 0xCC]);
        assert_eq!(p.freq_hz, 923_200_000);
        let corr = legacy_timestamp_correction_us(BW_125KHZ, 7, 1, true, 3, true);
        assert!(corr < 0);
        assert_eq!(p.count_us, (32_000i64 + i64::from(corr)) as u32);
        assert_eq!(p.to_rxpk()["datr"], "SF7BW125");
    }

    #[test]
    fn parse_fsk_chain() {
        let mut plan = ChannelPlan::as923_1_default();
        plan.fsk = Some(crate::plan::FskChan {
            radio: 0,
            freq_hz: 922_300_000,
            datarate: 50_000,
            sync_word: 0xC194C1,
            sync_word_size: 3,
        });
        let buf = frame(9, 0x01, b"fsk!", 1_000 * 32);
        let p = &parse_rx_buffer(&buf, &plan.chain_map())[0];
        assert_eq!(p.modulation, RxModulation::Fsk);
        let rx = p.to_rxpk();
        assert_eq!(rx["modu"], "FSK");
        assert_eq!(rx["datr"], 50_000);
        assert!(rx.get("lsnr").is_none());
        assert_eq!(p.count_us, 1_000 - 7); // 680000/50000 - 20
    }

    #[test]
    fn unknown_chain_skipped() {
        let plan = ChannelPlan::as923_1_default(); // no lora_std / fsk
        assert!(parse_rx_buffer(&frame(9, 0x01, b"x", 0), &plan.chain_map()).is_empty());
    }

    #[test]
    fn correction_grows_with_sf() {
        // hand-computed from the C formula: SF7 12 B CRC CR4/5 -> 468 µs
        assert_eq!(legacy_timestamp_correction_us(BW_125KHZ, 7, 1, true, 12, true), -468);
        let mut last = 0;
        for sf in 7..=12 {
            let c = legacy_timestamp_correction_us(BW_125KHZ, sf, 1, true, 20, true);
            assert!(c < last, "SF{sf} {c}");
            last = c;
        }
    }

    #[test]
    fn lut_selection_matches_semtech() {
        let lut = TxGainLut::default();
        assert_eq!(lut.select(14), TxGain { rf_power: 14, pa_gain: 0, pwr_idx: 17 });
        assert_eq!(lut.select(16), TxGain { rf_power: 16, pa_gain: 0, pwr_idx: 20 });
        assert_eq!(lut.select(30).rf_power, 27); // capped at the top entry
        assert_eq!(lut.select(5).rf_power, 12); // below table -> entry 0
        assert_eq!(lut.select(18).pa_gain, 1);
    }

    #[test]
    fn airtime_reference_values() {
        // Semtech LoRa calculator: SF7 BW125 CR4/5 12 B, preamble 8, CRC, explicit = 41.2 ms
        assert_eq!(time_on_air_us(7, 125_000, 1, 8, 12, true, false), 41_216);
        // SF12 BW125 (DE on) 12 B = 1155.1 ms
        assert_eq!(time_on_air_us(12, 125_000, 1, 8, 12, true, false), 1_155_072);
    }
}
