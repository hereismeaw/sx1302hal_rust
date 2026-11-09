//! RX FIFO poll and LoRa TX_TOP send (Semtech `sx1302_send` / RX buffer).

use crate::start::AS923_1_IF_HZ;
use crate::Concentrator;
use serde::{Deserialize, Serialize};
use sx1302_core::{HalError, Region, Result};
use sx1302_registers::{sx1302_freq_to_reg, SX1302_REG_RX_TOP_BASE_ADDR, TX_BUF_A};
use sx1302_transport::ConcentratorPort;

const TX_A: u16 = 0x5200;
const RX_NB_MSB: u16 = SX1302_REG_RX_TOP_BASE_ADDR + 200;
const RX_NB_LSB: u16 = SX1302_REG_RX_TOP_BASE_ADDR + 201;
const RX_RAM: u16 = 0x4000;

/// Semtech `BW_125KHZ`.
pub const BW_125KHZ: u8 = 0x04;

/// Semtech `loragw_sx1302_rx.c` packet framing.
const PKT_SYNC0: u8 = 0xA5;
const PKT_SYNC1: u8 = 0xC0;
const PKT_HEAD: usize = 9;
const PKT_TAIL: usize = 14;
/// Default SX1250 RSSI offset from Semtech `global_conf.json.sx1250.*`.
pub const RSSI_OFFSET_SX1250: f32 = -215.4;
const RADIO_A_HZ: u32 = 923_200_000;

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

/// Parsed concentrator RX packet (`lgw_pkt_rx_s` subset).
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct RxPacket {
    pub freq_hz: u32,
    pub if_chain: u8,
    pub modem_id: u8,
    pub sf: u8,
    pub cr: u8,
    pub size: u8,
    /// GWMP `stat`: 1 CRC OK, -1 CRC fail, 0 no CRC.
    pub stat: i8,
    pub rssi_chan: f32,
    pub rssi_sig: f32,
    pub snr: f32,
    pub count_us: u32,
    pub freq_offset: i32,
    pub payload_hex: String,
    pub payload: Vec<u8>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TxRequest {
    pub region: Region,
    pub freq_hz: u32,
    pub sf: u8,
    pub bw: u8,
    pub cr: u8,
    pub payload: Vec<u8>,
    pub power_idx: u8,
    /// LoRaWAN downlink = true (GWMP `ipol`).
    #[serde(default)]
    pub invert_pol: bool,
    #[serde(default = "default_preamble")]
    pub preamble: u16,
    #[serde(default)]
    pub trigger: TxTrigger,
}

fn default_preamble() -> u16 {
    8
}

/// `lgw_pkt_tx_s.tx_mode`: IMMEDIATE or TIMESTAMPED (1 MHz counter, GWMP `tmst`).
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq, Serialize, Deserialize)]
pub enum TxTrigger {
    #[default]
    Immediate,
    Timestamped(u32),
}

/// `TX_START_DELAY` register value (32 MHz ticks, Semtech default 1500 µs).
const TX_START_DELAY_TICKS: u32 = 0xBB80;

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
            power_idx: 22,
            invert_pol: false,
            preamble: 8,
            trigger: TxTrigger::Immediate,
        })
    }
}

/// Parse Semtech RX FIFO bytes into packets. Public for tests.
pub fn parse_rx_buffer(buf: &[u8], radio_hz: u32, if_hz: &[i32; 8]) -> Vec<RxPacket> {
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
        let tail_base = size;
        let crc_err = pkt[tail_base + 9] & 1 != 0;
        let sync_err = (pkt[tail_base + 9] >> 2) & 1 != 0;
        let hdr_err = (pkt[tail_base + 9] >> 3) & 1 != 0;
        let snr = pkt[tail_base + 10] as i8;
        let rssi_chan = pkt[tail_base + 11];
        let rssi_sig = pkt[tail_base + 12];
        let ts = u32::from(pkt[tail_base + 15])
            | (u32::from(pkt[tail_base + 16]) << 8)
            | (u32::from(pkt[tail_base + 17]) << 16)
            | (u32::from(pkt[tail_base + 18]) << 24);
        let stat = if hdr_err || sync_err {
            -1
        } else if !crc_en {
            0
        } else if crc_err {
            -1
        } else {
            1
        };
        let if_off = if_hz.get(chan as usize).copied().unwrap_or(0);
        let freq_hz = (radio_hz as i32 + if_off) as u32;
        out.push(RxPacket {
            freq_hz,
            if_chain: chan,
            modem_id: modem,
            sf,
            cr,
            size: size as u8,
            stat,
            rssi_chan: f32::from(rssi_chan) + RSSI_OFFSET_SX1250,
            rssi_sig: f32::from(rssi_sig) + RSSI_OFFSET_SX1250,
            snr: f32::from(snr) / 4.0,
            count_us: ts / 32,
            freq_offset: (foff as f32 * 0.119_209_29) as i32,
            payload_hex: payload.iter().map(|b| format!("{b:02x}")).collect::<Vec<_>>().join(""),
            payload,
        });
        idx += total;
    }
    out
}

fn b64(data: &[u8]) -> String {
    const T: &[u8] = b"ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
    let mut out = String::new();
    let mut i = 0;
    while i < data.len() {
        let b0 = data[i];
        let b1 = if i + 1 < data.len() { data[i + 1] } else { 0 };
        let b2 = if i + 2 < data.len() { data[i + 2] } else { 0 };
        let n = data.len() - i;
        out.push(T[(b0 >> 2) as usize] as char);
        out.push(T[(((b0 & 3) << 4) | (b1 >> 4)) as usize] as char);
        if n == 1 {
            out.push('=');
            out.push('=');
        } else {
            out.push(T[(((b1 & 0x0F) << 2) | (b2 >> 6)) as usize] as char);
            if n == 2 {
                out.push('=');
            } else {
                out.push(T[(b2 & 0x3F) as usize] as char);
            }
        }
        i += 3;
    }
    out
}

impl RxPacket {
    /// Semtech GWMP `rxpk` object (PROTOCOL.md).
    pub fn to_rxpk(&self) -> serde_json::Value {
        let cr = match self.cr {
            1 => "4/5",
            2 => "4/6",
            3 => "4/7",
            4 => "4/8",
            _ => "4/5",
        };
        serde_json::json!({
            "tmst": self.count_us,
            "freq": (self.freq_hz as f64) / 1_000_000.0,
            "chan": self.if_chain,
            "rfch": 0,
            "mid": self.modem_id,
            "stat": self.stat,
            "modu": "LORA",
            "datr": format!("SF{}BW125", self.sf),
            "codr": cr,
            "rssi": self.rssi_chan as i32,
            "rssis": self.rssi_sig as i32,
            "lsnr": (f64::from(self.snr) * 10.0).round() / 10.0,
            "foff": self.freq_offset,
            "size": self.size,
            "data": b64(&self.payload),
        })
    }
}

impl Concentrator {
    pub fn rx_poll(&self) -> Result<RxDump> {
        let port = self.bus.sx1302();
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
        let packets = parse_rx_buffer(&buf, RADIO_A_HZ, &AS923_1_IF_HZ)
            .into_iter()
            .filter(|p| p.size > 0)
            .collect();
        Ok(RxDump {
            nb_bytes: nb,
            hex: buf.iter().map(|b| format!("{b:02x}")).collect::<Vec<_>>().join(""),
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

impl Concentrator {
    pub fn send_lora(&self, req: &TxRequest) -> Result<String> {
        if !req.region.contains(req.freq_hz) {
            return Err(HalError::FrequencyOutOfBand {
                freq_hz: req.freq_hz,
                region: req.region,
            });
        }
        if req.payload.is_empty() || req.payload.len() > 255 {
            return Err(HalError::Other("payload 1..=255".into()));
        }
        if !(5..=12).contains(&req.sf) {
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

        let half_bw = 62_500u32;
        let fdev = sx1302_freq_to_reg(half_bw);
        port.write_u8(TX_A + 40, ((fdev >> 8) & 0x0F) as u8)?;
        port.write_u8(TX_A + 41, (fdev & 0xFF) as u8)?;

        let power = (1u8 << 6) | (req.power_idx & 0x3F);
        port.write_u8(TX_A + 13, power)?;
        port.write_u8(TX_A + 12, req.bw)?;

        port.rmw(TX_A + 96, 4, 4, req.bw)?;
        port.rmw(TX_A + 96, 0, 4, req.sf)?;
        port.rmw(TX_A + 97, 0, 3, req.cr)?;
        port.write_u8(TX_A + 99, req.payload.len() as u8)?;
        let preamble = req.preamble.max(6);
        port.write_u8(TX_A + 102, (preamble & 0xFF) as u8)?;
        port.write_u8(TX_A + 103, (preamble >> 8) as u8)?;
        port.rmw(TX_A + 98, 6, 1, 1)?; // MODEM_EN
        port.rmw(TX_A + 98, 4, 2, 2)?; // CADRXTX TX
        port.rmw(TX_A + 98, 0, 1, 1)?; // CRC_EN
        port.rmw(TX_A + 101, 7, 1, 1)?; // MODEM_START
        port.rmw(TX_A + 105, 0, 1, 0)?; // not continuous
        port.rmw(TX_A + 105, 1, 1, u8::from(req.invert_pol))?; // CHIRP_INVERT (ipol)
        let lowpass = if req.sf < 10 { 6 } else { 7 };
        port.rmw(TX_A + 105, 4, 3, lowpass)?; // CHIRP_LOWPASS
        let fine = u8::from(req.sf == 5 || req.sf == 6);
        port.rmw(TX_A + 98, 7, 1, fine)?; // FINE_SYNCH_EN
        // SET_PPM_ON: SF11/12 @ BW125 (low data-rate optimize)
        let ppm = u8::from(req.bw == BW_125KHZ && req.sf >= 11);
        port.rmw(TX_A + 97, 6, 2, ppm)?; // PPM_OFFSET_HDR_CTRL
        port.rmw(TX_A + 97, 4, 2, ppm)?; // PPM_OFFSET
        port.rmw(TX_A + 105, 3, 1, ppm)?; // PPM_OFFSET_SIG
        port.write_u8(TX_A + 9, 0)?; // LoRa modulation
        // Semtech sx1302_tx_configure + LoRa TX_IF_SRC=1
        port.rmw(TX_A + 32, 0, 2, 1)?; // TX_IF_SRC LoRa
        port.rmw(TX_A + 32, 2, 1, 1)?; // TX_IF_DST SX1250
        port.rmw(TX_A + 32, 3, 1, 1)?; // TX_MODE modulation
        port.rmw(TX_A + 32, 4, 1, 0)?; // TX_CLK_EDGE rising
        port.rmw(TX_A + 33, 0, 1, 1)?; // PLL_DIV_CTRL_AGC
        port.rmw(TX_A + 109, 0, 5, 6)?;
        port.rmw(TX_A + 110, 0, 5, 8)?;
        // TX_START_DELAY default 0xBB80 (Semtech loragw_reg)
        port.write_u8(TX_A + 5, 0xBB)?;
        port.write_u8(TX_A + 6, 0x80)?;

        self.write_tx_buf(&port, &req.payload)?;
        // AGC owns radio SPI and PA/LNA LUT (HOST_RADIO_CTRL=0, FORCE_HOST_FE=0)
        port.rmw(0x5601, 3, 1, 0)?;
        port.rmw(0x5780, 3, 1, 0)?;
        if let TxTrigger::Timestamped(count_us) = req.trigger {
            // loragw_sx1302.c: timer = count_us*32 - tx_start_delay (32 MHz domain, wraps).
            let target = count_us.wrapping_mul(32).wrapping_sub(TX_START_DELAY_TICKS);
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
            port.rmw(TX_A + 0, 1, 1, 0)?;
            port.rmw(TX_A + 0, 1, 1, 1)?; // TX_TRIG_DELAYED
            return Ok(format!(
                "TX_TOP_A scheduled tmst={count_us} (+{} ms) {} Hz SF{} {} bytes ipol={}",
                ahead / 32_000,
                req.freq_hz,
                req.sf,
                req.payload.len(),
                req.invert_pol
            ));
        }
        port.rmw(TX_A + 0, 0, 1, 0)?;
        port.rmw(TX_A + 0, 0, 1, 1)?;
        std::thread::sleep(std::time::Duration::from_millis(20));
        let fsm = port.read_u8(TX_A + 17)?;
        std::thread::sleep(std::time::Duration::from_millis(200));
        let fsm2 = port.read_u8(TX_A + 17)?;
        Ok(format!(
            "TX_TOP_A immediate {} Hz SF{} {} bytes fsm={:#04x}->{:#04x}",
            req.freq_hz,
            req.sf,
            req.payload.len(),
            fsm,
            fsm2
        ))
    }

    fn write_tx_buf(&self, port: &ConcentratorPort<'_>, payload: &[u8]) -> Result<()> {
        port.write_u8(TX_A + 7, 1)?;
        port.write_burst(TX_BUF_A, payload)?;
        port.write_u8(TX_A + 7, 0)?;
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::start::AS923_1_IF_HZ;

    #[test]
    fn parse_synthetic_lora() {
        // rebuild correctly
        let mut pkt = vec![0u8; 26];
        pkt[0] = 0xA5;
        pkt[1] = 0xC0;
        pkt[2] = 3;
        pkt[3] = 2;
        pkt[4] = 0x73; // crc + cr1 + sf7
        pkt[5] = 0;
        pkt[9] = 0xAA;
        pkt[10] = 0xBB;
        pkt[11] = 0xCC;
        pkt[12] = 0; // crc_error
        pkt[13] = 20u8; // snr as 20
        pkt[14] = 80;
        pkt[15] = 70;
        let ts: u32 = 32_000;
        pkt[18] = (ts & 0xFF) as u8;
        pkt[19] = ((ts >> 8) & 0xFF) as u8;
        pkt[20] = ((ts >> 16) & 0xFF) as u8;
        pkt[21] = ((ts >> 24) & 0xFF) as u8;
        pkt[24] = 0; // num_ts at size+21 = 3+21=24
        let sum = pkt[..25].iter().fold(0u8, |a, x| a.wrapping_add(*x));
        pkt[25] = sum;
        let pkts = parse_rx_buffer(&pkt, 923_200_000, &AS923_1_IF_HZ);
        assert_eq!(pkts.len(), 1);
        assert_eq!(pkts[0].size, 3);
        assert_eq!(pkts[0].sf, 7);
        assert_eq!(pkts[0].if_chain, 2);
        assert_eq!(pkts[0].payload, vec![0xAA, 0xBB, 0xCC]);
        assert_eq!(pkts[0].stat, 1);
        assert_eq!(pkts[0].count_us, 1000);
        assert_eq!(pkts[0].freq_hz, (923_200_000i32 + AS923_1_IF_HZ[2]) as u32);
        let rxpk = pkts[0].to_rxpk();
        assert_eq!(rxpk["datr"], "SF7BW125");
    }
}
