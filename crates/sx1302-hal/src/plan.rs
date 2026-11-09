//! RX channel plan (`lgw_conf_rxrf_s` / `lgw_conf_rxif_s`): 2 radios, 8 multi-SF
//! LoRa chains, 1 LoRa service chain, 1 FSK chain. JSON-loadable.

use serde::{Deserialize, Serialize};
use sx1302_core::{HalError, Region, Result};

/// SX1250 usable IF span: |IF| + BW/2 <= 800 kHz (`LGW_RF_RX_BANDWIDTH_*` 1.6 MHz).
pub const RF_RX_HALF_SPAN_HZ: i32 = 800_000;

/// Semtech BW codes.
pub const BW_125KHZ: u8 = 0x04;
pub const BW_250KHZ: u8 = 0x05;
pub const BW_500KHZ: u8 = 0x06;

pub fn bw_code_to_hz(code: u8) -> Option<u32> {
    match code {
        BW_125KHZ => Some(125_000),
        BW_250KHZ => Some(250_000),
        BW_500KHZ => Some(500_000),
        _ => None,
    }
}

pub fn bw_hz_to_code(hz: u32) -> Option<u8> {
    match hz {
        125_000 => Some(BW_125KHZ),
        250_000 => Some(BW_250KHZ),
        500_000 => Some(BW_500KHZ),
        _ => None,
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub struct MultiSfChan {
    /// 0 = radio A, 1 = radio B.
    pub radio: u8,
    pub freq_hz: u32,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub struct LoraStdChan {
    pub radio: u8,
    pub freq_hz: u32,
    /// 5..=12
    pub sf: u8,
    /// 125000 / 250000 / 500000
    pub bw_hz: u32,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub struct FskChan {
    pub radio: u8,
    pub freq_hz: u32,
    /// bit/s
    pub datarate: u32,
    #[serde(default = "default_sync_word")]
    pub sync_word: u64,
    #[serde(default = "default_sync_word_size")]
    pub sync_word_size: u8,
}

/// Measured on WM1302 / SenseCAP M1 (2026-11-09): radio B in single-input mode
/// reads ~40 dB low (-83 vs -42 dBm, same node); differential fixes it. Radio A
/// is identical in both modes.
fn default_single_input() -> [bool; 2] {
    [true, false]
}

fn default_sync_word() -> u64 {
    0xC1_94_C1
}
fn default_sync_word_size() -> u8 {
    3
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct ChannelPlan {
    /// SX1250 A / B RX centre frequencies.
    pub radio_hz: [u32; 2],
    /// Semtech `single_input_mode` per radio (SX1250 RF input single-ended).
    #[serde(default = "default_single_input")]
    pub single_input_mode: [bool; 2],
    /// Up to 8 multi-SF BW125 chains (if_chain 0..7).
    pub multisf: Vec<MultiSfChan>,
    /// if_chain 8.
    #[serde(default)]
    pub lora_std: Option<LoraStdChan>,
    /// if_chain 9.
    #[serde(default)]
    pub fsk: Option<FskChan>,
}

/// What an if_chain decodes, resolved to absolute RF.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ChainKind {
    LoraMulti { freq_hz: u32 },
    LoraStd { freq_hz: u32, bw: u8 },
    Fsk { freq_hz: u32, datarate: u32 },
}

pub type ChainMap = [Option<ChainKind>; 10];

impl ChannelPlan {
    /// AS923-1 plan matching ChirpStack `as923` (923.2/923.4 + 922.6–924.0 CFList),
    /// split over both radios so every chain sits within ±300 kHz of its radio.
    pub fn as923_1_default() -> Self {
        let a = 922_900_000;
        let b = 923_700_000;
        let mut multisf = Vec::new();
        for f in [922_600_000, 922_800_000, 923_000_000, 923_200_000] {
            multisf.push(MultiSfChan { radio: 0, freq_hz: f });
        }
        for f in [923_400_000, 923_600_000, 923_800_000, 924_000_000] {
            multisf.push(MultiSfChan { radio: 1, freq_hz: f });
        }
        Self {
            radio_hz: [a, b],
            single_input_mode: default_single_input(),
            multisf,
            lora_std: None,
            fsk: None,
        }
    }

    pub fn from_json(s: &str) -> Result<Self> {
        let p: Self =
            serde_json::from_str(s).map_err(|e| HalError::Other(format!("channel plan: {e}")))?;
        Ok(p)
    }

    pub fn if_hz(&self, radio: u8, freq_hz: u32) -> i32 {
        freq_hz as i64 as i32 - self.radio_hz[usize::from(radio & 1)] as i32
    }

    pub fn validate(&self, region: Region) -> Result<()> {
        let err = |m: String| Err(HalError::Other(format!("channel plan: {m}")));
        if self.multisf.len() > 8 {
            return err(format!("{} multi-SF chains (max 8)", self.multisf.len()));
        }
        let check =|what: &str, radio: u8, freq: u32, bw: u32| -> Result<()> {
            if radio > 1 {
                return err(format!("{what}: radio {radio} (0 or 1)"));
            }
            let half = bw / 2;
            if !region.contains(freq - half) || !region.contains(freq + half) {
                return err(format!("{what}: {freq} Hz ±{half} outside {region:?}"));
            }
            let ifhz = self.if_hz(radio, freq);
            if ifhz.abs() + half as i32 > RF_RX_HALF_SPAN_HZ {
                return err(format!(
                    "{what}: IF {ifhz} Hz + BW/2 exceeds ±{RF_RX_HALF_SPAN_HZ} Hz of radio {radio}"
                ));
            }
            Ok(())
        };
        for (i, c) in self.multisf.iter().enumerate() {
            check(&format!("multisf[{i}]"), c.radio, c.freq_hz, 125_000)?;
        }
        if let Some(s) = &self.lora_std {
            if !(5..=12).contains(&s.sf) {
                return err(format!("lora_std SF{}", s.sf));
            }
            if bw_hz_to_code(s.bw_hz).is_none() {
                return err(format!("lora_std bw {}", s.bw_hz));
            }
            check("lora_std", s.radio, s.freq_hz, s.bw_hz)?;
        }
        if let Some(f) = &self.fsk {
            if !(500..=250_000).contains(&f.datarate) {
                return err(format!("fsk datarate {}", f.datarate));
            }
            if !(1..=8).contains(&f.sync_word_size) {
                return err(format!("fsk sync_word_size {}", f.sync_word_size));
            }
            check("fsk", f.radio, f.freq_hz, 125_000)?;
        }
        for r in self.radio_hz {
            if !region.contains(r) {
                return err(format!("radio {r} Hz outside {region:?}"));
            }
        }
        Ok(())
    }

    pub fn chain_map(&self) -> ChainMap {
        let mut m: ChainMap = [None; 10];
        for (i, c) in self.multisf.iter().enumerate() {
            m[i] = Some(ChainKind::LoraMulti { freq_hz: c.freq_hz });
        }
        if let Some(s) = &self.lora_std {
            m[8] = Some(ChainKind::LoraStd {
                freq_hz: s.freq_hz,
                bw: bw_hz_to_code(s.bw_hz).unwrap_or(BW_125KHZ),
            });
        }
        if let Some(f) = &self.fsk {
            m[9] = Some(ChainKind::Fsk {
                freq_hz: f.freq_hz,
                datarate: f.datarate,
            });
        }
        m
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn default_plan_valid() {
        let p = ChannelPlan::as923_1_default();
        p.validate(Region::As923_1).unwrap();
        assert_eq!(p.if_hz(0, 922_600_000), -300_000);
        assert_eq!(p.if_hz(1, 924_000_000), 300_000);
    }

    #[test]
    fn rejects_out_of_span() {
        let mut p = ChannelPlan::as923_1_default();
        p.multisf[0].freq_hz = 921_900_000; // -1 MHz from radio A
        assert!(p.validate(Region::As923_1).is_err());
    }

    #[test]
    fn json_roundtrip_with_fsk() {
        let mut p = ChannelPlan::as923_1_default();
        p.fsk = Some(FskChan {
            radio: 0,
            freq_hz: 922_300_000,
            datarate: 50_000,
            sync_word: default_sync_word(),
            sync_word_size: 3,
        });
        p.validate(Region::As923_1).unwrap();
        let s = serde_json::to_string(&p).unwrap();
        assert_eq!(ChannelPlan::from_json(&s).unwrap(), p);
        assert!(matches!(p.chain_map()[9], Some(ChainKind::Fsk { datarate: 50_000, .. })));
    }
}
