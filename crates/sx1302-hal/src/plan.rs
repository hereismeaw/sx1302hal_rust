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

fn default_rssi_offset() -> [f32; 2] {
    [-215.4, -215.4]
}

/// `struct lgw_rssi_tcomp_s`: offset(T) = (a·T⁴ + b·T³ + c·T² + d·T + e) / 2¹⁶ dB.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct RssiTcomp {
    pub coeff_a: f32,
    pub coeff_b: f32,
    pub coeff_c: f32,
    pub coeff_d: f32,
    pub coeff_e: f32,
}

impl Default for RssiTcomp {
    /// Semtech `global_conf.json.sx1250.*` reference values.
    fn default() -> Self {
        Self { coeff_a: 0.0, coeff_b: 0.0, coeff_c: 20.41, coeff_d: 2162.56, coeff_e: 0.0 }
    }
}

impl RssiTcomp {
    /// `sx1302_rssi_get_temperature_offset`.
    pub fn offset_db(&self, t: f32) -> f32 {
        let t = f64::from(t);
        ((f64::from(self.coeff_a) * t.powi(4)
            + f64::from(self.coeff_b) * t.powi(3)
            + f64::from(self.coeff_c) * t.powi(2)
            + f64::from(self.coeff_d) * t
            + f64::from(self.coeff_e))
            / 65536.0) as f32
    }
}

/// Semtech reference (`global_conf.json.sx1250.*`) leaves both radios differential.
/// Measured on WM1302 / SenseCAP M1 (2026-11-09): radio B in single-input mode
/// reads ~40 dB low (-83 vs -42 dBm, same node); radio A is identical either way.
fn default_single_input() -> [bool; 2] {
    [false, false]
}

fn default_sync_word() -> u64 {
    0xC1_94_C1
}
fn default_sync_word_size() -> u8 {
    3
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct ChannelPlan {
    /// SX1250 A / B RX centre frequencies.
    pub radio_hz: [u32; 2],
    /// Semtech `single_input_mode` per radio (SX1250 RF input single-ended).
    #[serde(default = "default_single_input")]
    pub single_input_mode: [bool; 2],
    /// Board RSSI calibration per radio (`rssi_offset`, dB).
    #[serde(default = "default_rssi_offset")]
    pub rssi_offset: [f32; 2],
    /// RSSI temperature compensation per radio (`rssi_tcomp`).
    #[serde(default)]
    pub rssi_tcomp: [RssiTcomp; 2],
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
            rssi_offset: default_rssi_offset(),
            rssi_tcomp: [RssiTcomp::default(); 2],
            multisf,
            lora_std: None,
            fsk: None,
        }
    }

    /// RP002 US915 channel `0..64`: 902.3 MHz + n × 200 kHz, 125 kHz, DR0–DR3.
    pub fn us915_uplink_125_hz(ch: u8) -> Option<u32> {
        if ch < 64 {
            Some(902_300_000 + u32::from(ch) * 200_000)
        } else {
            None
        }
    }

    /// RP002 US915 channel `64..72`: 903.0 MHz + n × 1.6 MHz, 500 kHz, DR4.
    pub fn us915_uplink_500_hz(ch: u8) -> Option<u32> {
        if ch < 8 {
            Some(903_000_000 + u32::from(ch) * 1_600_000)
        } else {
            None
        }
    }

    /// RP002 US915 downlink `0..8`: 923.3 MHz + n × 600 kHz, 500 kHz. TX only.
    pub fn us915_downlink_hz(ch: u8) -> Option<u32> {
        if ch < 8 {
            Some(923_300_000 + u32::from(ch) * 600_000)
        } else {
            None
        }
    }

    /// One of the eight US915 sub-bands (`0..8`). The SX1302 listens to that
    /// sub-band's eight 125 kHz channels plus its 500 kHz uplink. Index 1 is
    /// TTN FSB2 (channels 8–15 and 65).
    pub fn us915_subband(index: u8) -> Option<Self> {
        if index > 7 {
            return None;
        }
        let base = 902_300_000 + u32::from(index) * 1_600_000;
        let mut multisf = Vec::with_capacity(8);
        for i in 0..4u32 {
            multisf.push(MultiSfChan { radio: 0, freq_hz: base + i * 200_000 });
        }
        for i in 4..8u32 {
            multisf.push(MultiSfChan { radio: 1, freq_hz: base + i * 200_000 });
        }
        Some(Self {
            radio_hz: [base + 300_000, base + 1_100_000],
            single_input_mode: default_single_input(),
            rssi_offset: default_rssi_offset(),
            rssi_tcomp: [RssiTcomp::default(); 2],
            multisf,
            lora_std: Some(LoraStdChan {
                radio: 0,
                freq_hz: Self::us915_uplink_500_hz(index).unwrap_or(base),
                sf: 8,
                bw_hz: 500_000,
            }),
            fsk: None,
        })
    }

    /// One of the five FCC-800 sub-bands (`0..5`) of the project plan in
    /// `sx1302_core` (`FCC800_LORA_*`): eight 125 kHz uplinks from
    /// 809.1 MHz + index × 1.6 MHz plus the 500 kHz uplink of that sub-band.
    /// Downlinks go out 45 MHz higher and are not part of the RX plan.
    pub fn fcc800_subband(index: u8) -> Option<Self> {
        if index >= sx1302_core::FCC800_LORA_SUBBANDS {
            return None;
        }
        let base = sx1302_core::fcc800_lora_uplink_125_hz(index * 8)?;
        let mut multisf = Vec::with_capacity(8);
        for i in 0..8u32 {
            multisf.push(MultiSfChan { radio: u8::from(i >= 4), freq_hz: base + i * 200_000 });
        }
        Some(Self {
            radio_hz: [base + 300_000, base + 1_100_000],
            single_input_mode: default_single_input(),
            rssi_offset: default_rssi_offset(),
            rssi_tcomp: [RssiTcomp::default(); 2],
            multisf,
            lora_std: Some(LoraStdChan {
                radio: 0,
                freq_hz: sx1302_core::fcc800_lora_uplink_500_hz(index)?,
                sf: 8,
                bw_hz: 500_000,
            }),
            fsk: None,
        })
    }

    /// Preset plan for a region. `subband` is 0-based: US915 `0..8` (default 1,
    /// TTN FSB2), FCC-800 `0..5` (default 0). AS923-1 has one preset.
    pub fn preset(region: Region, subband: Option<u8>) -> Result<Self> {
        let bad = |n: u8, max: u8| HalError::Other(format!("{region} sub-band {n} (0..{max})"));
        match region {
            Region::As923_1 => Ok(Self::as923_1_default()),
            Region::Us915 => {
                let n = subband.unwrap_or(1);
                Self::us915_subband(n).ok_or_else(|| bad(n, 8))
            }
            Region::Fcc800 => {
                let n = subband.unwrap_or(0);
                Self::fcc800_subband(n).ok_or_else(|| bad(n, sx1302_core::FCC800_LORA_SUBBANDS))
            }
            other => Err(HalError::Other(format!("no preset channel plan for {other}"))),
        }
    }

    /// The same plan moved by `delta_hz` (radios and every channel). Used to hop
    /// between blocks that share one IF layout, e.g. US915 sub-bands (1.6 MHz apart).
    pub fn shifted(&self, delta_hz: i64) -> Result<Self> {
        let mv = |f: u32| -> Result<u32> {
            u32::try_from(i64::from(f) + delta_hz)
                .map_err(|_| HalError::Other(format!("channel plan: {f} Hz shifted by {delta_hz} Hz")))
        };
        let mut p = self.clone();
        for r in &mut p.radio_hz {
            *r = mv(*r)?;
        }
        for c in &mut p.multisf {
            c.freq_hz = mv(c.freq_hz)?;
        }
        if let Some(s) = &mut p.lora_std {
            s.freq_hz = mv(s.freq_hz)?;
        }
        if let Some(f) = &mut p.fsk {
            f.freq_hz = mv(f.freq_hz)?;
        }
        Ok(p)
    }

    /// US915 sub-band 2 (RP002 channels 8–15 + 500 kHz uplink 65).
    pub fn us915_sb2() -> Self {
        Self::us915_subband(1).expect("sub-band 1")
    }

    /// Band both radios sit in, then [`Self::validate`]. ISM2400 is named so a
    /// plan aimed there fails with the reason, not a generic miss.
    pub fn band(&self) -> Result<Region> {
        let inside = |r: Region| self.radio_hz.iter().all(|f| r.contains(*f));
        let region = if inside(Region::As923_1) {
            Region::As923_1
        } else if inside(Region::Us915) {
            Region::Us915
        } else if inside(Region::Eu868) {
            Region::Eu868
        } else if inside(Region::Fcc800) {
            Region::Fcc800
        } else if inside(Region::Ism2400) {
            return Err(HalError::Other(
                "ISM2400 2.4 GHz is SX1280 LoRa at 812 kHz; this SX1250 frontend cannot tune it".into(),
            ));
        } else {
            return Err(HalError::Other(
                "channel plan radios are not inside one known sub-GHz band".into(),
            ));
        };
        self.validate(region)?;
        Ok(region)
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
        let u = ChannelPlan::us915_sb2();
        assert_eq!(u.band().unwrap(), Region::Us915);
        assert_eq!(u.lora_std.unwrap().freq_hz, 904_600_000);
        assert_eq!(ChannelPlan::us915_uplink_125_hz(0), Some(902_300_000));
        assert_eq!(ChannelPlan::us915_uplink_125_hz(63), Some(914_900_000));
        assert_eq!(ChannelPlan::us915_uplink_500_hz(7), Some(914_200_000));
        assert_eq!(ChannelPlan::us915_downlink_hz(0), Some(923_300_000));
        assert_eq!(ChannelPlan::us915_downlink_hz(7), Some(927_500_000));
        assert!(ChannelPlan::us915_uplink_125_hz(64).is_none());
        for i in 0..8 {
            ChannelPlan::us915_subband(i).unwrap().validate(Region::Us915).unwrap();
        }
        assert!(ChannelPlan::us915_subband(8).is_none());
    }

    #[test]
    fn fcc800_plan_and_presets() {
        for i in 0..5 {
            let p = ChannelPlan::fcc800_subband(i).unwrap();
            assert_eq!(p.band().unwrap(), Region::Fcc800);
            assert_eq!(p.multisf.len(), 8);
        }
        assert!(ChannelPlan::fcc800_subband(5).is_none());
        let first = ChannelPlan::fcc800_subband(0).unwrap();
        assert_eq!(first.multisf[0].freq_hz, 809_100_000);
        assert_eq!(first.lora_std.unwrap().freq_hz, 809_800_000);
        let last = ChannelPlan::fcc800_subband(4).unwrap();
        assert_eq!(last.multisf[7].freq_hz, 816_900_000);
        assert_eq!(last.lora_std.unwrap().freq_hz, 816_200_000);

        assert_eq!(ChannelPlan::preset(Region::Us915, None).unwrap(), ChannelPlan::us915_sb2());
        assert_eq!(ChannelPlan::preset(Region::Fcc800, Some(4)).unwrap(), last);
        assert_eq!(ChannelPlan::preset(Region::As923_1, None).unwrap(), ChannelPlan::as923_1_default());
        assert!(ChannelPlan::preset(Region::Us915, Some(8)).is_err());
        assert!(ChannelPlan::preset(Region::Eu868, None).is_err());
    }

    #[test]
    fn shifted_plans_are_the_other_subbands() {
        let sb0 = ChannelPlan::us915_subband(0).unwrap();
        for i in 0..8u8 {
            let p = sb0.shifted(i64::from(i) * 1_600_000).unwrap();
            assert_eq!(p, ChannelPlan::us915_subband(i).unwrap());
        }
        let f0 = ChannelPlan::fcc800_subband(0).unwrap();
        assert_eq!(f0.shifted(4 * 1_600_000).unwrap(), ChannelPlan::fcc800_subband(4).unwrap());
        assert!(f0.shifted(5 * 1_600_000).unwrap().band().is_err());
        // AS923-1, 920–925 MHz: the default block and the one 1.6 MHz below fit.
        // 920–925 lies inside the US915 window, so `band()` would call a block
        // outside it US915: a hop has to be validated against its own region.
        let a = ChannelPlan::as923_1_default();
        let low = a.shifted(-1_600_000).unwrap();
        assert_eq!(low.band().unwrap(), Region::As923_1);
        assert_eq!(low.multisf[0].freq_hz, 921_000_000);
        assert!(a.shifted(1_600_000).unwrap().validate(Region::As923_1).is_err());
        assert!(a.shifted(-3_200_000).unwrap().validate(Region::As923_1).is_err());
        assert!(a.shifted(-5_000_000_000).is_err());
    }

    #[test]
    fn tcomp_reference() {
        let t = RssiTcomp::default();
        assert_eq!(t.offset_db(0.0), 0.0);
        // (20.41*39.125^2 + 2162.56*39.125) / 65536
        assert!((t.offset_db(39.125) - 1.7678).abs() < 0.001, "{}", t.offset_db(39.125));
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
