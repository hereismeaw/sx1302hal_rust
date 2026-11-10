//! Board-independent types for the native SX1302/SX1303 HAL.

use serde::{Deserialize, Serialize};
use thiserror::Error;

/// Semtech SX1302 SPI mux byte (`loragw_com.h`, commit 4b42025).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[repr(u8)]
pub enum SpiMuxTarget {
    Sx1302 = 0x00,
    RadioA = 0x01,
    RadioB = 0x02,
}

impl SpiMuxTarget {
    pub const fn as_u8(self) -> u8 {
        self as u8
    }
}

/// Semtech default SPI clock (`loragw_spi.h` `SPI_SPEED`).
pub const SEMTECH_SPI_SPEED_HZ: u32 = 2_000_000;

/// COMMON_VERSION expected reset value from Semtech register table (0x10).
pub const SX1302_VERSION_EXPECTED: u8 = 0x10;

/// COMMON_VERSION register absolute address (`SX1302_REG_COMMON_BASE_ADDR + 6`).
pub const SX1302_REG_COMMON_VERSION: u16 = 0x5606;

/// COMMON_PAGE register (`SX1302_REG_COMMON_BASE_ADDR + 0`).
pub const SX1302_REG_COMMON_PAGE: u16 = 0x5600;

/// COMMON_DUMMY register (`SX1302_REG_COMMON_BASE_ADDR + 7`).
pub const SX1302_REG_COMMON_DUMMY: u16 = 0x5607;

/// OTP byte address pointer (`SX1302_REG_OTP_BASE_ADDR + 0`).
pub const SX1302_REG_OTP_BYTE_ADDR: u16 = 0x6180;
/// OTP read data (`SX1302_REG_OTP_BASE_ADDR + 1`).
pub const SX1302_REG_OTP_RD_DATA: u16 = 0x6181;
/// OTP Chip Model ID index used by `sx1302_get_model_id`.
pub const SX1302_OTP_MODEL_ID_ADDR: u8 = 0xD0;
/// OTP access-mode register (`OTP_BASE + 3`).
pub const SX1302_REG_OTP_CFG_ACCESS_MODE: u16 = 0x6183;
/// OTP status (`OTP_BASE + 2`), bit0 = FSM_READY.
pub const SX1302_REG_OTP_STATUS: u16 = 0x6182;

#[derive(Debug, Error)]
pub enum HalError {
    #[error("transport: {0}")]
    Transport(String),
    #[error("gpio refused: {0}")]
    GpioRefused(String),
    #[error("chip not detected (version=0x{got:02x}, expected=0x{expected:02x})")]
    ChipNotDetected { got: u8, expected: u8 },
    #[error("io: {0}")]
    Io(String),
    #[error("unsupported on this host: {0}")]
    Unsupported(&'static str),
    #[error("region required for TX (pass --region EU868|US915|AS923-1)")]
    RegionRequired,
    #[error("frequency {freq_hz} Hz outside {region} band")]
    FrequencyOutOfBand { freq_hz: u32, region: Region },
    #[error("{0}")]
    Other(String),
}

/// Regulatory band for RF TX. Required before SET_TX / PA_EN.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum Region {
    Eu868,
    Us915,
    /// LoRaWAN AS923-1, RF limited to 920–925 MHz on this gateway.
    #[serde(rename = "AS923-1")]
    As923_1,
    /// LoRaWAN 2.4 GHz (Semtech ISM2400). SX1280, not the SX1250 on this HAL.
    #[serde(rename = "ISM2400")]
    Ism2400,
    /// IotaComm FCC Part 90 licensed block, 809.0–817.0 MHz (announced 2026-01-29).
    /// No LoRa Alliance channel raster. SX1250 image cal has no row here
    /// (Semtech's nearest row is 779–787 MHz).
    #[serde(rename = "FCC-800")]
    Fcc800,
}

/// ISM2400 default uplink channels (MHz): 2403, 2425, 2479. BW 812 kHz, DR0–DR7.
pub const ISM2400_CHANNELS_HZ: [u32; 3] = [2_403_000_000, 2_425_000_000, 2_479_000_000];
/// RX2 default: 2423 MHz, DR0.
pub const ISM2400_RX2_HZ: u32 = 2_423_000_000;

/// Part 90 12.5 kHz channel centers inside the announced 809.0–817.0 MHz block.
/// Downlink is uplink + 45 MHz, which lands on 854.0125–861.9875 MHz — the FSK
/// range in the TEKTELIC IotaComm gateway FCC exhibit. Not a LoRa raster.
pub const FCC800_CH_STEP_HZ: u32 = 12_500;
pub const FCC800_UPLINK_FIRST_HZ: u32 = 809_012_500;
pub const FCC800_UPLINK_COUNT: u16 = 639;
pub const FCC800_DUPLEX_HZ: u32 = 45_000_000;

pub fn fcc800_uplink_hz(index: u16) -> Option<u32> {
    if index >= FCC800_UPLINK_COUNT {
        None
    } else {
        Some(FCC800_UPLINK_FIRST_HZ + u32::from(index) * FCC800_CH_STEP_HZ)
    }
}

pub fn fcc800_downlink_hz(index: u16) -> Option<u32> {
    fcc800_uplink_hz(index).map(|hz| hz + FCC800_DUPLEX_HZ)
}

/// DR0 = SF12 … DR7 = SF5, all at 812 kHz, coding rate 4/8 long interleaver.
pub fn ism2400_sf(dr: u8) -> Option<u8> {
    match dr {
        0..=7 => Some(12 - dr),
        _ => None,
    }
}

impl Region {
    pub fn parse(s: &str) -> Result<Self> {
        match s.trim().to_ascii_uppercase().replace('_', "-").as_str() {
            "EU868" | "EU" => Ok(Self::Eu868),
            "US915" | "US" => Ok(Self::Us915),
            "AS923" | "AS" | "AS923-1" => Ok(Self::As923_1),
            "ISM2400" | "2.4" | "2G4" => Ok(Self::Ism2400),
            "FCC-800" | "FCC800" | "809" => Ok(Self::Fcc800),
            _ => Err(HalError::Other(format!("unknown region {s}"))),
        }
    }

    pub fn default_tx_hz(self) -> u32 {
        match self {
            Self::Eu868 => 868_300_000,
            Self::Us915 => 903_900_000,
            Self::As923_1 => 923_200_000,
            Self::Ism2400 => ISM2400_CHANNELS_HZ[0],
            // No published channel. 0 fails `contains`, so TX must pass an explicit Hz.
            Self::Fcc800 => 0,
        }
    }

    /// Inclusive RF window used for TX frequency checks.
    pub fn band_hz(self) -> (u32, u32) {
        match self {
            Self::Eu868 => (863_000_000, 870_000_000),
            Self::Us915 => (902_000_000, 928_000_000),
            Self::As923_1 => (920_000_000, 925_000_000),
            // Semtech ISM2400: devices operate 2400–2480 MHz.
            Self::Ism2400 => (2_400_000_000, 2_480_000_000),
            Self::Fcc800 => (809_000_000, 817_000_000),
        }
    }

    pub fn contains(self, freq_hz: u32) -> bool {
        let (lo, hi) = self.band_hz();
        (lo..=hi).contains(&freq_hz)
    }

    pub fn as_str(self) -> &'static str {
        match self {
            Self::Eu868 => "EU868",
            Self::Us915 => "US915",
            Self::As923_1 => "AS923-1",
            Self::Ism2400 => "ISM2400",
            Self::Fcc800 => "FCC-800",
        }
    }
}

impl std::fmt::Display for Region {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(self.as_str())
    }
}

pub type Result<T> = std::result::Result<T, HalError>;

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum VerificationStatus {
    VerifiedOnHardware,
    CandidateFromDocs,
    ConflictUnverified,
    NotPresent,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ChipIdentity {
    pub version: u8,
    pub matches_sx1302: bool,
    pub spi_path: String,
    pub mux: SpiMuxTarget,
    pub eui: Option<u64>,
    pub model_id: Option<u8>,
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn region_as923_1_is_920_925() {
        let r = Region::parse("AS923-1").unwrap();
        assert_eq!(r, Region::parse("AS923").unwrap());
        let g = Region::parse("ISM2400").unwrap();
        assert!(g.contains(2_403_000_000));
        assert_eq!(ism2400_sf(0), Some(12));
        assert_eq!(ism2400_sf(7), Some(5));
        assert!(ism2400_sf(8).is_none());
        assert_eq!(r.as_str(), "AS923-1");
        assert_eq!(r.default_tx_hz(), 923_200_000);
        assert_eq!(r.band_hz(), (920_000_000, 925_000_000));
        assert!(r.contains(920_000_000));
        assert!(r.contains(925_000_000));
        assert!(!r.contains(919_999_999));
        assert!(!r.contains(925_000_001));
        assert!(!r.contains(868_300_000));
        let fcc = Region::parse("809").unwrap();
        assert_eq!(fcc, Region::Fcc800);
        assert!(fcc.contains(809_000_000) && fcc.contains(817_000_000));
        assert!(!fcc.contains(902_000_000));
        assert_eq!(fcc.default_tx_hz(), 0);
        assert!(Region::Us915.contains(903_900_000));
        assert_eq!(fcc800_uplink_hz(0), Some(809_012_500));
        assert_eq!(fcc800_uplink_hz(638), Some(816_987_500));
        assert_eq!(fcc800_downlink_hz(0), Some(854_012_500));
        assert_eq!(fcc800_downlink_hz(638), Some(861_987_500));
        assert!(fcc800_uplink_hz(639).is_none());
    }
}
