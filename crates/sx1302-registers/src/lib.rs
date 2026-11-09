//! Full SX1302 register map from Semtech `loragw_reg.c` / `loragw_reg.h`
//! (sx1302_hal commit `4b42025d1751e04632c0b04160e0d29dbbb222a5`, v2.1.0).
//!
//! 1044 named bitfields. Pack/unpack matches C `reg_w` / `reg_r`.
//! Addresses are not invented.

pub mod access;
pub mod ids;
pub mod table;

pub use access::{field, pack_into_byte, span_bytes, unpack_from_byte};
pub use table::{Field, LOREGS, LGW_TOTALREGS};

pub const SX1302_REG_EXT_MEM_PAGED_BASE_ADDR: u16 = 0x0000;
pub const SX1302_REG_RX_BUFFER_BASE_ADDR: u16 = 0x4000;
pub const SX1302_REG_TX_TOP_A_BASE_ADDR: u16 = 0x5200;
pub const SX1302_REG_TX_TOP_B_BASE_ADDR: u16 = 0x5400;
pub const SX1302_REG_COMMON_BASE_ADDR: u16 = 0x5600;
pub const SX1302_REG_GPIO_BASE_ADDR: u16 = 0x5640;
pub const SX1302_REG_MBIST_BASE_ADDR: u16 = 0x56C0;
pub const SX1302_REG_RADIO_FE_BASE_ADDR: u16 = 0x5700;
pub const SX1302_REG_AGC_MCU_BASE_ADDR: u16 = 0x5780;
pub const SX1302_REG_CLK_CTRL_BASE_ADDR: u16 = 0x57C0;
pub const SX1302_REG_RX_TOP_BASE_ADDR: u16 = 0x5800;
pub const SX1302_REG_RX_TOP_LORA_SERVICE_FSK_BASE_ADDR: u16 = 0x5B00;
pub const SX1302_REG_CAPTURE_RAM_BASE_ADDR: u16 = 0x6000;
pub const SX1302_REG_ARB_MCU_BASE_ADDR: u16 = 0x6080;
pub const SX1302_REG_TIMESTAMP_BASE_ADDR: u16 = 0x6100;
pub const SX1302_REG_OTP_BASE_ADDR: u16 = 0x6180;

/// Shadow registers (not fuse program): `OTP_MODEM_EN_0/1`.
pub const OTP_MODEM_EN_0: u16 = SX1302_REG_OTP_BASE_ADDR + 8;
pub const OTP_MODEM_EN_1: u16 = SX1302_REG_OTP_BASE_ADDR + 9;
pub const TX_BUF_A: u16 = 0x5300;
pub const TX_BUF_B: u16 = 0x5500;

/// `IF_HZ_TO_REG` from Semtech `loragw_sx1302.c`.
pub fn if_hz_to_reg(freq_hz: i32) -> u16 {
    ((i64::from(freq_hz) * 32) / 15625) as u16
}

/// `SX1302_FREQ_TO_REG`: `f * 2^18 / 32e6`.
pub fn sx1302_freq_to_reg(freq_hz: u32) -> u32 {
    ((u64::from(freq_hz) << 18) / 32_000_000) as u32
}

/// Semtech `calculate_freq_to_time_drift` for BW 125 kHz.
pub fn freq_to_time_drift_125k(freq_hz: u32) -> (u16, u8) {
    let mut mantissa = (125_000u64 * (1u64 << 20)) / u64::from(freq_hz);
    let mut exponent = 0u8;
    while mantissa < 2048 {
        exponent += 1;
        mantissa <<= 1;
    }
    (mantissa as u16, exponent)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn if_zero() {
        assert_eq!(if_hz_to_reg(0), 0);
        assert_eq!(if_hz_to_reg(200_000), 409);
    }

    #[test]
    fn freq2time_as923() {
        let (m, e) = freq_to_time_drift_125k(923_200_000);
        assert!(m >= 2048);
        assert!(e > 0);
    }
}
