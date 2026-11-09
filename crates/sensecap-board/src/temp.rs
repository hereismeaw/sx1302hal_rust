//! STTS751 board temperature sensor (I2C-1 @ 0x39), as Semtech `loragw_stts751.c`.
//! Read-only: the sensor already runs with CONF 0x8C (what Semtech writes), so no
//! register is written except the read pointer.

use sx1302_core::HalError;

pub const STTS751_I2C_BUS: &str = "/dev/i2c-1";
pub const STTS751_ADDR: u16 = 0x39;
const REG_TEMP_H: u8 = 0x00;
const REG_TEMP_L: u8 = 0x02;
const REG_PROD_ID: u8 = 0xFD;
const REG_MAN_ID: u8 = 0xFE;
const MAN_ID_ST: u8 = 0x53;

/// `stts751_get_temperature`: ((int8)H << 8 | L) / 256.
pub fn decode(high: u8, low: u8) -> f32 {
    f32::from((i16::from(high as i8) << 8) | i16::from(low)) / 256.0
}

pub struct Stts751 {
    #[cfg(target_os = "linux")]
    file: std::fs::File,
}

impl Stts751 {
    /// Open and verify IDs (manufacturer 0x53, product 0x00/0x01).
    #[cfg(target_os = "linux")]
    pub fn open() -> Result<Self, HalError> {
        use std::os::fd::AsRawFd;
        const I2C_SLAVE: u64 = 0x0703;
        let file = std::fs::OpenOptions::new()
            .read(true)
            .write(true)
            .open(STTS751_I2C_BUS)
            .map_err(|e| HalError::Io(format!("{STTS751_I2C_BUS}: {e}")))?;
        let rc = unsafe { libc::ioctl(file.as_raw_fd(), I2C_SLAVE as _, libc::c_ulong::from(STTS751_ADDR)) };
        if rc < 0 {
            return Err(HalError::Io("I2C_SLAVE 0x39 failed".into()));
        }
        let s = Self { file };
        let man = s.read_reg(REG_MAN_ID)?;
        let prod = s.read_reg(REG_PROD_ID)?;
        if man != MAN_ID_ST || prod > 1 {
            return Err(HalError::Other(format!(
                "0x39 is not an STTS751 (man 0x{man:02x} prod 0x{prod:02x})"
            )));
        }
        Ok(s)
    }

    #[cfg(not(target_os = "linux"))]
    pub fn open() -> Result<Self, HalError> {
        Err(HalError::Other("STTS751 needs Linux i2c-dev".into()))
    }

    #[cfg(target_os = "linux")]
    fn read_reg(&self, reg: u8) -> Result<u8, HalError> {
        use std::io::{Read, Write};
        let mut f = &self.file;
        f.write_all(&[reg]).map_err(|e| HalError::Io(format!("STTS751 ptr: {e}")))?;
        let mut b = [0u8];
        f.read_exact(&mut b).map_err(|e| HalError::Io(format!("STTS751 read: {e}")))?;
        Ok(b[0])
    }

    #[cfg(not(target_os = "linux"))]
    fn read_reg(&self, _reg: u8) -> Result<u8, HalError> {
        Err(HalError::Other("STTS751 needs Linux i2c-dev".into()))
    }

    /// Temperature in °C.
    pub fn temperature_c(&self) -> Result<f32, HalError> {
        // Semtech order: low then high byte.
        let low = self.read_reg(REG_TEMP_L)?;
        let high = self.read_reg(REG_TEMP_H)?;
        Ok(decode(high, low))
    }
}

#[cfg(test)]
mod tests {
    use super::decode;

    #[test]
    fn decodes_like_semtech() {
        assert_eq!(decode(0x27, 0x20), 39.125); // measured on the SenseCAP M1
        assert_eq!(decode(0xFF, 0x00), -1.0);
        assert_eq!(decode(0x00, 0x80), 0.5);
    }
}
