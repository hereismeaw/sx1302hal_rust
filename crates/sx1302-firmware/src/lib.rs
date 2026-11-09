//! AGC/ARB firmware blobs as data. Binaries are loaded from Semtech
//! `sx1302_hal` at a recorded SHA when present on disk.

use sx1302_core::{HalError, Result};

pub const UPSTREAM_HAL_SHA: &str = "4b42025d1751e04632c0b04160e0d29dbbb222a5";
pub const MCU_FW_SIZE: usize = 8192;
pub const AGC_MEM_ADDR: u16 = 0x0000;
pub const ARB_MEM_ADDR: u16 = 0x2000;

pub const AGC_MCU_CTRL: u16 = 0x5780;
pub const AGC_MCU_STATUS: u16 = 0x5781;
pub const ARB_MCU_CTRL: u16 = 0x6080;
pub const ARB_MCU_STATUS: u16 = 0x6081;
pub const COMMON_PAGE: u16 = 0x5600;

#[derive(Debug, Clone, Copy)]
pub struct FirmwareRef {
    pub name: &'static str,
    pub sha: &'static str,
}

/// Embedded Semtech AGC (SX1250) and ARB images, 8192 bytes each.
pub const AGC_FW_SX1250: &[u8] = include_bytes!("../../../firmware/agc_fw_sx1250.bin");
pub const ARB_FW_BIN: &[u8] = include_bytes!("../../../firmware/arb_fw.bin");
pub const FW_VERSION_AGC_SX1250: u8 = 10;
pub const FW_VERSION_ARB: u8 = 2;

pub const AGC_FW: FirmwareRef = FirmwareRef {
    name: "agc_fw_sx1250.bin",
    sha: UPSTREAM_HAL_SHA,
};

pub const ARB_FW: FirmwareRef = FirmwareRef {
    name: "arb_fw.bin",
    sha: UPSTREAM_HAL_SHA,
};

/// Parse Semtech `.var` C array (`static const uint8_t name[8192] = { ... };`).
pub fn parse_var(src: &str) -> Result<Vec<u8>> {
    let start = src
        .find('{')
        .ok_or_else(|| HalError::Other("firmware .var missing '{'".into()))?;
    let end = src
        .rfind('}')
        .ok_or_else(|| HalError::Other("firmware .var missing '}'".into()))?;
    let mut out = Vec::with_capacity(MCU_FW_SIZE);
    for tok in src[start + 1..end].split(',') {
        let t = tok.trim();
        if t.is_empty() {
            continue;
        }
        let t = t.trim_end_matches(|c: char| c == '\n' || c == '\r');
        if t.is_empty() {
            continue;
        }
        let v = if t.starts_with("0x") || t.starts_with("0X") {
            u8::from_str_radix(&t[2..], 16)
        } else {
            t.parse::<u8>()
        }
        .map_err(|e| HalError::Other(format!("firmware byte {t}: {e}")))?;
        out.push(v);
    }
    if out.len() != MCU_FW_SIZE {
        return Err(HalError::Other(format!(
            "firmware size {} expected {MCU_FW_SIZE}",
            out.len()
        )));
    }
    Ok(out)
}

pub fn load_bin_or_var(path: &std::path::Path) -> Result<Vec<u8>> {
    let bytes = std::fs::read(path).map_err(|e| HalError::Io(e.to_string()))?;
    if bytes.len() == MCU_FW_SIZE {
        return Ok(bytes);
    }
    let s = String::from_utf8(bytes).map_err(|e| HalError::Other(e.to_string()))?;
    parse_var(&s)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parse_tiny_var_rejects_wrong_len() {
        let err = parse_var("static uint8_t x[] = { 0x00, 1 };").unwrap_err();
        assert!(err.to_string().contains("size"));
    }

    #[test]
    fn embedded_fw_size() {
        assert_eq!(AGC_FW_SX1250.len(), MCU_FW_SIZE);
        assert_eq!(ARB_FW_BIN.len(), MCU_FW_SIZE);
    }
}
