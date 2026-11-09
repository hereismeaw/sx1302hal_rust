//! Bitfield pack/unpack matching Semtech `reg_w` / `reg_r` in `loragw_reg.c`.
//! Fields with `offs + leng > 8` are rejected (same as the C HAL).

use super::table::{Field, LOREGS, LGW_TOTALREGS};
use sx1302_core::{HalError, Result};

pub fn field(id: u16) -> Result<&'static Field> {
    let i = id as usize;
    if i >= LGW_TOTALREGS {
        return Err(HalError::Other(format!("register id {id} >= {LGW_TOTALREGS}")));
    }
    Ok(&LOREGS[i])
}

/// Bytes to read/write for this field (always 1 in the C HAL).
pub fn span_bytes(f: &Field) -> Result<u8> {
    if f.leng == 0 {
        return Err(HalError::Other("zero-length field".into()));
    }
    if f.offs + f.leng > 8 {
        return Err(HalError::Other(format!(
            "{}: offs+leng > 8 (C HAL also rejects this)",
            f.name
        )));
    }
    Ok(1)
}

/// Insert `value` into a previously read byte (RMW). Direct 8-bit write when offs=0,leng=8.
pub fn pack_into_byte(f: &Field, current: u8, value: i32) -> Result<u8> {
    span_bytes(f)?;
    if f.read_only {
        return Err(HalError::Other(format!("{} is read-only", f.name)));
    }
    if f.leng == 8 && f.offs == 0 {
        return Ok(value as u8);
    }
    let mask = ((1u16 << f.leng) - 1) as u8;
    let packed = (value as u8) & mask;
    let cleared = current & !(mask << f.offs);
    Ok(cleared | (packed << f.offs))
}

/// Extract a field from one SPI-read byte. Sign-extends when `f.signed`.
pub fn unpack_from_byte(f: &Field, byte: u8) -> Result<i32> {
    span_bytes(f)?;
    let left = byte << (8 - f.leng - f.offs);
    if f.signed {
        Ok(((left as i8) >> (8 - f.leng)) as i32)
    } else {
        Ok((left >> (8 - f.leng)) as i32)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::ids;

    #[test]
    fn version_is_ro_addr() {
        let f = field(ids::COMMON_VERSION_VERSION).unwrap();
        assert_eq!(f.addr, 0x5606);
        assert!(f.read_only);
        assert_eq!(f.default, 16);
        assert_eq!(unpack_from_byte(f, 0x10).unwrap(), 0x10);
    }

    #[test]
    fn swap_iq_radio_a_bit1() {
        let f = field(ids::COMMON_CTRL1_SWAP_IQ_RADIO_A).unwrap();
        assert_eq!(f.addr, 0x5602);
        assert_eq!(f.offs, 1);
        assert_eq!(f.leng, 1);
        assert_eq!(pack_into_byte(f, 0x00, 1).unwrap(), 0x02);
        assert_eq!(unpack_from_byte(f, 0x02).unwrap(), 1);
    }

    #[test]
    fn host_radio_ctrl_bit3() {
        let f = field(ids::COMMON_CTRL0_HOST_RADIO_CTRL).unwrap();
        assert_eq!(f.offs, 3);
        assert_eq!(pack_into_byte(f, 0, 1).unwrap(), 0x08);
    }

    #[test]
    fn signed_extract() {
        let f = Field {
            name: "t",
            page: 0,
            addr: 0,
            offs: 0,
            signed: true,
            leng: 8,
            read_only: true,
            checkable: true,
            default: 0,
        };
        assert_eq!(unpack_from_byte(&f, 0xFF).unwrap(), -1);
    }

    #[test]
    fn table_len() {
        assert_eq!(LOREGS.len(), 1044);
        assert_eq!(LOREGS[0].name, "COMMON_PAGE_PAGE");
    }
}
