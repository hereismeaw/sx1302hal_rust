//! Auxiliary SX126x on the WM1302 SPI module (GPIO6 CS, SPI_NO_CS).
//! Seeed names SX1262; Semtech HAL comments SX1261. Identify before LBT.
//! No SX126 TX from this crate.

use serde::{Deserialize, Serialize};
use sx1302_core::{HalError, Result, VerificationStatus};

const SPI_NO_CS: u8 = 0x40;
const OP_GET_STATUS: u8 = 0xC0;
const OP_SET_STANDBY: u8 = 0x80;
const OP_SET_RF_FREQ: u8 = 0x86;
const OP_SET_RX: u8 = 0x82;
const OP_GET_RSSI_INST: u8 = 0x15;
const STDBY_RC: u8 = 0x00;

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct AuxRadioPins {
    pub reset_bcm: u8,
    pub cs_bcm: u8,
    pub busy_bcm: Option<u8>,
    pub dio1_bcm: u8,
    pub dio2_bcm: u8,
    pub verification: VerificationStatus,
}

pub fn wm1302_hat_candidates() -> AuxRadioPins {
    AuxRadioPins {
        reset_bcm: 5,
        cs_bcm: 6,
        busy_bcm: None, // GPIO18 is POWER_EN on SenseCAP M1
        dio1_bcm: 23,
        dio2_bcm: 24,
        verification: VerificationStatus::ConflictUnverified,
    }
}

pub fn refuse_until_identified() -> Result<()> {
    Err(HalError::Other(
        "SX126x model not identified on this module; SPI/BUSY ops blocked".into(),
    ))
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct AuxIdentify {
    pub reset_bcm: u8,
    pub cs_bcm: u8,
    pub spi_no_cs: bool,
    pub status: Option<u8>,
    pub notes: Vec<String>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct LbtSample {
    pub freq_hz: u32,
    pub rssi_dbm: Option<i16>,
    pub status: Option<u8>,
    pub busy: bool,
    pub notes: Vec<String>,
}

/// GPIO6 CS + `SPI_NO_CS` so kernel CE0 stays idle. No TX.
pub fn identify() -> Result<AuxIdentify> {
    #[cfg(not(target_os = "linux"))]
    {
        Err(HalError::Unsupported("Linux GPIO/SPI required"))
    }
    #[cfg(target_os = "linux")]
    {
        linux_xfer(|xfer| {
            let mut st = [0u8; 2];
            xfer(&[OP_GET_STATUS, 0x00], &mut st)?;
            Ok(st[0])
        })
        .map(|(status, notes)| AuxIdentify {
            reset_bcm: 5,
            cs_bcm: 6,
            spi_no_cs: true,
            status,
            notes,
        })
    }
}

/// Instant RSSI at `freq_hz` (AS923-1 band). Listen only.
pub fn lbt_rssi(freq_hz: u32, rssi_target_dbm: i16) -> Result<LbtSample> {
    if !(920_000_000..=925_000_000).contains(&freq_hz) {
        return Err(HalError::FrequencyOutOfBand {
            freq_hz,
            region: sx1302_core::Region::As923_1,
        });
    }
    #[cfg(not(target_os = "linux"))]
    {
        let _ = rssi_target_dbm;
        Err(HalError::Unsupported("Linux GPIO/SPI required"))
    }
    #[cfg(target_os = "linux")]
    {
        linux_xfer(|xfer| {
            xfer(&[OP_SET_STANDBY, STDBY_RC], &mut [0u8; 2])?;
            std::thread::sleep(std::time::Duration::from_millis(10));
            let fr = sx126_freq_reg(freq_hz);
            xfer(
                &[
                    OP_SET_RF_FREQ,
                    (fr >> 24) as u8,
                    (fr >> 16) as u8,
                    (fr >> 8) as u8,
                    fr as u8,
                ],
                &mut [0u8; 5],
            )?;
            xfer(&[OP_SET_RX, 0x00, 0x00, 0x00], &mut [0u8; 4])?;
            std::thread::sleep(std::time::Duration::from_millis(5));
            let mut rssi_buf = [0u8; 2];
            xfer(&[OP_GET_RSSI_INST, 0x00], &mut rssi_buf)?;
            let rssi = -(i16::from(rssi_buf[1]) / 2);
            Ok((rssi_buf[0], rssi, rssi_buf[1]))
        })
        .map(|(opt, mut notes)| {
            let (status, rssi) = match opt {
                Some((st, rssi, raw)) => {
                    notes.push(format!("GET_RSSI_INST raw=0x{raw:02x} => {rssi} dBm"));
                    (Some(st), Some(rssi))
                }
                None => (None, None),
            };
            let busy = rssi.map(|r| r > rssi_target_dbm).unwrap_or(true);
            LbtSample {
                freq_hz,
                rssi_dbm: rssi,
                status,
                busy,
                notes,
            }
        })
    }
}

fn sx126_freq_reg(freq_hz: u32) -> u32 {
    ((u64::from(freq_hz) << 25) / 32_000_000) as u32
}

#[cfg(target_os = "linux")]
fn linux_xfer<T, F>(op: F) -> Result<(Option<T>, Vec<String>)>
where
    F: FnOnce(&mut dyn FnMut(&[u8], &mut [u8]) -> Result<()>) -> Result<T>,
{
    use gpiocdev::line::Value;
    use gpiocdev::Request;
    use std::fs::File;
    use std::os::unix::io::AsRawFd;
    use std::thread::sleep;
    use std::time::Duration;

    let mut notes = vec![
        "SPI_NO_CS: kernel CE0 idle; GPIO6 is SX126 CS".into(),
        "GPIO18 is SenseCAP POWER_EN (not BUSY); not sampled".into(),
        "GPIO13 never driven; no SX126 TX".into(),
    ];
    let cs = Request::builder()
        .on_chip("/dev/gpiochip0")
        .with_line(6u32)
        .as_output(Value::Active)
        .request()
        .map_err(|e| HalError::Io(format!("GPIO6 CS: {e}")))?;
    sleep(Duration::from_millis(1));

    // bcm2712 rejects SPI_NO_CS (EINVAL). Use spidev0.1 so kernel CE0
    // (SX1302) stays idle; GPIO6 is the SX126 CS.
    let spi_path = "/dev/spidev0.1";
    let file = File::options()
        .read(true)
        .write(true)
        .open(spi_path)
        .or_else(|_| {
            File::options()
                .read(true)
                .write(true)
                .open("/dev/spidev0.0")
        })
        .map_err(|e| HalError::Io(format!("spidev: {e}")))?;
    let fd = file.as_raw_fd();
    let mut mode = 0u8;
    let mut bits = 8u8;
    let mut speed = 500_000u32;
    unsafe {
        let _ = get_mode(fd, &mut mode);
        mode &= !SPI_NO_CS;
        set_mode(fd, &mut mode)?;
        set_bits(fd, &mut bits)?;
        set_speed(fd, &mut speed)?;
    }
    notes.push(format!(
        "spi path via fd, mode=0x{mode:02x} speed=500kHz; GPIO6 CS; SPI_NO_CS unsupported on bcm2712"
    ));

    let mut xfer = |tx: &[u8], rx: &mut [u8]| -> Result<()> {
        if tx.len() != rx.len() {
            return Err(HalError::Transport("sx126 tx/rx len".into()));
        }
        let _ = cs.set_value(6, Value::Inactive);
        sleep(Duration::from_micros(50));
        spi_msg(fd, tx, rx, speed, bits)?;
        let _ = cs.set_value(6, Value::Active);
        sleep(Duration::from_micros(50));
        Ok(())
    };

    match op(&mut xfer) {
        Ok(v) => {
            let _ = cs.set_value(6, Value::Active);
            Ok((Some(v), notes))
        }
        Err(e) => {
            notes.push(format!("{e}"));
            let _ = cs.set_value(6, Value::Active);
            Ok((None, notes))
        }
    }
}

#[cfg(target_os = "linux")]
nix::ioctl_write_ptr!(spi_ioc_wr_mode, b'k', 1, u8);
#[cfg(target_os = "linux")]
nix::ioctl_read!(spi_ioc_rd_mode, b'k', 1, u8);
#[cfg(target_os = "linux")]
nix::ioctl_write_ptr!(spi_ioc_wr_bits, b'k', 3, u8);
#[cfg(target_os = "linux")]
nix::ioctl_write_ptr!(spi_ioc_wr_speed, b'k', 4, u32);
#[cfg(target_os = "linux")]
nix::ioctl_write_ptr!(spi_ioc_wr_mode32, b'k', 5, u32);

#[cfg(target_os = "linux")]
unsafe fn get_mode(fd: i32, mode: &mut u8) -> Result<()> {
    unsafe {
        spi_ioc_rd_mode(fd, mode).map_err(|e| HalError::Transport(e.to_string()))?;
    }
    Ok(())
}
#[cfg(target_os = "linux")]
unsafe fn set_mode(fd: i32, mode: &mut u8) -> Result<()> {
    unsafe {
        spi_ioc_wr_mode(fd, mode).map_err(|e| HalError::Transport(e.to_string()))?;
    }
    Ok(())
}
#[cfg(target_os = "linux")]
unsafe fn set_bits(fd: i32, bits: &mut u8) -> Result<()> {
    unsafe {
        spi_ioc_wr_bits(fd, bits).map_err(|e| HalError::Transport(e.to_string()))?;
    }
    Ok(())
}
#[cfg(target_os = "linux")]
unsafe fn set_speed(fd: i32, speed: &mut u32) -> Result<()> {
    unsafe {
        spi_ioc_wr_speed(fd, speed).map_err(|e| HalError::Transport(e.to_string()))?;
    }
    Ok(())
}
#[cfg(target_os = "linux")]
unsafe fn set_mode32(fd: i32, mode: u32) -> Result<()> {
    let mut m = mode;
    unsafe {
        spi_ioc_wr_mode32(fd, &mut m).map_err(|e| HalError::Transport(e.to_string()))?;
    }
    Ok(())
}

#[cfg(target_os = "linux")]
#[repr(C)]
struct SpiIocTransfer {
    tx_buf: u64,
    rx_buf: u64,
    len: u32,
    speed_hz: u32,
    delay_usecs: u16,
    bits_per_word: u8,
    cs_change: u8,
    tx_nbits: u8,
    rx_nbits: u8,
    word_delay_usecs: u8,
    pad: u8,
}

#[cfg(target_os = "linux")]
fn spi_msg(fd: i32, tx: &[u8], rx: &mut [u8], speed: u32, bits: u8) -> Result<()> {
    let mut xfer = SpiIocTransfer {
        tx_buf: tx.as_ptr() as u64,
        rx_buf: rx.as_mut_ptr() as u64,
        len: tx.len() as u32,
        speed_hz: speed,
        delay_usecs: 0,
        bits_per_word: bits,
        cs_change: 0,
        tx_nbits: 0,
        rx_nbits: 0,
        word_delay_usecs: 0,
        pad: 0,
    };
    let size = std::mem::size_of::<SpiIocTransfer>() as u32;
    let request = nix::request_code_write!(b'k', 0, size);
    let ret = unsafe { libc::ioctl(fd, request as libc::c_ulong, &mut xfer) };
    if ret < 0 {
        return Err(HalError::Transport("SPI_IOC_MESSAGE failed".into()));
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn freq_reg_as923() {
        let r = sx126_freq_reg(923_200_000);
        assert!(r > 0);
    }
}
