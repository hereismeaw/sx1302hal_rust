//! Linux `spidev` ioctl wrapper. SPI_IOC_MESSAGE(1) full-duplex.
#![cfg(target_os = "linux")]

use crate::protocol::{encode_read, encode_write};
use crate::SpiConfig;
use nix::errno::Errno;
use sx1302_core::{HalError, Result, SpiMuxTarget};
use std::fs::File;
use std::os::unix::io::AsRawFd;

const SPI_IOC_MAGIC: u8 = b'k';
/// Linux `SPI_NO_CS` (`include/uapi/linux/spi/spi.h`).
pub const SPI_NO_CS: u8 = 0x40;

#[repr(C)]
#[derive(Clone, Copy)]
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

nix::ioctl_write_ptr!(spi_ioc_rd_mode, SPI_IOC_MAGIC, 1, u8);
nix::ioctl_write_ptr!(spi_ioc_wr_mode, SPI_IOC_MAGIC, 1, u8);
nix::ioctl_write_ptr!(spi_ioc_rd_bits, SPI_IOC_MAGIC, 3, u8);
nix::ioctl_write_ptr!(spi_ioc_wr_bits, SPI_IOC_MAGIC, 3, u8);
nix::ioctl_write_ptr!(spi_ioc_rd_speed, SPI_IOC_MAGIC, 4, u32);
nix::ioctl_write_ptr!(spi_ioc_wr_speed, SPI_IOC_MAGIC, 4, u32);

/// `SPI_IOC_MESSAGE(1)` — `_IOW(SPI_IOC_MAGIC, 0, sizeof(spi_ioc_transfer))`
fn spi_ioc_message1(fd: i32, xfer: *mut SpiIocTransfer) -> nix::Result<i32> {
    // Layout: _IOC(_IOC_WRITE, 'k', 0, sizeof(spi_ioc_transfer))
    let size = std::mem::size_of::<SpiIocTransfer>() as u32;
    let request = nix::request_code_write!(SPI_IOC_MAGIC, 0, size);
    let ret = unsafe { libc::ioctl(fd, request as libc::c_ulong, xfer) };
    Errno::result(ret)
}

pub struct Spidev {
    file: File,
    speed_hz: u32,
    bits: u8,
}

impl Spidev {
    pub fn open(cfg: &SpiConfig) -> Result<Self> {
        let file = File::options()
            .read(true)
            .write(true)
            .open(&cfg.path)
            .map_err(|e| HalError::Io(format!("{}: {e}", cfg.path)))?;
        let fd = file.as_raw_fd();
        let mut mode = cfg.mode;
        let mut bits = cfg.bits_per_word;
        let mut speed = cfg.speed_hz;
        unsafe {
            spi_ioc_wr_mode(fd, &mut mode).map_err(|e| HalError::Transport(e.to_string()))?;
            spi_ioc_wr_bits(fd, &mut bits).map_err(|e| HalError::Transport(e.to_string()))?;
            spi_ioc_wr_speed(fd, &mut speed).map_err(|e| HalError::Transport(e.to_string()))?;
        }
        Ok(Self {
            file,
            speed_hz: cfg.speed_hz,
            bits: cfg.bits_per_word,
        })
    }

    fn transfer(&mut self, tx: &[u8], rx: &mut [u8]) -> Result<()> {
        if tx.len() != rx.len() {
            return Err(HalError::Transport("tx/rx length mismatch".into()));
        }
        let mut xfer = SpiIocTransfer {
            tx_buf: tx.as_ptr() as u64,
            rx_buf: rx.as_mut_ptr() as u64,
            len: tx.len() as u32,
            speed_hz: self.speed_hz,
            delay_usecs: 0,
            bits_per_word: self.bits,
            cs_change: 0,
            tx_nbits: 0,
            rx_nbits: 0,
            word_delay_usecs: 0,
            pad: 0,
        };
        let n = spi_ioc_message1(self.file.as_raw_fd(), &mut xfer)
            .map_err(|e| HalError::Transport(format!("SPI_IOC_MESSAGE: {e}")))?;
        if n as u32 != xfer.len {
            return Err(HalError::Transport(format!(
                "short SPI transfer: {n} != {}",
                xfer.len
            )));
        }
        Ok(())
    }

    pub fn read_reg(&mut self, mux: SpiMuxTarget, address: u16, dest: &mut [u8]) -> Result<()> {
        let tx = encode_read(mux, address, dest.len());
        let mut rx = vec![0u8; tx.len()];
        self.transfer(&tx, &mut rx)?;
        dest.copy_from_slice(&rx[4..]);
        Ok(())
    }

    pub fn write_reg(&mut self, mux: SpiMuxTarget, address: u16, data: &[u8]) -> Result<()> {
        let tx = encode_write(mux, address, data);
        let mut rx = vec![0u8; tx.len()];
        self.transfer(&tx, &mut rx)
    }

    pub fn raw(&mut self, tx: &[u8], rx: &mut [u8]) -> Result<()> {
        self.transfer(tx, rx)
    }

    /// Kernel leaves CE0 idle; GPIO CS owns the device (SX126x).
    pub fn set_no_cs(&mut self, enable: bool) -> Result<()> {
        let fd = self.file.as_raw_fd();
        let mut mode = 0u8;
        unsafe {
            let _ = spi_ioc_rd_mode(fd, &mut mode);
            if enable {
                mode |= SPI_NO_CS;
            } else {
                mode &= !SPI_NO_CS;
            }
            spi_ioc_wr_mode(fd, &mut mode).map_err(|e| HalError::Transport(e.to_string()))?;
        }
        Ok(())
    }
}
