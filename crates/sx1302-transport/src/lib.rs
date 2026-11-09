//! Native Linux `spidev` transport with a shared-bus manager.
//!
//! Protocol is taken from Semtech `loragw_spi.c` (sx1302_hal
//! `4b42025d1751e04632c0b94160e0d29dbbb222a5`):
//! - mux byte
//! - address high: bit7 = write, bits6..0 = addr[14:8]
//! - address low
//! - dummy byte on reads
//! - payload
//!
//! SX126x GPIO-CS is modelled but not driven until the carrier pin map is
//! verified. Kernel `spidev0.0` (SPI0 CE0 / BCM GPIO8) is the verified
//! SX1302 chip-select on the official WM1302 SPI module.

mod protocol;
#[cfg(target_os = "linux")]
mod spidev;
mod metrics;

pub use protocol::{READ_ACCESS, WRITE_ACCESS, encode_radio, encode_read, encode_write};
#[cfg(target_os = "linux")]
pub use spidev::SPI_NO_CS;
pub use metrics::{SpiMetrics, TransferStats};

use std::sync::{Arc, Mutex};
use sx1302_core::{HalError, Result, SpiMuxTarget};

/// Chip-select ownership policy for the shared WM1302 SPI bus.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ChipSelect {
    /// Kernel SPI0 CE0 (`/dev/spidev0.0`), SX1302 on WM1302 SPI.
    KernelCe0,
    /// GPIO-controlled CS (SX126x on Seeed HAT GPIO6). Must not conflict
    /// with kernel CE1 (`GPIO7`) unless the overlay is confirmed.
    GpioLine { chip: u32, offset: u32 },
}

#[derive(Debug, Clone)]
pub struct SpiConfig {
    pub path: String,
    pub speed_hz: u32,
    pub mode: u8,
    pub bits_per_word: u8,
}

impl Default for SpiConfig {
    fn default() -> Self {
        Self {
            path: "/dev/spidev0.0".into(),
            speed_hz: sx1302_core::SEMTECH_SPI_SPEED_HZ,
            mode: 0,
            bits_per_word: 8,
        }
    }
}

/// Serializes access to one physical SPI controller.
pub struct SpiBus {
    inner: Arc<Mutex<BusInner>>,
    config: SpiConfig,
}

struct BusInner {
    #[cfg(target_os = "linux")]
    dev: spidev::Spidev,
    metrics: SpiMetrics,
    selected: Option<ChipSelect>,
}

impl SpiBus {
    pub fn open(config: SpiConfig) -> Result<Self> {
        #[cfg(not(target_os = "linux"))]
        {
            let _ = config;
            return Err(HalError::Unsupported("Linux spidev required"));
        }
        #[cfg(target_os = "linux")]
        {
            let dev = spidev::Spidev::open(&config)?;
            Ok(Self {
                inner: Arc::new(Mutex::new(BusInner {
                    dev,
                    metrics: SpiMetrics::default(),
                    selected: None,
                })),
                config,
            })
        }
    }

    pub fn config(&self) -> &SpiConfig {
        &self.config
    }

    pub fn sx1302(&self) -> ConcentratorPort<'_> {
        ConcentratorPort {
            bus: self,
            mux: SpiMuxTarget::Sx1302,
            cs: ChipSelect::KernelCe0,
        }
    }

    pub fn radio(&self, mux: SpiMuxTarget) -> RadioPort<'_> {
        RadioPort {
            bus: self,
            mux,
            cs: ChipSelect::KernelCe0,
        }
    }

    pub fn metrics(&self) -> Result<SpiMetrics> {
        let g = self.inner.lock().map_err(|e| HalError::Transport(e.to_string()))?;
        Ok(g.metrics.clone())
    }
}

/// Exclusive view of the concentrator on the shared bus.
pub struct ConcentratorPort<'a> {
    bus: &'a SpiBus,
    mux: SpiMuxTarget,
    #[allow(dead_code)]
    cs: ChipSelect,
}

impl ConcentratorPort<'_> {
    pub fn read_u8(&self, address: u16) -> Result<u8> {
        let mut buf = [0u8; 1];
        self.read_burst(address, &mut buf)?;
        Ok(buf[0])
    }

    pub fn write_u8(&self, address: u16, data: u8) -> Result<()> {
        self.write_burst(address, &[data])
    }

    /// Linux spidev rejects ~8 KiB `SPI_IOC_MESSAGE` (EMSGSIZE). Semtech chunks too.
    const SPI_CHUNK: usize = 1024;

    pub fn read_burst(&self, address: u16, dest: &mut [u8]) -> Result<()> {
        if dest.is_empty() {
            return Err(HalError::Transport("empty burst".into()));
        }
        if dest.len() > Self::SPI_CHUNK {
            let mut off = 0usize;
            while off < dest.len() {
                let n = (dest.len() - off).min(Self::SPI_CHUNK);
                self.read_burst(address + off as u16, &mut dest[off..off + n])?;
                off += n;
            }
            return Ok(());
        }
        let mut g = self.bus.inner.lock().map_err(|e| HalError::Transport(e.to_string()))?;
        if let Some(sel) = g.selected {
            if sel != self.cs {
                return Err(HalError::Transport(
                    "another chip-select is asserted on the shared bus".into(),
                ));
            }
        }
        g.selected = Some(self.cs);
        let started = std::time::Instant::now();
        #[cfg(target_os = "linux")]
        {
            let r = g.dev.read_reg(self.mux, address, dest);
            g.metrics.record_read(dest.len(), started.elapsed(), r.is_ok());
            g.selected = None;
            r
        }
        #[cfg(not(target_os = "linux"))]
        {
            let _ = (address, dest, started);
            g.selected = None;
            Err(HalError::Unsupported("Linux spidev required"))
        }
    }

    pub fn write_burst(&self, address: u16, data: &[u8]) -> Result<()> {
        if data.is_empty() {
            return Err(HalError::Transport("empty burst".into()));
        }
        if data.len() > Self::SPI_CHUNK {
            let mut off = 0usize;
            while off < data.len() {
                let n = (data.len() - off).min(Self::SPI_CHUNK);
                self.write_burst(address + off as u16, &data[off..off + n])?;
                off += n;
            }
            return Ok(());
        }
        let mut g = self.bus.inner.lock().map_err(|e| HalError::Transport(e.to_string()))?;
        g.selected = Some(self.cs);
        let started = std::time::Instant::now();
        #[cfg(target_os = "linux")]
        {
            let r = g.dev.write_reg(self.mux, address, data);
            g.metrics.record_write(data.len(), started.elapsed(), r.is_ok());
            g.selected = None;
            r
        }
        #[cfg(not(target_os = "linux"))]
        {
            let _ = (address, data, started);
            g.selected = None;
            Err(HalError::Unsupported("Linux spidev required"))
        }
    }

    /// Read COMMON_VERSION without GPIO reset (non-destructive probe).
    pub fn probe_version(&self) -> Result<u8> {
        self.read_u8(sx1302_core::SX1302_REG_COMMON_VERSION)
    }

    /// Semtech `lgw_spi_rmw`: mask `leng` bits at `offs`.
    pub fn rmw(&self, address: u16, offs: u8, leng: u8, data: u8) -> Result<()> {
        let old = self.read_u8(address)?;
        let mask = ((1u16 << leng) - 1) as u8;
        let shifted = (data & mask) << offs;
        let new = (old & !(mask << offs)) | shifted;
        self.write_u8(address, new)
    }
}

/// SX1250 / SX126x on the shared SPI mux (`sx1250_spi.c`).
pub struct RadioPort<'a> {
    bus: &'a SpiBus,
    mux: SpiMuxTarget,
    cs: ChipSelect,
}

impl RadioPort<'_> {
    pub fn write_cmd(&self, opcode: u8, data: &[u8]) -> Result<()> {
        let tx = encode_radio(self.mux, opcode, data);
        let mut rx = vec![0u8; tx.len()];
        self.xfer(&tx, &mut rx)
    }

    pub fn read_cmd(&self, opcode: u8, data: &mut [u8]) -> Result<()> {
        let tx = encode_radio(self.mux, opcode, data);
        let mut rx = vec![0u8; tx.len()];
        self.xfer(&tx, &mut rx)?;
        if rx.len() < 2 + data.len() {
            return Err(HalError::Transport("radio short read".into()));
        }
        data.copy_from_slice(&rx[2..2 + data.len()]);
        Ok(())
    }

    fn xfer(&self, tx: &[u8], rx: &mut [u8]) -> Result<()> {
        std::thread::sleep(std::time::Duration::from_millis(1));
        let mut g = self.bus.inner.lock().map_err(|e| HalError::Transport(e.to_string()))?;
        g.selected = Some(self.cs);
        let started = std::time::Instant::now();
        #[cfg(target_os = "linux")]
        {
            let r = g.dev.raw(tx, rx);
            g.metrics.record_write(tx.len(), started.elapsed(), r.is_ok());
            g.selected = None;
            r
        }
        #[cfg(not(target_os = "linux"))]
        {
            let _ = (tx, rx, started);
            g.selected = None;
            Err(HalError::Unsupported("Linux spidev required"))
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn encode_matches_semtech_write_frame() {
        let f = encode_write(SpiMuxTarget::Sx1302, 0x5606, &[0x10]);
        assert_eq!(f[0], 0x00);
        assert_eq!(f[1], WRITE_ACCESS | 0x56);
        assert_eq!(f[2], 0x06);
        assert_eq!(f[3], 0x10);
    }

    #[test]
    fn encode_matches_semtech_read_frame() {
        let f = encode_read(SpiMuxTarget::Sx1302, 0x5606, 1);
        assert_eq!(f.len(), 5);
        assert_eq!(f[1], READ_ACCESS | 0x56);
        assert_eq!(f[2], 0x06);
        assert_eq!(f[3], 0x00);
    }

    #[test]
    fn encode_sx1250_get_status() {
        let f = encode_radio(SpiMuxTarget::RadioA, 0xC0, &[0x00]);
        assert_eq!(f, vec![0x01, 0xC0, 0x00]);
    }
}
