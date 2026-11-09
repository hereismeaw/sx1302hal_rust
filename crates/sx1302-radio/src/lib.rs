//! SX1250 RF front-end via SX1302 SPI mux (Semtech sx1250_spi.c / loragw_sx1250.c).

use serde::{Deserialize, Serialize};
use sx1302_core::{HalError, Result, SpiMuxTarget};
use sx1302_transport::{ConcentratorPort, RadioPort, SpiBus};

/// Semtech `sx1250_op_code_t`.
pub const GET_STATUS: u8 = 0xC0;
pub const SET_STANDBY: u8 = 0x80;
pub const SET_RX: u8 = 0x82;
pub const SET_TX: u8 = 0x83;
pub const SET_RF_FREQUENCY: u8 = 0x86;
pub const WRITE_REGISTER: u8 = 0x0D;
pub const CALIBRATE: u8 = 0x89;
pub const CALIBRATE_IMAGE: u8 = 0x98;
pub const GET_DEVICE_ERRORS: u8 = 0x17;
pub const STDBY_RC: u8 = 0x00;
pub const STDBY_XOSC: u8 = 0x01;

pub const COMMON_CTRL0: u16 = 0x5601;
pub const AGC_MCU_CTRL: u16 = 0x5780;
pub const AGC_MCU_RF_EN_A: u16 = 0x5783;
pub const AGC_MCU_RF_EN_B: u16 = 0x5784;
pub const CLK_CTRL: u16 = 0x57C0;

/// Dummy frequency used by Semtech `util_chip_id` (Hz). RX only.
pub const CHIP_ID_DUMMY_FREQ_HZ: u32 = 868_500_000;

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum RfChain {
    A = 0,
    B = 1,
}

impl RfChain {
    pub fn mux(self) -> SpiMuxTarget {
        match self {
            RfChain::A => SpiMuxTarget::RadioA,
            RfChain::B => SpiMuxTarget::RadioB,
        }
    }
    fn rf_en_addr(self) -> u16 {
        match self {
            RfChain::A => AGC_MCU_RF_EN_A,
            RfChain::B => AGC_MCU_RF_EN_B,
        }
    }
    fn mode_offs(self) -> u8 {
        match self {
            RfChain::A => 0,
            RfChain::B => 1,
        }
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct RadioStatus {
    pub chain: RfChain,
    pub raw: u8,
    pub chip_mode: u8,
}

/// `sx1302_radio_reset` + `sx1302_radio_set_mode` for SX1250. PA/LNA stay off.
pub fn reset_sx1250(port: &ConcentratorPort<'_>, chain: RfChain) -> Result<()> {
    port.rmw(COMMON_CTRL0, 4, 1, 0)?; // CLK32_RIF_CTRL = SPI clock
    let en = chain.rf_en_addr();
    port.rmw(en, 2, 1, 1)?; // RADIO_EN
    port.rmw(en, 1, 1, 0)?; // PA_EN
    port.rmw(en, 0, 1, 0)?; // LNA_EN
    port.rmw(en, 3, 1, 1)?; // RADIO_RST
    std::thread::sleep(std::time::Duration::from_millis(500));
    port.rmw(en, 3, 1, 0)?;
    std::thread::sleep(std::time::Duration::from_millis(10));
    port.rmw(en, 3, 1, 1)?; // SX1250 extra reset pulse
    std::thread::sleep(std::time::Duration::from_millis(10));
    port.rmw(COMMON_CTRL0, chain.mode_offs(), 1, 1)?; // SX1250 mode
    Ok(())
}

pub fn get_status(bus: &SpiBus, chain: RfChain) -> Result<RadioStatus> {
    let radio = bus.radio(chain.mux());
    let mut buf = [0u8];
    radio.read_cmd(GET_STATUS, &mut buf)?;
    Ok(RadioStatus {
        chain,
        raw: buf[0],
        chip_mode: (buf[0] >> 4) & 0x07,
    })
}

pub fn host_fe_safe(port: &ConcentratorPort<'_>) -> Result<()> {
    port.rmw(COMMON_CTRL0, 3, 1, 1)?; // HOST_RADIO_CTRL
    port.rmw(AGC_MCU_CTRL, 3, 1, 1)?; // FORCE_HOST_FE_CTRL
    port.rmw(AGC_MCU_RF_EN_A, 1, 1, 0)?;
    port.rmw(AGC_MCU_RF_EN_A, 0, 1, 0)?;
    port.rmw(AGC_MCU_RF_EN_B, 1, 1, 0)?;
    port.rmw(AGC_MCU_RF_EN_B, 0, 1, 0)?;
    Ok(())
}

/// `sx1302_radio_clock_select` after the radio is in RX.
pub fn clock_select(port: &ConcentratorPort<'_>, chain: RfChain) -> Result<()> {
    match chain {
        RfChain::A => {
            port.rmw(CLK_CTRL, 0, 1, 1)?;
            port.rmw(CLK_CTRL, 1, 1, 0)?;
        }
        RfChain::B => {
            port.rmw(CLK_CTRL, 0, 1, 0)?;
            port.rmw(CLK_CTRL, 1, 1, 1)?;
        }
    }
    port.rmw(CLK_CTRL, 2, 1, 1)?; // CLKDIV_EN
    port.rmw(COMMON_CTRL0, 4, 1, 1)?; // CLK32_RIF_CTRL = radio 32 MHz
    Ok(())
}

fn write_reg3(radio: &RadioPort<'_>, a: u8, b: u8, c: u8) -> Result<()> {
    radio.write_cmd(WRITE_REGISTER, &[a, b, c])
}

/// `sx1250_setup` without image calibration (region not locked). Ends in SET_RX.
pub fn setup_rx_clock(bus: &SpiBus, chain: RfChain, freq_hz: u32) -> Result<()> {
    let radio = bus.radio(chain.mux());
    radio.write_cmd(SET_STANDBY, &[STDBY_RC])?;
    std::thread::sleep(std::time::Duration::from_millis(10));
    let mut st = [0u8];
    radio.read_cmd(GET_STATUS, &mut st)?;
    if ((st[0] >> 4) & 0x07) != 0x02 {
        return Err(HalError::Other(format!(
            "SX1250 {chain:?} not STANDBY_RC (status=0x{:02x})",
            st[0]
        )));
    }
    radio.write_cmd(CALIBRATE, &[0x7F])?;
    std::thread::sleep(std::time::Duration::from_millis(10));
    radio.write_cmd(SET_STANDBY, &[STDBY_XOSC])?;
    std::thread::sleep(std::time::Duration::from_millis(10));
    radio.read_cmd(GET_STATUS, &mut st)?;
    if ((st[0] >> 4) & 0x07) != 0x03 {
        return Err(HalError::Other(format!(
            "SX1250 {chain:?} not STANDBY_XOSC (status=0x{:02x})",
            st[0]
        )));
    }
    write_reg3(&radio, 0x06, 0xA1, 0x01)?;
    write_reg3(&radio, 0x06, 0xA2, 0x00)?;
    write_reg3(&radio, 0x06, 0xA3, 0x00)?;
    write_reg3(&radio, 0x05, 0x82, 0x00)?;
    write_reg3(&radio, 0x05, 0x83, 0x00)?;
    write_reg3(&radio, 0x05, 0x84, 0x00)?;
    write_reg3(&radio, 0x05, 0x85, 0x00)?;
    write_reg3(&radio, 0x05, 0x80, 0x00)?;
    write_reg3(&radio, 0x08, 0xB6, 0x2A)?;
    let freq_reg = ((u64::from(freq_hz) << 25) / 32_000_000) as u32;
    radio.write_cmd(
        SET_RF_FREQUENCY,
        &[
            (freq_reg >> 24) as u8,
            (freq_reg >> 16) as u8,
            (freq_reg >> 8) as u8,
            freq_reg as u8,
        ],
    )?;
    radio.write_cmd(WRITE_REGISTER, &[0x08, 0x8F, 0x00, 0x00, 0x00])?;
    calibrate_image(&radio, freq_hz)?;
    radio.write_cmd(SET_RX, &[0xFF, 0xFF, 0xFF])?;
    // WM1302 / SenseCAP M1: SX1250 single-ended RF (Semtech `single_input_mode`).
    write_reg3(&radio, 0x08, 0xE2, 0x0D)?;
    write_reg3(&radio, 0x05, 0x87, 0x0B)?; // FPGA_MODE_RX
    Ok(())
}

/// Semtech `sx1250_calibrate` — 902–928 MHz uses 0xE1/0xE9 (covers AS923-1).
fn calibrate_image(radio: &RadioPort<'_>, freq_hz: u32) -> Result<()> {
    let (lo, hi) = if (902_000_000..928_000_000).contains(&freq_hz) {
        (0xE1u8, 0xE9u8)
    } else if (863_000_000..870_000_000).contains(&freq_hz) {
        (0xD7, 0xDB)
    } else {
        return Err(HalError::Other(format!(
            "SX1250 CALIBRATE_IMAGE: no table for {freq_hz} Hz"
        )));
    };
    radio.write_cmd(CALIBRATE_IMAGE, &[lo, hi])?;
    std::thread::sleep(std::time::Duration::from_millis(10));
    let mut err = [0u8; 3];
    radio.read_cmd(GET_DEVICE_ERRORS, &mut err)?;
    if (err[2] >> 4) & 1 != 0 {
        return Err(HalError::Other("SX1250 image calibration error".into()));
    }
    Ok(())
}

/// Host-controlled PA on one chain (FORCE_HOST_FE remains set).
pub fn host_pa_enable(port: &ConcentratorPort<'_>, chain: RfChain, on: bool) -> Result<()> {
    port.rmw(AGC_MCU_CTRL, 3, 1, 1)?;
    let v = if on { 1 } else { 0 };
    port.rmw(chain.rf_en_addr(), 1, 1, v)?;
    Ok(())
}

/// SX1250 SET_TX at `freq_hz`. Caller must have validated the region.
pub fn enable_tx(bus: &SpiBus, chain: RfChain, freq_hz: u32) -> Result<()> {
    setup_rx_clock(bus, chain, freq_hz)?;
    let radio = bus.radio(chain.mux());
    radio.write_cmd(SET_TX, &[0xFF, 0xFF, 0xFF])?;
    Ok(())
}

/// Switch SX1250 RX/TX without full calibration (radio already running).
pub fn set_tx_continuous(bus: &SpiBus, chain: RfChain) -> Result<()> {
    let radio = bus.radio(chain.mux());
    radio.write_cmd(SET_TX, &[0xFF, 0xFF, 0xFF])?;
    Ok(())
}

pub fn set_rx_continuous(bus: &SpiBus, chain: RfChain) -> Result<()> {
    let radio = bus.radio(chain.mux());
    radio.write_cmd(SET_RX, &[0xFF, 0xFF, 0xFF])?;
    Ok(())
}
