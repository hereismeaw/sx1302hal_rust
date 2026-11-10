//! Native SX1302 concentrator instance. No C HAL, no FFI.

use serde::{Deserialize, Serialize};
use sx1302_core::{
    ChipIdentity, HalError, Region, Result, SX1302_VERSION_EXPECTED, SpiMuxTarget,
};
use sx1302_firmware::{
    AGC_MEM_ADDR, AGC_MCU_CTRL, AGC_MCU_STATUS, ARB_MEM_ADDR, ARB_MCU_CTRL, ARB_MCU_STATUS,
    COMMON_PAGE, MCU_FW_SIZE,
};
use sx1302_radio::{self as radio, RadioStatus, RfChain};
use sx1302_transport::{ConcentratorPort, SpiBus, SpiConfig};

mod lorawan;
mod pkt;
mod plan;
mod start;
mod timestamp;
pub use lorawan::{MType, MacRev, PhyHdr};
pub use pkt::{
    legacy_timestamp_correction_us, parse_rx_buffer, time_on_air_us, FskTx, RxDump, RxModulation,
    RxPacket, TxGain, TxGainLut, TxRequest, TxTrigger, RSSI_OFFSET_SX1250,
};
pub use plan::{
    bw_code_to_hz, bw_hz_to_code, ChainKind, ChainMap, ChannelPlan, FskChan, LoraStdChan,
    MultiSfChan, RssiTcomp, BW_125KHZ, BW_250KHZ, BW_500KHZ,
};
pub use timestamp::TimestampDump;

pub struct Concentrator {
    bus: SpiBus,
    /// Active RX plan (set by `start`), used to resolve RX packet frequencies.
    plan: std::sync::Mutex<ChannelPlan>,
    /// 27-bit µs counter wrap tracking (`timestamp_counter_t`).
    ts: std::sync::Mutex<timestamp::TsWrap>,
    /// Board temperature (°C) for RSSI compensation; `None` = no compensation.
    temperature: std::sync::Mutex<Option<f32>>,
}

impl Concentrator {
    /// Feed the board temperature (STTS751) used for `rssi_tcomp`.
    pub fn set_temperature(&self, t: Option<f32>) {
        *self.temperature.lock().unwrap() = t;
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DetectReport {
    pub identity: Option<ChipIdentity>,
    pub dummy: Option<u8>,
    pub reset_performed: bool,
    pub radios: Vec<RadioStatus>,
    pub clock_chain: Option<RfChain>,
    pub notes: Vec<String>,
}

impl Concentrator {
    pub fn open(config: SpiConfig) -> Result<Self> {
        Ok(Self {
            bus: SpiBus::open(config)?,
            plan: std::sync::Mutex::new(ChannelPlan::as923_1_default()),
            ts: std::sync::Mutex::new(timestamp::TsWrap::default()),
            temperature: std::sync::Mutex::new(None),
        })
    }

    pub fn bus(&self) -> &SpiBus {
        &self.bus
    }

    /// SPI identity only. No radio reset.
    pub fn detect(&self) -> Result<DetectReport> {
        self.identify(false)
    }

    /// Internal SX1250 reset + RX clock (no Pi GPIO, no TX). Then OTP EUI.
    pub fn bringup(&self) -> Result<DetectReport> {
        self.identify(true)
    }

    fn identify(&self, radios: bool) -> Result<DetectReport> {
        let port = self.bus.sx1302();
        let mut notes = vec![
            "Call `sx1302-cli gpio reset` to drive GPIO18 POWER_EN, GPIO17 RESET, GPIO5 SX126 RESET.".into(),
            "GPIO13 AD5338R is never driven.".into(),
        ];
        let version = match port.probe_version() {
            Ok(v) => v,
            Err(e) => {
                notes.push(format!("SPI probe failed: {e}"));
                return Ok(DetectReport {
                    identity: None,
                    dummy: None,
                    reset_performed: false,
                    radios: vec![],
                    clock_chain: None,
                    notes,
                });
            }
        };
        let dummy = port.read_u8(sx1302_core::SX1302_REG_COMMON_DUMMY).ok();
        let matches = version == SX1302_VERSION_EXPECTED;
        if !matches {
            notes.push(format!(
                "COMMON_VERSION=0x{version:02x} (expected 0x{SX1302_VERSION_EXPECTED:02x})"
            ));
        }

        let mut radio_status = Vec::new();
        let mut clock_chain = None;
        if radios && matches {
            match self.bringup_radios(&port, &mut notes, &mut radio_status) {
                Ok(c) => clock_chain = c,
                Err(e) => notes.push(format!("radio bring-up: {e}")),
            }
        }

        let _ = port.write_u8(sx1302_core::SX1302_REG_OTP_CFG_ACCESS_MODE, 0);
        let eui = self.read_eui(&port).ok();
        let model_id = self
            .read_otp_byte(&port, sx1302_core::SX1302_OTP_MODEL_ID_ADDR)
            .ok();
        if let Some(id) = eui {
            let bytes = id.to_be_bytes();
            if bytes.iter().all(|&b| b == bytes[0]) {
                notes.push(format!(
                    "OTP EUI 0x{id:016x} repeated 0x{:02x}; not used as gateway ID",
                    bytes[0]
                ));
            } else {
                notes.push(format!("OTP EUI 0x{id:016x}"));
            }
        }

        Ok(DetectReport {
            identity: Some(ChipIdentity {
                version,
                matches_sx1302: matches,
                spi_path: self.bus.config().path.clone(),
                mux: SpiMuxTarget::Sx1302,
                eui,
                model_id,
            }),
            dummy,
            reset_performed: radios,
            radios: radio_status,
            clock_chain,
            notes,
        })
    }

    fn bringup_radios(
        &self,
        port: &ConcentratorPort<'_>,
        notes: &mut Vec<String>,
        out: &mut Vec<RadioStatus>,
    ) -> Result<Option<RfChain>> {
        radio::host_fe_safe(port)?;
        notes.push("HOST_RADIO_CTRL + FORCE_HOST_FE; PA/LNA forced off".into());

        for chain in [RfChain::A, RfChain::B] {
            radio::reset_sx1250(port, chain)?;
            match radio::get_status(&self.bus, chain) {
                Ok(st) => {
                    notes.push(format!(
                        "SX1250 {chain:?} GET_STATUS=0x{:02x} chip_mode={}",
                        st.raw, st.chip_mode
                    ));
                    out.push(st);
                }
                Err(e) => notes.push(format!("SX1250 {chain:?} GET_STATUS failed: {e}")),
            }
        }

        let mut clock = None;
        for chain in [RfChain::A, RfChain::B] {
            match radio::setup_rx_clock(&self.bus, chain, radio::CHIP_ID_DUMMY_FREQ_HZ) {
                Ok(()) => {
                    notes.push(format!(
                        "SX1250 {chain:?} STANDBY_XOSC + SET_RX @ {} Hz (RX clock, no TX)",
                        radio::CHIP_ID_DUMMY_FREQ_HZ
                    ));
                    radio::clock_select(port, chain)?;
                    notes.push(format!("SX1302 clock from radio {chain:?}"));
                    clock = Some(chain);
                    break;
                }
                Err(e) => notes.push(format!("SX1250 {chain:?} setup: {e}")),
            }
        }
        Ok(clock)
    }

    pub(crate) fn read_otp_byte(&self, port: &ConcentratorPort<'_>, addr: u8) -> Result<u8> {
        port.write_u8(sx1302_core::SX1302_REG_OTP_BYTE_ADDR, addr)?;
        port.read_u8(sx1302_core::SX1302_REG_OTP_RD_DATA)
    }

    fn read_eui(&self, port: &ConcentratorPort<'_>) -> Result<u64> {
        let mut eui = 0u64;
        for i in 0u8..8 {
            let val = self.read_otp_byte(port, i)?;
            eui |= u64::from(val) << (56 - (i * 8));
        }
        Ok(eui)
    }

    /// Load Semtech AGC then ARB MCU firmware (8192 bytes each).
    pub fn load_mcu_firmware(&self, agc: &[u8], arb: &[u8]) -> Result<Vec<String>> {
        if agc.len() != MCU_FW_SIZE || arb.len() != MCU_FW_SIZE {
            return Err(HalError::Other("firmware must be 8192 bytes".into()));
        }
        let port = self.bus.sx1302();
        let mut notes = Vec::new();
        self.load_one_mcu(&port, agc, AGC_MEM_ADDR, AGC_MCU_CTRL, AGC_MCU_STATUS, "AGC")?;
        notes.push("AGC firmware loaded and verified".into());
        self.load_one_mcu(&port, arb, ARB_MEM_ADDR, ARB_MCU_CTRL, ARB_MCU_STATUS, "ARB")?;
        notes.push("ARB firmware loaded and verified".into());
        Ok(notes)
    }

    fn load_one_mcu(
        &self,
        port: &ConcentratorPort<'_>,
        fw: &[u8],
        mem: u16,
        ctrl: u16,
        status: u16,
        name: &str,
    ) -> Result<()> {
        port.write_u8(COMMON_PAGE, 0)?;
        port.rmw(ctrl, 2, 1, 1)?; // MCU_CLEAR
        port.rmw(ctrl, 1, 1, 1)?; // HOST_PROG
        port.write_burst(mem, fw)?;
        let mut back = vec![0u8; MCU_FW_SIZE];
        port.read_burst(mem, &mut back)?;
        if back != fw {
            return Err(HalError::Other(format!("{name} firmware verify mismatch")));
        }
        port.rmw(ctrl, 1, 1, 0)?; // HOST_PROG off
        port.rmw(ctrl, 2, 1, 0)?; // MCU_CLEAR off
        // AGC CLK_EN is bit4; ARB CLK_EN is bit5 (bit4 is ARB RADIO_RST).
        if ctrl == ARB_MCU_CTRL {
            port.rmw(ctrl, 4, 1, 0)?; // ARB RADIO_RST off
            port.rmw(ctrl, 5, 1, 1)?; // ARB CLK_EN
        } else {
            port.rmw(ctrl, 4, 1, 1)?; // AGC CLK_EN
        }
        std::thread::sleep(std::time::Duration::from_millis(10));
        let st = port.read_u8(status)?;
        let _ = st;
        Ok(())
    }

    /// Bring radios up at the region default frequency, then SX1250 SET_TX + PA_EN.
    pub fn start_tx(&self, region: Region, freq_hz: Option<u32>) -> Result<DetectReport> {
        let freq = freq_hz.unwrap_or(region.default_tx_hz());
        if !region.contains(freq) {
            return Err(HalError::FrequencyOutOfBand {
                freq_hz: freq,
                region,
            });
        }
        let mut report = self.bringup()?;
        let port = self.bus.sx1302();
        let chain = report.clock_chain.unwrap_or(RfChain::A);
        radio::enable_tx(&self.bus, chain, freq)?;
        radio::host_pa_enable(&port, chain, true)?;
        report.notes.push(format!(
            "TX enabled: region {} freq {freq} Hz chain {chain:?} SET_TX + PA_EN (host FE)",
            region.as_str()
        ));
        Ok(report)
    }

    pub fn otp_eui(&self) -> Result<u64> {
        let port = self.bus.sx1302();
        let _ = port.write_u8(sx1302_core::SX1302_REG_OTP_CFG_ACCESS_MODE, 0);
        self.read_eui(&port)
    }

    pub fn require_detected(&self) -> Result<ChipIdentity> {
        let r = self.detect()?;
        match r.identity {
            Some(id) if id.matches_sx1302 => Ok(id),
            Some(id) => Err(HalError::ChipNotDetected {
                got: id.version,
                expected: SX1302_VERSION_EXPECTED,
            }),
            None => Err(HalError::ChipNotDetected {
                got: 0,
                expected: SX1302_VERSION_EXPECTED,
            }),
        }
    }
}
