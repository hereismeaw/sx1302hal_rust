//! `lgw_start` subset: firmware, AGC handshake, AS923-1 channelizer, modem enable.
//! Register addresses from Semtech `loragw_reg.c` SHA 4b42025.

use crate::plan::{bw_hz_to_code, ChannelPlan, BW_125KHZ};
use crate::Concentrator;
use sx1302_core::{HalError, Result};
use sx1302_firmware::{
    AGC_MCU_CTRL, AGC_MCU_STATUS, ARB_MCU_CTRL, ARB_MCU_STATUS, FW_VERSION_AGC_SX1250,
    FW_VERSION_ARB, MCU_FW_SIZE,
};
use sx1302_radio::{self as radio, RfChain};
use sx1302_registers::{
    if_hz_to_reg, OTP_MODEM_EN_0, OTP_MODEM_EN_1,
    SX1302_REG_ARB_MCU_BASE_ADDR, SX1302_REG_COMMON_BASE_ADDR, SX1302_REG_RADIO_FE_BASE_ADDR,
    SX1302_REG_RX_TOP_BASE_ADDR, SX1302_REG_RX_TOP_LORA_SERVICE_FSK_BASE_ADDR,
};
use sx1302_transport::ConcentratorPort;
use std::time::{Duration, Instant};

const AGC_MB_WR0: u16 = 0x578C;
const AGC_MB_RD0: u16 = 0x5790;
const AGC_RADIO_A_INIT_DONE: u8 = 0x80;
const AGC_RADIO_B_INIT_DONE: u8 = 0x20;
const COMMON_GEN: u16 = SX1302_REG_COMMON_BASE_ADDR + 5;
const COMMON_CTRL0: u16 = SX1302_REG_COMMON_BASE_ADDR + 1;
const RX_TOP: u16 = SX1302_REG_RX_TOP_BASE_ADDR;
const LSV: u16 = SX1302_REG_RX_TOP_LORA_SERVICE_FSK_BASE_ADDR;
const RADIO_FE: u16 = SX1302_REG_RADIO_FE_BASE_ADDR;
const ARB: u16 = SX1302_REG_ARB_MCU_BASE_ADDR;
/// SX1302 GPIO block (`SX1302_REG_GPIO_*`, base 0x5640).
const GPIO_DIR_L: u16 = 0x5641;
const GPIO_OUT_L: u16 = 0x5643;
const GPIO_SEL_0: u16 = 0x5648;

/// `calculate_freq_to_time_drift`: bw·2^20 / radio_freq, normalised to >= 2048.
fn freq_to_time_drift(bw_hz: u32, radio_hz: u32) -> (u16, u8) {
    let mut mantissa = (u64::from(bw_hz) << 20) / u64::from(radio_hz);
    let mut exponent = 0u8;
    while mantissa < 2048 {
        exponent += 1;
        mantissa <<= 1;
    }
    (mantissa as u16, exponent)
}

pub struct AgcParams {
    pub ana_min: u8,
    pub ana_max: u8,
    pub ana_thresh_l: u8,
    pub ana_thresh_h: u8,
    pub dec_attn_min: u8,
    pub dec_attn_max: u8,
    pub dec_thresh_l: u8,
    pub dec_thresh_h1: u8,
    pub dec_thresh_h2: u8,
    pub chan_attn_min: u8,
    pub chan_attn_max: u8,
    pub chan_thresh_l: u8,
    pub chan_thresh_h: u8,
    pub device_sel: u8,
    pub hp_max: u8,
    pub pa_duty: u8,
}

/// `agc_params_sx1250` from `loragw_agc_params.h`.
pub const AGC_SX1250: AgcParams = AgcParams {
    ana_min: 1,
    ana_max: 13,
    ana_thresh_l: 3,
    ana_thresh_h: 12,
    dec_attn_min: 4,
    dec_attn_max: 15,
    dec_thresh_l: 40,
    dec_thresh_h1: 80,
    dec_thresh_h2: 90,
    chan_attn_min: 4,
    chan_attn_max: 14,
    chan_thresh_l: 52,
    chan_thresh_h: 132,
    device_sel: 0,
    hp_max: 7,
    pa_duty: 4,
};

fn mb_wr(port: &ConcentratorPort<'_>, mailbox: u8, value: u8) -> Result<()> {
    port.write_u8(AGC_MB_WR0 - u16::from(mailbox), value)
}

fn mb_rd(port: &ConcentratorPort<'_>, mailbox: u8) -> Result<u8> {
    port.read_u8(AGC_MB_RD0 - u16::from(mailbox))
}

fn wait_status(port: &ConcentratorPort<'_>, want: u8) -> Result<()> {
    let t0 = Instant::now();
    loop {
        let st = port.read_u8(AGC_MCU_STATUS)?;
        if st == want {
            return Ok(());
        }
        if t0.elapsed() > Duration::from_secs(3) {
            return Err(HalError::Other(format!(
                "AGC status timeout (got 0x{st:02x} want 0x{want:02x})"
            )));
        }
        std::thread::sleep(Duration::from_millis(2));
    }
}

impl Concentrator {
    /// Start with the default AS923-1 plan ([`ChannelPlan::as923_1_default`]).
    pub fn start_as923_1(&self) -> Result<Vec<String>> {
        self.start(&ChannelPlan::as923_1_default())
    }

    pub fn plan(&self) -> ChannelPlan {
        self.plan.lock().unwrap().clone()
    }

    /// `lgw_start` with an explicit RX channel plan. The band is whichever
    /// sub-GHz window both radios sit in (`ChannelPlan::band`).
    pub fn start(&self, plan: &ChannelPlan) -> Result<Vec<String>> {
        let region = plan.band()?;
        *self.plan.lock().unwrap() = plan.clone();
        let [freq_a, freq_b] = plan.radio_hz;
        let mut notes = Vec::new();
        let port = self.bus.sx1302();

        port.write_u8(GPIO_OUT_L, 0x00)?; // sx1302_set_gpio(0): LEDs off during start
        radio::host_fe_safe(&port)?;
        for chain in [RfChain::A, RfChain::B] {
            radio::reset_sx1250(&port, chain)?;
        }
        radio::setup_rx(&self.bus, RfChain::A, freq_a, plan.single_input_mode[0])?;
        radio::setup_rx(&self.bus, RfChain::B, freq_b, plan.single_input_mode[1])?;
        radio::clock_select(&port, RfChain::A)?;
        port.rmw(COMMON_CTRL0, 3, 1, 0)?; // HOST_RADIO_CTRL = AGC (Semtech lgw_start)
        notes.push(format!("radios in RX @ {freq_a}/{freq_b} Hz ({region})"));

        port.rmw(0x5785, 4, 4, 0x04)?; // PA_LUT A
        port.rmw(0x5785, 0, 4, 0x02)?; // LNA_LUT A
        port.write_u8(0x5786, 0x42)?; // LUT B PA=4 LNA=2
        self.configure_radio_fe(&port)?;
        self.configure_channelizer(&port, plan)?;
        self.configure_correlator(&port)?;
        self.configure_lora_modem(&port, freq_a)?;
        if let Some(s) = &plan.lora_std {
            self.configure_lora_service(&port, plan, s)?;
        }
        if let Some(f) = &plan.fsk {
            self.configure_fsk(&port, plan, f)?;
        }
        port.rmw(RX_TOP + 142, 0, 5, 6)?;
        port.rmw(RX_TOP + 143, 0, 5, 8)?;
        port.rmw(SX1302_REG_COMMON_BASE_ADDR + 2, 1, 1, 0)?;
        port.rmw(SX1302_REG_COMMON_BASE_ADDR + 2, 3, 1, 0)?;
        port.rmw(RX_TOP + 135, 1, 1, 0)?;
        port.write_u8(RX_TOP + 192, 0x00)?;
        port.rmw(RX_TOP + 192, 2, 1, 1)?; // LEGACY_TIMESTAMP
        Self::config_gpio(&port)?;
        let common_gen = Self::common_gen(plan);
        port.write_u8(COMMON_GEN, common_gen)?;
        notes.push(format!(
            "{} multi-SF chains{}{} + legacy TS before AGC",
            plan.multisf.len(),
            plan.lora_std
                .map(|s| format!(" + LoRa std {} Hz SF{} BW{}k", s.freq_hz, s.sf, s.bw_hz / 1000))
                .unwrap_or_default(),
            plan.fsk
                .map(|f| format!(" + FSK {} Hz {} b/s", f.freq_hz, f.datarate))
                .unwrap_or_default(),
        ));

        let agc = sx1302_firmware::AGC_FW_SX1250;
        let arb = sx1302_firmware::ARB_FW_BIN;
        if agc.len() != MCU_FW_SIZE || arb.len() != MCU_FW_SIZE {
            return Err(HalError::Other("embedded firmware size".into()));
        }
        self.load_mcu_firmware(agc, arb)?;
        notes.push("AGC+ARB firmware loaded".into());
        port.rmw(AGC_MCU_CTRL, 3, 1, 0)?;
        self.agc_start(&port)?;
        notes.push(format!(
            "AGC start handshake done (fw version {FW_VERSION_AGC_SX1250})"
        ));
        self.arb_start(&port)?;
        notes.push(format!("ARB start handshake done (fw version {FW_VERSION_ARB})"));
        // Re-arm SX1250 RX after AGC (host SPI), single-input, LNA on.
        port.rmw(COMMON_CTRL0, 3, 1, 1)?;
        radio::setup_rx(&self.bus, RfChain::A, freq_a, plan.single_input_mode[0])?;
        radio::setup_rx(&self.bus, RfChain::B, freq_b, plan.single_input_mode[1])?;
        radio::clock_select(&port, RfChain::A)?;
        let st_a = radio::get_status(&self.bus, RfChain::A)?;
        let st_b = radio::get_status(&self.bus, RfChain::B)?;
        notes.push(format!(
            "SX1250 A status=0x{:02x} mode={} B status=0x{:02x} mode={} (5=RX)",
            st_a.raw, st_a.chip_mode, st_b.raw, st_b.chip_mode
        ));
        port.rmw(COMMON_CTRL0, 3, 1, 0)?;
        port.rmw(AGC_MCU_CTRL, 3, 1, 1)?;
        port.write_u8(0x5783, 0x0D)?;
        port.write_u8(0x5784, 0x0D)?;
        port.rmw(ARB_MCU_CTRL, 4, 1, 0)?; // ARB RADIO_RST off
        port.rmw(ARB_MCU_CTRL, 5, 1, 1)?; // ARB CLK_EN
        self.configure_channelizer(&port, plan)?;
        self.configure_correlator(&port)?;
        port.write_u8(OTP_MODEM_EN_0, 0xFF)?;
        port.write_u8(OTP_MODEM_EN_1, 0xFF)?;
        port.rmw(RX_TOP + 142, 0, 5, 6)?;
        port.rmw(RX_TOP + 143, 0, 5, 8)?;
        port.write_u8(COMMON_GEN, common_gen)?;
        port.rmw(SX1302_REG_COMMON_BASE_ADDR + 2, 1, 1, 0)?;
        port.rmw(SX1302_REG_COMMON_BASE_ADDR + 2, 3, 1, 0)?;
        port.rmw(RX_TOP + 135, 1, 1, 0)?;
        port.write_u8(GPIO_OUT_L, 0x01)?; // CONFIG_DONE -> "Config" LED on
        notes.push("HAT LEDs: GPIO0 CONFIG_DONE on, GPIO2 TX ON, GPIO4 RX toggle".into());
        let rssi_a = port.read_u8(RADIO_FE + 5)?;
        let rssi_dec = port.read_u8(RADIO_FE + 4)?;
        let rf_en_a = port.read_u8(0x5783)?;
        let rf_en_b = port.read_u8(0x5784)?;
        let host_ctrl = port.read_u8(COMMON_CTRL0)?;
        notes.push(format!(
            "radio_fe + OTP_MODEM_EN + public 0x34 sync + COMMON_GEN rssi_bb={rssi_a} rssi_dec={rssi_dec} rf_en_a=0x{rf_en_a:02x} rf_en_b=0x{rf_en_b:02x} ctrl0=0x{host_ctrl:02x}"
        ));
        Ok(notes)
    }

    pub fn set_swap_iq(&self, on: bool) -> Result<()> {
        let port = self.bus.sx1302();
        let v = u8::from(on);
        port.rmw(SX1302_REG_COMMON_BASE_ADDR + 2, 1, 1, v)?;
        port.rmw(SX1302_REG_COMMON_BASE_ADDR + 2, 3, 1, v)?;
        port.rmw(RX_TOP + 135, 1, 1, v)?;
        Ok(())
    }

    fn configure_correlator(&self, port: &ConcentratorPort<'_>) -> Result<()> {
        for i in 0..8u16 {
            let cfg1 = RX_TOP + 37 + i * 7;
            port.write_u8(cfg1 + 1, 52)?; // ACC_PNR
            port.write_u8(cfg1 + 3, 24)?; // MSP_PNR
            port.rmw(cfg1 + 5, 3, 3, 7)?; // MSP_PEAK_NB
            port.rmw(cfg1 + 6, 2, 3, 5)?; // MSP2_PEAK_NB
        }
        port.write_u8(RX_TOP + 35, 0xFF)?;
        port.write_u8(RX_TOP + 36, 0xFF)?;
        port.write_u8(RX_TOP + 32, 0xFF)?;
        port.write_u8(RX_TOP + 33, 0xFF)?;
        port.write_u8(RX_TOP + 34, 0xFF)?;
        Ok(())
    }

    fn configure_radio_fe(&self, port: &ConcentratorPort<'_>) -> Result<()> {
        port.write_u8(RADIO_FE + 7, 0x03)?;
        port.write_u8(RADIO_FE + 8, 0x07)?;
        port.write_u8(RADIO_FE + 20, 0x03)?;
        port.write_u8(RADIO_FE + 21, 0x07)?;
        port.write_u8(RADIO_FE + 2, 23)?;
        port.write_u8(RADIO_FE + 3, 66)?;
        port.write_u8(RADIO_FE + 15, 23)?;
        port.write_u8(RADIO_FE + 16, 66)?;
        port.rmw(RADIO_FE + 1, 5, 1, 1)?;
        port.rmw(RADIO_FE + 1, 0, 4, 0x0b)?;
        port.rmw(RADIO_FE + 14, 5, 1, 1)?;
        port.rmw(RADIO_FE + 14, 0, 4, 0x0b)?;
        Ok(())
    }

    fn configure_lora_modem(&self, port: &ConcentratorPort<'_>, radio_hz: u32) -> Result<()> {
        port.rmw(RX_TOP + 96, 0, 1, 0)?; // DC_NOTCH off
        port.rmw(RX_TOP + 117, 3, 1, 1)?; // FORCE_DEFAULT_FIR
        port.rmw(RX_TOP + 120, 4, 1, 1)?; // GAIN_DROP_COMP
        port.rmw(RX_TOP + 120, 6, 2, 1)?; // TARGET_LVL
        port.write_u8(OTP_MODEM_EN_0, 0xFF)?;
        port.write_u8(OTP_MODEM_EN_1, 0xFF)?;
        port.rmw(RX_TOP + 183, 0, 3, 0)?;
        port.write_u8(RX_TOP + 184, 126)?;
        port.write_u8(ARB + 29, (5 << 4) | 1)?;
        port.write_u8(ARB + 30, (13 << 4) | 9)?;
        port.write_u8(ARB + 31, (5 << 4) | 1)?;
        port.write_u8(ARB + 32, (13 << 4) | 9)?;
        port.write_u8(RX_TOP + 131, 10)?;
        port.write_u8(RX_TOP + 132, 0)?;
        let (mant, exp) = freq_to_time_drift(125_000, radio_hz);
        port.rmw(RX_TOP + 160, 0, 4, ((mant >> 8) & 0x0F) as u8)?;
        port.write_u8(RX_TOP + 161, (mant & 0xFF) as u8)?;
        port.rmw(RX_TOP + 162, 0, 3, exp)?;
        port.rmw(RX_TOP + 163, 2, 1, 1)?; // invert_time_symb
        port.rmw(RX_TOP + 126, 3, 1, 1)?; // TXRX_CFG1 MODEM_EN
        Ok(())
    }

    fn agc_start(&self, port: &ConcentratorPort<'_>) -> Result<()> {
        let p = &AGC_SX1250;
        wait_status(port, 0x01)?;
        let ver = mb_rd(port, 0)?;
        if ver != FW_VERSION_AGC_SX1250 {
            return Err(HalError::Other(format!(
                "AGC fw version {ver} expected {FW_VERSION_AGC_SX1250}"
            )));
        }
        mb_wr(port, 0, 0)?; // auto ana
        mb_wr(port, 1, 0)?; // auto dec
        mb_wr(port, 3, AGC_RADIO_A_INIT_DONE)?;
        wait_status(port, 0x02)?;
        mb_wr(port, 0, 0)?;
        mb_wr(port, 1, 0)?;
        mb_wr(port, 3, AGC_RADIO_B_INIT_DONE)?;
        wait_status(port, 0x03)?;

        let steps: [(&[u8], u8, u8); 6] = [
            (&[p.ana_min, p.ana_max], 0x03, 0x04),
            (&[p.ana_thresh_l, p.ana_thresh_h], 0x04, 0x05),
            (&[p.dec_attn_min, p.dec_attn_max], 0x05, 0x06),
            (&[p.dec_thresh_l, p.dec_thresh_h1, p.dec_thresh_h2], 0x06, 0x07),
            (&[p.chan_attn_min, p.chan_attn_max], 0x07, 0x08),
            (&[p.chan_thresh_l, p.chan_thresh_h], 0x08, 0x09),
        ];
        for (vals, notify, next) in steps {
            for (i, v) in vals.iter().enumerate() {
                mb_wr(port, i as u8, *v)?;
            }
            mb_wr(port, 3, notify)?;
            wait_status(port, next)?;
        }
        mb_wr(port, 0, p.device_sel)?;
        mb_wr(port, 1, p.hp_max)?;
        mb_wr(port, 2, p.pa_duty)?;
        mb_wr(port, 3, 0x09)?;
        wait_status(port, 0x0A)?;
        mb_wr(port, 0, 8)?;
        mb_wr(port, 3, 0x0A)?;
        wait_status(port, 0x0B)?;
        mb_wr(port, 0, 0)?; // LBT off
        mb_wr(port, 3, 0x0B)?;
        wait_status(port, 0x0F)?;
        mb_wr(port, 3, 0x0F)?;
        Ok(())
    }

    fn wait_arb(port: &ConcentratorPort<'_>, want: u8) -> Result<()> {
        let t0 = Instant::now();
        loop {
            let st = port.read_u8(ARB_MCU_STATUS)?;
            if st == want {
                return Ok(());
            }
            if t0.elapsed() > Duration::from_secs(3) {
                return Err(HalError::Other(format!(
                    "ARB status timeout (got 0x{st:02x} want 0x{want:02x})"
                )));
            }
            std::thread::sleep(Duration::from_millis(2));
        }
    }

    fn arb_start(&self, port: &ConcentratorPort<'_>) -> Result<()> {
        Self::wait_arb(port, 0x01)?;
        let ver = port.read_u8(ARB + 13)?; // DEBUG_STS_0 = version
        if ver != FW_VERSION_ARB {
            return Err(HalError::Other(format!(
                "ARB fw version {ver} expected {FW_VERSION_ARB}"
            )));
        }
        port.write_u8(ARB + 9, 7)?; // stats SF7
        port.write_u8(ARB + 12, 0x00)?; // dual demod off
        port.write_u8(ARB + 11, 3)?; // double-detect threshold
        port.write_u8(ARB + 10, 1)?; // resume
        Self::wait_arb(port, 0x00)?;
        Ok(())
    }

    /// `sx1302_config_gpio`: SX1302 GPIOs drive the WM1302 HAT LEDs.
    /// GPIO0 = CONFIG_DONE (register), GPIO2 = TX ON, GPIO4 = RX toggle (status),
    /// GPIO7 = AGC (LBT input), others register-driven; direction 0x7F.
    fn config_gpio(port: &ConcentratorPort<'_>) -> Result<()> {
        const REGISTER: u8 = 0x00;
        const AGC: u8 = 0x01;
        const STATUS: u8 = 0x09;
        let sel = [REGISTER, REGISTER, STATUS, REGISTER, STATUS, REGISTER, REGISTER, AGC];
        for (i, s) in sel.iter().enumerate() {
            port.write_u8(GPIO_SEL_0 + i as u16, *s)?;
        }
        port.write_u8(GPIO_DIR_L, 0x7F)?;
        Ok(())
    }

    /// Drive the HAT "Config" LED (SX1302 GPIO0).
    pub fn set_config_led(&self, on: bool) -> Result<()> {
        self.bus.sx1302().rmw(GPIO_OUT_L, 0, 1, u8::from(on))
    }

    /// COMMON_GEN: GLOBAL_EN | CONCENTRATOR | FSK (if planned) | MBWSSF (if planned).
    fn common_gen(plan: &ChannelPlan) -> u8 {
        0x08 | 0x02 | if plan.fsk.is_some() { 0x04 } else { 0 } | u8::from(plan.lora_std.is_some())
    }

    fn configure_channelizer(&self, port: &ConcentratorPort<'_>, plan: &ChannelPlan) -> Result<()> {
        let mut radio_select = 0u8;
        for i in 0..8usize {
            // unused chains: IF 0 on radio A (correlators stay off via OTP/ACC config)
            let (radio, if_hz) = match plan.multisf.get(i) {
                Some(c) => (c.radio, plan.if_hz(c.radio, c.freq_hz)),
                None => (0, 0),
            };
            if radio == 1 {
                radio_select |= 1 << i;
            }
            let reg = if_hz_to_reg(if_hz);
            let base = RX_TOP + (i as u16) * 2;
            port.rmw(base, 0, 5, ((reg >> 8) & 0x1F) as u8)?;
            port.write_u8(base + 1, (reg & 0xFF) as u8)?;
        }
        port.write_u8(RX_TOP + 16, radio_select)?; // RADIO_SELECT (bit = radio B)
        // Multi-SF correlators only for planned chains.
        let en = if plan.multisf.len() >= 8 { 0xFF } else { (1u8 << plan.multisf.len()) - 1 };
        port.write_u8(RX_TOP + 35, en)?;
        port.rmw(RX_TOP + 17, 3, 5, 0x05)?; // RSSI filter
        port.write_u8(RX_TOP + 18, 85)?;
        // Fixed channelizer gain (Semtech fix_gain) so correlator is not waiting on AGC.
        port.write_u8(RX_TOP + 23, 0x00)?;
        port.rmw(RX_TOP + 25, 0, 4, 5)?; // CHAN_GAIN = 5
        Ok(())
    }

    /// `sx1302_lora_service_correlator_configure` + `sx1302_lora_service_modem_configure`.
    fn configure_lora_service(
        &self,
        port: &ConcentratorPort<'_>,
        plan: &ChannelPlan,
        s: &crate::plan::LoraStdChan,
    ) -> Result<()> {
        let bw = bw_hz_to_code(s.bw_hz).unwrap_or(BW_125KHZ);
        let if_reg = if_hz_to_reg(plan.if_hz(s.radio, s.freq_hz));
        port.rmw(LSV, 0, 5, ((if_reg >> 8) & 0x1F) as u8)?;
        port.write_u8(LSV + 1, (if_reg & 0xFF) as u8)?;
        port.rmw(LSV + 2, 0, 1, s.radio & 1)?; // RADIO_SELECT
        port.rmw(LSV + 3, 0, 1, 0)?; // DC_NOTCH off
        port.rmw(LSV + 24, 3, 1, 1)?; // FORCE_DEFAULT_FIR
        port.rmw(LSV + 27, 4, 1, 1)?; // GAIN_DROP_COMP
        port.rmw(LSV + 27, 6, 2, 1)?; // TARGET_LVL
        port.rmw(LSV + 50, 6, 2, 3)?; // GAIN_P_AUTO
        port.rmw(LSV + 51, 3, 3, 3)?; // GAIN_I_PAYLOAD
        let (gain_p_preamb, gain_i_en) = match s.sf {
            5 | 6 => (4, 0),
            7..=10 => (6, 0),
            _ => (
                7,
                match bw {
                    0x05 => 2,
                    0x06 => 3,
                    _ => 1,
                },
            ),
        };
        port.rmw(LSV + 50, 0, 3, gain_p_preamb)?; // GAIN_P_PREAMB
        port.rmw(LSV + 51, 6, 2, gain_i_en)?; // GAIN_I_EN
        port.rmw(LSV + 33, 0, 4, s.sf)?; // MODEM_SF
        port.rmw(LSV + 33, 4, 4, bw)?; // MODEM_BW
        let ppm = u8::from(
            (bw == BW_125KHZ && s.sf >= 11) || (bw == crate::plan::BW_250KHZ && s.sf == 12),
        );
        port.rmw(LSV + 34, 4, 2, ppm)?; // PPM_OFFSET
        let preamble: u16 = if s.sf <= 6 { 12 } else { 8 };
        port.write_u8(LSV + 39, (preamble & 0xFF) as u8)?;
        port.write_u8(LSV + 40, (preamble >> 8) as u8)?;
        let (mant, exp) = freq_to_time_drift(s.bw_hz, plan.radio_hz[usize::from(s.radio & 1)]);
        port.rmw(LSV + 54, 0, 4, ((mant >> 8) & 0x0F) as u8)?;
        port.write_u8(LSV + 55, (mant & 0xFF) as u8)?;
        port.rmw(LSV + 56, 0, 3, exp)?;
        port.rmw(LSV + 57, 3, 1, 1)?; // invert_time_symb
        port.rmw(LSV + 25, 6, 1, 1)?; // DAGC_IN_COMP
        port.rmw(LSV + 34, 3, 1, 1)?; // MODEM_EN
        port.rmw(LSV + 35, 2, 2, 1)?; // CADRXTX RX
        port.rmw(LSV + 35, 4, 1, 1)?; // MODEM_START
        port.rmw(LSV + 35, 5, 1, u8::from(s.sf <= 6))?; // FINE_SYNCH
        port.rmw(LSV + 64, 0, 3, 7)?; // MSP_PEAK_NB
        port.rmw(LSV + 64, 4, 3, 5)?; // MSP2_PEAK_NB
        port.rmw(LSV + 66, 7, 1, 1)?; // USE_GAIN_SYMB
        port.rmw(LSV + 66, 0, 7, 52)?; // ACC_PNR
        port.rmw(LSV + 46, 0, 5, 6)?; // public 0x34
        port.rmw(LSV + 47, 0, 5, 8)?;
        port.rmw(LSV + 42, 4, 2, 1)?; // DFT_PEAK_EN = AUTO
        Ok(())
    }

    /// `sx1302_fsk_configure`: 125 kHz channel, variable length, CCITT CRC, whitening.
    fn configure_fsk(
        &self,
        port: &ConcentratorPort<'_>,
        plan: &ChannelPlan,
        f: &crate::plan::FskChan,
    ) -> Result<()> {
        let if_reg = if_hz_to_reg(plan.if_hz(f.radio, f.freq_hz));
        port.rmw(LSV + 80, 0, 5, ((if_reg >> 8) & 0x1F) as u8)?;
        port.write_u8(LSV + 81, (if_reg & 0xFF) as u8)?;
        port.rmw(LSV + 84, 1, 1, f.radio & 1)?; // RADIO_SELECT
        port.rmw(LSV + 83, 3, 3, f.sync_word_size - 1)?; // PSIZE
        let sync = f.sync_word << (8 * (8 - u32::from(f.sync_word_size)));
        for i in 0..8u16 {
            port.write_u8(LSV + 100 - i, ((sync >> (8 * i)) & 0xFF) as u8)?;
        }
        let br = 32_000_000 / f.datarate;
        port.write_u8(LSV + 91, (br >> 8) as u8)?;
        port.write_u8(LSV + 92, br as u8)?;
        port.rmw(LSV + 83, 0, 3, 3)?; // CH_BW_EXPO 125 kHz
        port.rmw(LSV + 84, 0, 1, 0)?; // RX_INVERT
        port.rmw(LSV + 84, 3, 1, 0)?; // MODEM_INVERT_IQ
        port.rmw(LSV + 85, 5, 3, 4)?; // RSSI_LENGTH
        port.rmw(LSV + 82, 0, 1, 1)?; // PKT_MODE variable
        port.rmw(LSV + 82, 1, 1, 1)?; // CRC_EN
        port.rmw(LSV + 82, 2, 2, 2)?; // DCFREE whitening
        port.rmw(LSV + 82, 4, 1, 0)?; // CRC IBM off (CCITT)
        port.rmw(LSV + 85, 0, 5, 10)?; // ERROR_OSR_TOL
        port.write_u8(LSV + 88, 255)?; // PKT_LENGTH
        port.write_u8(LSV + 86, 0)?; // NODE_ADRS
        port.write_u8(LSV + 87, 0)?; // BROADCAST
        port.rmw(LSV + 84, 2, 1, 1)?; // AUTO_AFC
        port.rmw(LSV + 89, 0, 2, 0)?;
        port.write_u8(LSV + 90, 128)?;
        Ok(())
    }
}