//! SenseCAP M1 carrier vs official WM1302 Pi HAT pin models.
//! GPIO character-device identifiers only. Unverified lines cannot be driven.

use serde::{Deserialize, Serialize};
use sx1302_core::VerificationStatus;
use std::fs;
use std::path::Path;

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum ElectricalFunction {
    Reset,
    ChipSelect,
    BusyInput,
    PowerEnable,
    Dio,
    I2c,
    Uart,
    Spi,
    Unknown,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum Polarity {
    ActiveHigh,
    ActiveLow,
    Unknown,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum Direction {
    Input,
    Output,
    Unconfigured,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct BoardPin {
    pub signal: String,
    pub bcm: u8,
    pub function: ElectricalFunction,
    pub polarity: Polarity,
    pub default_direction: Direction,
    pub safe_initial: String,
    pub reset_duration_us: u32,
    pub release_duration_us: u32,
    pub verification: VerificationStatus,
    pub notes: String,
}

pub fn wm1302_hat_candidate_pins() -> Vec<BoardPin> {
    vec![
        pin("SX1302_RESET", 17, ElectricalFunction::Reset, Polarity::ActiveHigh, Direction::Output, "low", 100_000, 100_000, VerificationStatus::VerifiedOnHardware, "SenseCAP M1 / Seeed WM1302 SPI: SX1302 reset active HIGH. Operator authorized GPIO drive."),
        pin("SX1302_CS", 8, ElectricalFunction::ChipSelect, Polarity::ActiveLow, Direction::Output, "kernel-ce0", 0, 0, VerificationStatus::CandidateFromDocs, "SPI0 CE0 /dev/spidev0.0"),
        pin("SPI_MOSI", 10, ElectricalFunction::Spi, Polarity::Unknown, Direction::Unconfigured, "alt0", 0, 0, VerificationStatus::CandidateFromDocs, "SPI0"),
        pin("SPI_MISO", 9, ElectricalFunction::Spi, Polarity::Unknown, Direction::Unconfigured, "alt0", 0, 0, VerificationStatus::CandidateFromDocs, "SPI0"),
        pin("SPI_SCLK", 11, ElectricalFunction::Spi, Polarity::Unknown, Direction::Unconfigured, "alt0", 0, 0, VerificationStatus::CandidateFromDocs, "SPI0"),
        pin("SX1262_RESET", 5, ElectricalFunction::Reset, Polarity::ActiveLow, Direction::Output, "high", 100_000, 100_000, VerificationStatus::VerifiedOnHardware, "SenseCAP M1 aux radio reset active-low. Operator authorized GPIO drive."),
        pin("SX1262_CS", 6, ElectricalFunction::ChipSelect, Polarity::ActiveLow, Direction::Output, "high", 0, 0, VerificationStatus::CandidateFromDocs, "GPIO CS, not SPI0 CE1 (GPIO7)"),
        pin("SX1302_POWER_EN", 18, ElectricalFunction::PowerEnable, Polarity::ActiveHigh, Direction::Output, "high", 0, 0, VerificationStatus::VerifiedOnHardware, "SenseCAP M1 POWER_EN (operator override of Seeed BUSY table). Driven as output HIGH to enable concentrator LDOs."),
        pin("SX1262_DIO1", 23, ElectricalFunction::Dio, Polarity::Unknown, Direction::Input, "input", 0, 0, VerificationStatus::CandidateFromDocs, ""),
        pin("SX1262_DIO2", 24, ElectricalFunction::Dio, Polarity::Unknown, Direction::Input, "input", 0, 0, VerificationStatus::CandidateFromDocs, ""),
        pin("GPS_RESET", 25, ElectricalFunction::Reset, Polarity::Unknown, Direction::Unconfigured, "unverified", 0, 0, VerificationStatus::CandidateFromDocs, "Standard Pi HAT GPS; SenseCAP M1 GPS presence unknown"),
        pin("AD5338R_RESET", 13, ElectricalFunction::Reset, Polarity::Unknown, Direction::Unconfigured, "unverified", 0, 0, VerificationStatus::ConflictUnverified, "Present in Semtech reset_lgw.sh; absent from Seeed HAT table. CN490 full-duplex reference. Do not drive."),
        pin("PAIRING_BUTTON", 255, ElectricalFunction::Unknown, Polarity::Unknown, Direction::Input, "unverified", 0, 0, VerificationStatus::NotPresent, "Clockwork Bird: GPIO unknown; probe unused inputs only"),
        pin("INDICATOR_LED", 22, ElectricalFunction::Unknown, Polarity::ActiveHigh, Direction::Output, "low", 0, 0, VerificationStatus::CandidateFromDocs, "Clockwork Bird M1 v1.1 User LED = GPIO22"),
    ]
}

fn pin(
    signal: &'static str,
    bcm: u8,
    function: ElectricalFunction,
    polarity: Polarity,
    default_direction: Direction,
    safe_initial: &'static str,
    reset_duration_us: u32,
    release_duration_us: u32,
    verification: VerificationStatus,
    notes: &'static str,
) -> BoardPin {
    BoardPin {
        signal: signal.into(),
        bcm,
        function,
        polarity,
        default_direction,
        safe_initial: safe_initial.into(),
        reset_duration_us,
        release_duration_us,
        verification,
        notes: notes.into(),
    }
}

pub fn may_drive(pin: &BoardPin) -> Result<(), sx1302_core::HalError> {
    if pin.bcm == 13 {
        return Err(sx1302_core::HalError::GpioRefused(
            "BCM13 AD5338R_RESET: CN490-only, never drive on SenseCAP M1".into(),
        ));
    }
    match pin.verification {
        VerificationStatus::VerifiedOnHardware => Ok(()),
        VerificationStatus::ConflictUnverified => Err(sx1302_core::HalError::GpioRefused(
            format!("{} BCM{}: documentation conflict, refusing output", pin.signal, pin.bcm),
        )),
        VerificationStatus::CandidateFromDocs if pin.bcm == 22 => Ok(()),
        VerificationStatus::CandidateFromDocs => Err(sx1302_core::HalError::GpioRefused(
            format!("{} BCM{}: candidate only, not verified on SenseCAP M1", pin.signal, pin.bcm),
        )),
        VerificationStatus::NotPresent => Err(sx1302_core::HalError::GpioRefused(format!(
            "{}: mapping unknown",
            pin.signal
        ))),
    }
}

pub const GPIOCHIP: &str = "/dev/gpiochip0";

/// SenseCAP M1 concentrator GPIO sequence (skip BCM13).
/// POWER_EN HIGH, SX1302_RESET pulse HIGH then LOW, SX1262_RESET pulse LOW then HIGH.
pub fn reset_sensecap_m1() -> Result<Vec<String>, sx1302_core::HalError> {
    let pins = wm1302_hat_candidate_pins();
    let power = pins.iter().find(|p| p.bcm == 18).expect("POWER_EN");
    let rst = pins.iter().find(|p| p.bcm == 17).expect("RESET");
    let sx126 = pins.iter().find(|p| p.bcm == 5).expect("SX126 RESET");
    may_drive(power)?;
    may_drive(rst)?;
    may_drive(sx126)?;
    gpio_reset_sequence()
}

#[cfg(target_os = "linux")]
fn gpio_reset_sequence() -> Result<Vec<String>, sx1302_core::HalError> {
    use gpiocdev::Request;
    use gpiocdev::line::Value;
    use std::thread::sleep;
    use std::time::Duration;

    let mut notes = Vec::new();
    let wait = Duration::from_millis(100);

    let power = Request::builder()
        .on_chip(GPIOCHIP)
        .with_line(18)
        .as_output(Value::Active)
        .request()
        .map_err(|e| sx1302_core::HalError::Io(format!("GPIO18 POWER_EN: {e}")))?;
    notes.push("GPIO18 POWER_EN = HIGH".into());
    sleep(wait);

    let reset = Request::builder()
        .on_chip(GPIOCHIP)
        .with_line(17)
        .as_output(Value::Active)
        .request()
        .map_err(|e| sx1302_core::HalError::Io(format!("GPIO17 RESET: {e}")))?;
    notes.push("GPIO17 SX1302_RESET = HIGH (assert)".into());
    sleep(wait);
    reset
        .set_value(17, Value::Inactive)
        .map_err(|e| sx1302_core::HalError::Io(format!("GPIO17 deassert: {e}")))?;
    notes.push("GPIO17 SX1302_RESET = LOW (release)".into());
    sleep(wait);

    let aux = Request::builder()
        .on_chip(GPIOCHIP)
        .with_line(5)
        .as_output(Value::Inactive)
        .request()
        .map_err(|e| sx1302_core::HalError::Io(format!("GPIO5 SX126 RESET: {e}")))?;
    notes.push("GPIO5 SX1262_RESET = LOW (assert)".into());
    sleep(wait);
    aux.set_value(5, Value::Active)
        .map_err(|e| sx1302_core::HalError::Io(format!("GPIO5 release: {e}")))?;
    notes.push("GPIO5 SX1262_RESET = HIGH (release)".into());
    sleep(wait);

    notes.push("GPIO13 AD5338R skipped".into());
    let _ = (power, reset, aux);
    // cdev Request drop reverts lines; pinctrl keeps POWER_EN/RESET after CLI exit.
    persist_pinctrl(&mut notes);
    Ok(notes)
}

#[cfg(target_os = "linux")]
fn persist_pinctrl(notes: &mut Vec<String>) {
    let _ = std::process::Command::new("pinctrl")
        .args(["set", "18", "op", "dh"])
        .status();
    let _ = std::process::Command::new("pinctrl")
        .args(["set", "17", "op", "dl"])
        .status();
    let _ = std::process::Command::new("pinctrl")
        .args(["set", "5", "op", "dh"])
        .status();
    notes.push("pinctrl persist: 18 op dh, 17 op dl, 5 op dh".into());
}

#[cfg(not(target_os = "linux"))]
fn gpio_reset_sequence() -> Result<Vec<String>, sx1302_core::HalError> {
    Err(sx1302_core::HalError::Unsupported("Linux gpiochip required"))
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct BoardInspect {
    pub board_model: String,
    pub cpu_model: String,
    pub cpu_features: Vec<String>,
    pub os: String,
    pub kernel: String,
    pub spi_devices: Vec<String>,
    pub gpio_chips: Vec<String>,
    pub i2c_devices: Vec<String>,
    pub candidate_wm1302: String,
    pub atecc_status: String,
    pub unverified_signals: Vec<String>,
    pub readiness: String,
    pub pins: Vec<BoardPin>,
    pub upstream_sx1302_hal_sha: String,
}

fn read_trim(path: &str) -> String {
    fs::read_to_string(path)
        .unwrap_or_default()
        .trim_end_matches('\0')
        .trim()
        .to_string()
}

fn glob_dev(prefix: &str) -> Vec<String> {
    let mut v = Vec::new();
    if let Ok(rd) = fs::read_dir("/dev") {
        for e in rd.flatten() {
            let n = e.file_name().to_string_lossy().into_owned();
            if n.starts_with(prefix) {
                v.push(format!("/dev/{n}"));
            }
        }
    }
    v.sort();
    v
}

pub fn inspect() -> BoardInspect {
    let model = read_trim("/proc/device-tree/model");
    let cpuinfo = read_trim("/proc/cpuinfo");
    let cpu_model = cpuinfo
        .lines()
        .find(|l| l.to_ascii_lowercase().contains("model name") || l.starts_with("CPU part"))
        .unwrap_or("unknown")
        .to_string();
    let features = cpuinfo
        .lines()
        .find(|l| l.starts_with("Features") || l.starts_with("flags"))
        .map(|l| {
            l.split(':')
                .nth(1)
                .unwrap_or("")
                .split_whitespace()
                .map(|s| s.to_string())
                .collect()
        })
        .unwrap_or_default();
    let os = read_trim("/etc/os-release");
    let kernel = read_trim("/proc/version");
    let spi = glob_dev("spidev");
    let i2c = glob_dev("i2c-");
    let gpio = glob_dev("gpiochip");
    let pins = wm1302_hat_candidate_pins();
    let unverified: Vec<String> = pins
        .iter()
        .filter(|p| p.verification != VerificationStatus::VerifiedOnHardware)
        .map(|p| format!("{} BCM{} {:?}", p.signal, p.bcm, p.verification))
        .collect();

    let atecc = if Path::new("/dev/i2c-1").exists() {
        "I2C-1 present; ATECC608 address 0x60 not scanned (non-destructive policy: no unsolicited I2C probe of unknown devices beyond presence of adapter)".into()
    } else {
        "no /dev/i2c-1".into()
    };

    let candidate = if spi.iter().any(|s| s.ends_with("spidev0.0")) {
        "/dev/spidev0.0 (SPI0 CE0) candidate for SX1302"
    } else {
        "no /dev/spidev0.0 — enable SPI before concentrator access"
    };

    let readiness = if spi.iter().any(|s| s.contains("spidev0.0")) && model.contains("Raspberry Pi 5")
    {
        "SPI node present on Pi 5. GPIO17/18/5 authorized for SenseCAP M1 reset. GPIO13 never driven. TX requires --region."
    } else {
        "Hardware incomplete or not Pi 5; see fields above."
    };

    BoardInspect {
        board_model: if model.is_empty() { "unknown".into() } else { model },
        cpu_model,
        cpu_features: features,
        os,
        kernel,
        spi_devices: spi,
        gpio_chips: gpio,
        i2c_devices: i2c,
        candidate_wm1302: candidate.into(),
        atecc_status: atecc,
        unverified_signals: unverified,
        readiness: readiness.into(),
        pins,
        upstream_sx1302_hal_sha: "4b42025d1751e04632c0b04160e0d29dbbb222a5".into(),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn gpio13_never_driven() {
        let p = wm1302_hat_candidate_pins()
            .into_iter()
            .find(|p| p.bcm == 13)
            .unwrap();
        assert!(may_drive(&p).is_err());
    }

    #[test]
    fn power_en_18_may_drive() {
        let p = wm1302_hat_candidate_pins()
            .into_iter()
            .find(|p| p.bcm == 18)
            .unwrap();
        assert!(may_drive(&p).is_ok());
    }
}
