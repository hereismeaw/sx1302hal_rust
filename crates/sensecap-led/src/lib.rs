//! SenseCAP M1 user LED: Clockwork Bird header annotation BCM GPIO22.
//! Polarity: drive HIGH = on (community map; verified by CLI blink).

use serde::{Deserialize, Serialize};
use sx1302_core::{HalError, Result, VerificationStatus};

pub const LED_BCM: u8 = 22;

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct LedReport {
    pub bcm: u8,
    pub on: bool,
    pub verification: VerificationStatus,
    pub notes: Vec<String>,
}

pub fn gpio_unknown() -> Result<()> {
    Err(HalError::GpioRefused(
        "use sensecap_led::set; GPIO22 is the Clockwork Bird M1 LED candidate".into(),
    ))
}

pub fn set(on: bool) -> Result<LedReport> {
    #[cfg(not(target_os = "linux"))]
    {
        let _ = on;
        Err(HalError::Unsupported("Linux gpiochip required"))
    }
    #[cfg(target_os = "linux")]
    {
        linux_set(on)
    }
}

#[cfg(target_os = "linux")]
fn linux_set(on: bool) -> Result<LedReport> {
    use gpiocdev::line::Value;
    use gpiocdev::Request;
    use std::process::Command;

    let val = if on { Value::Active } else { Value::Inactive };
    let req = Request::builder()
        .on_chip("/dev/gpiochip0")
        .with_line(u32::from(LED_BCM))
        .as_output(val)
        .request()
        .map_err(|e| HalError::Io(format!("GPIO22 LED: {e}")))?;
    let _ = req;
    let level = if on { "dh" } else { "dl" };
    let _ = Command::new("pinctrl")
        .args(["set", "22", "op", level])
        .status();
    Ok(LedReport {
        bcm: LED_BCM,
        on,
        verification: VerificationStatus::CandidateFromDocs,
        notes: vec![
            "Clockwork Bird M1 v1.1 header: User LED = GPIO22".into(),
            "GPIO13 never driven".into(),
            format!("pinctrl persist 22 op {level}"),
        ],
    })
}
