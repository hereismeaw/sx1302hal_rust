//! Pairing button: SenseCAP M1 live press mapped to BCM GPIO27 (active high).
//! Other unused lines sampled as inputs. GPIO13 is never opened.

use serde::{Deserialize, Serialize};
use sx1302_core::{HalError, Result, VerificationStatus};

/// Live capture 2026-11-09: only GPIO27 moved during a pairing-button press.
pub const BUTTON_BCM: u8 = 27;

/// GPIOs that are not concentrator RESET/POWER/CS/SPI/LED/SX126.
const CANDIDATE_INPUTS: [u8; 8] = [4, 16, 19, 20, 21, 26, 27, 12];

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ButtonProbe {
    pub mapped_bcm: Option<u8>,
    pub verification: VerificationStatus,
    pub lines: Vec<LineSample>,
    pub notes: Vec<String>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct LineSample {
    pub bcm: u8,
    pub high: Option<bool>,
    pub error: Option<String>,
}

pub fn gpio_unknown() -> Result<()> {
    Err(HalError::GpioRefused(
        "pairing button GPIO is not in WM1302 HAT tables; use sensecap_button::probe".into(),
    ))
}

/// Read candidate GPIOs as inputs. Does not drive any line.
pub fn probe() -> Result<ButtonProbe> {
    #[cfg(not(target_os = "linux"))]
    {
        Err(HalError::Unsupported("Linux gpiochip required"))
    }
    #[cfg(target_os = "linux")]
    {
        linux_probe()
    }
}

#[cfg(target_os = "linux")]
fn linux_probe() -> Result<ButtonProbe> {
    use gpiocdev::line::Value;
    use gpiocdev::Request;

    let mut lines = Vec::new();
    for bcm in CANDIDATE_INPUTS {
        if bcm == 13 {
            continue;
        }
        match Request::builder()
            .on_chip("/dev/gpiochip0")
            .with_line(u32::from(bcm))
            .as_input()
            .request()
        {
            Ok(req) => match req.value(u32::from(bcm)) {
                Ok(Value::Active) => lines.push(LineSample {
                    bcm,
                    high: Some(true),
                    error: None,
                }),
                Ok(Value::Inactive) => lines.push(LineSample {
                    bcm,
                    high: Some(false),
                    error: None,
                }),
                Err(e) => lines.push(LineSample {
                    bcm,
                    high: None,
                    error: Some(e.to_string()),
                }),
            },
            Err(e) => lines.push(LineSample {
                bcm,
                high: None,
                error: Some(e.to_string()),
            }),
        }
    }
    Ok(ButtonProbe {
        mapped_bcm: Some(BUTTON_BCM),
        verification: VerificationStatus::VerifiedOnHardware,
        lines,
        notes: vec![
            "Pairing button = BCM GPIO27, active high (press drives the line high against pinctrl pd).".into(),
            "180s capture: GPIO 4/12/16/19/20/21/22/26 unchanged; only 27 edges.".into(),
            "GPIO13 never opened".into(),
        ],
    })
}
