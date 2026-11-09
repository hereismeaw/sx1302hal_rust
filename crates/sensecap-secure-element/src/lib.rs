//! ATECC608 on the SenseCAP M1 carrier. Read-only inspection only.
//! Never lock zones, never overwrite Helium slots.

use serde::{Deserialize, Serialize};
use sx1302_core::{HalError, Result};

pub const CANDIDATE_I2C_ADDR: u8 = 0x60;

pub fn destructive_ops_blocked() -> Result<()> {
    Err(HalError::Other(
        "ATECC608 personalize/lock/write commands are disabled".into(),
    ))
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct AteccInspect {
    pub adapter: String,
    pub addr: u8,
    pub present: bool,
    pub serial_hex: Option<String>,
    pub notes: Vec<String>,
}

/// Wake + Info/Serial read. No GenKey, no Write, no Lock.
pub fn inspect_readonly() -> Result<AteccInspect> {
    #[cfg(not(target_os = "linux"))]
    {
        return Err(HalError::Unsupported("Linux I2C required"));
    }
    #[cfg(target_os = "linux")]
    {
        linux_inspect()
    }
}

#[cfg(target_os = "linux")]
fn linux_inspect() -> Result<AteccInspect> {
    use std::fs::OpenOptions;
    use std::io::{Read, Write};
    use std::os::unix::io::AsRawFd;
    use std::thread::sleep;
    use std::time::Duration;

    const I2C_SLAVE: u64 = 0x0703;
    let mut notes = vec!["read-only: no Lock/Write/GenKey".into()];
    let mut f = OpenOptions::new()
        .read(true)
        .write(true)
        .open("/dev/i2c-1")
        .map_err(|e| HalError::Io(format!("/dev/i2c-1: {e}")))?;
    let fd = f.as_raw_fd();
    let addr = i32::from(CANDIDATE_I2C_ADDR);
    let rc = unsafe { libc::ioctl(fd, I2C_SLAVE, addr) };
    if rc < 0 {
        return Err(HalError::Io("I2C_SLAVE 0x60 failed".into()));
    }
    // Wake pulse: 0x00 to address 0 then delay (ATCA wakeup).
    let _ = f.write_all(&[0x00]);
    sleep(Duration::from_millis(2));
    // Info command word: count, opcode 0x30, param1 0x00, crc — simplified probe.
    let cmd = [0x03, 0x30, 0x00];
    let wrote = f.write(&cmd);
    sleep(Duration::from_millis(5));
    let mut buf = [0u8; 9];
    let n = f.read(&mut buf).unwrap_or(0);
    let present = wrote.is_ok() && n > 0;
    let serial = if present && n >= 8 {
        Some(buf[..n.min(9)].iter().map(|b| format!("{b:02x}")).collect())
    } else {
        None
    };
    if !present {
        notes.push("no ATECC response on 0x60 (chip absent or sleep)".into());
    }
    Ok(AteccInspect {
        adapter: "/dev/i2c-1".into(),
        addr: CANDIDATE_I2C_ADDR,
        present,
        serial_hex: serial,
        notes,
    })
}