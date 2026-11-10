//! GPS UART. Absent until `gps` is set. 9600 8N1, the L76 / u-blox default.
//! The PPS wire is the SX1302 pin, not this UART.

use std::io::Read;
use std::fs::OpenOptions;
use sx1302_hal::{parse_rmc, Rmc};

pub struct GpsPort {
    file: std::fs::File,
    buf: Vec<u8>,
    warned: bool,
}

impl GpsPort {
    pub fn open(path: &str) -> std::io::Result<Self> {
        let mut file = OpenOptions::new().read(true).write(true).open(path)?;
        set_9600(&mut file)?;
        Ok(Self { file, buf: Vec::new(), warned: false })
    }

    pub fn poll(&mut self) -> Vec<Rmc> {
        let mut tmp = [0u8; 256];
        match self.file.read(&mut tmp) {
            Ok(0) => return Vec::new(),
            Err(e) if e.kind() == std::io::ErrorKind::WouldBlock => return Vec::new(),
            Err(e) => {
                if !self.warned {
                    eprintln!("gps read: {e}");
                    self.warned = true;
                }
                return Vec::new();
            }
            Ok(n) => self.buf.extend_from_slice(&tmp[..n]),
        }
        let mut out = Vec::new();
        while let Some(i) = self.buf.iter().position(|b| *b == b'\n') {
            let line: Vec<u8> = self.buf.drain(..=i).collect();
            if let Ok(s) = std::str::from_utf8(&line) {
                if let Some(rmc) = parse_rmc(s) {
                    out.push(rmc);
                }
            }
        }
        if self.buf.len() > 1024 {
            self.buf.clear();
        }
        out
    }
}

#[cfg(unix)]
fn set_9600(file: &mut std::fs::File) -> std::io::Result<()> {
    use std::os::unix::io::AsRawFd;
    let fd = file.as_raw_fd();
    unsafe {
        let mut t: libc::termios = std::mem::zeroed();
        if libc::tcgetattr(fd, &mut t) != 0 {
            return Err(std::io::Error::last_os_error());
        }
        libc::cfmakeraw(&mut t);
        libc::cfsetispeed(&mut t, libc::B9600);
        libc::cfsetospeed(&mut t, libc::B9600);
        t.c_cflag |= libc::CREAD | libc::CLOCAL | libc::CS8;
        if libc::tcsetattr(fd, libc::TCSANOW, &t) != 0 {
            return Err(std::io::Error::last_os_error());
        }
        let flags = libc::fcntl(fd, libc::F_GETFL);
        if flags < 0 || libc::fcntl(fd, libc::F_SETFL, flags | libc::O_NONBLOCK) < 0 {
            return Err(std::io::Error::last_os_error());
        }
    }
    Ok(())
}

#[cfg(not(unix))]
fn set_9600(_file: &mut std::fs::File) -> std::io::Result<()> {
    Ok(())
}

/// Host unix milliseconds at the PPS edge.
/// `conc_now_us` and `pps_us` are the SX1302 counter, so the age is the time
/// since the edge rather than the time since this process noticed it.
/// `None` when the edge is more than 2 s old.
pub fn host_ms_at_pps(host_ms_now: u64, conc_now_us: u32, pps_us: u32) -> Option<u64> {
    let age_us = conc_now_us.wrapping_sub(pps_us);
    if age_us > 2_000_000 {
        return None;
    }
    Some(host_ms_now.saturating_sub(u64::from(age_us) / 1000))
}

/// Unix milliseconds to set the host to, `age` after the RMC second.
pub fn unix_ms_now(rmc_unix_ms: u64, conc_now_us: u32, pps_us: u32) -> Option<u64> {
    let age_us = conc_now_us.wrapping_sub(pps_us);
    if age_us > 2_000_000 {
        return None;
    }
    Some(rmc_unix_ms.saturating_add(u64::from(age_us) / 1000))
}

/// Step `CLOCK_REALTIME` to `unix_ms`. When `write_rtc` is set, copy it onto the
/// RTC with `hwclock --systohc`. A missing `hwclock` leaves the kernel clock set.
pub fn step_host_and_rtc(unix_ms: u64, write_rtc: bool) -> std::io::Result<()> {
    #[cfg(unix)]
    {
        let mut ts: libc::timespec = unsafe { std::mem::zeroed() };
        ts.tv_sec = (unix_ms / 1000) as _;
        ts.tv_nsec = ((unix_ms % 1000) * 1_000_000) as _;
        if unsafe { libc::clock_settime(libc::CLOCK_REALTIME, &ts) } != 0 {
            return Err(std::io::Error::last_os_error());
        }
        if write_rtc {
            match std::process::Command::new("hwclock").args(["--systohc", "--utc"]).status() {
                Ok(s) if s.success() => {}
                Ok(s) => eprintln!("rtc: hwclock exited {s} (kernel clock was set)"),
                Err(e) => eprintln!("rtc: {e} (kernel clock was set)"),
            }
        }
        Ok(())
    }
    #[cfg(not(unix))]
    {
        let _ = (unix_ms, write_rtc);
        Err(std::io::Error::new(std::io::ErrorKind::Unsupported, "clock_settime"))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn edge_time_uses_the_concentrator_not_the_poll() {
        // Noticed 500 ms late. The counter still says the edge was 500 ms ago.
        assert_eq!(host_ms_at_pps(10_500, 1_500_000, 1_000_000), Some(10_000));
        assert_eq!(unix_ms_now(10_000, 1_500_000, 1_000_000), Some(10_500));
        assert!(host_ms_at_pps(10_000, 3_000_000, 0).is_none());
    }
}
