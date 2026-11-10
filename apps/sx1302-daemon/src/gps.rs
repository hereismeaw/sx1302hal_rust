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
