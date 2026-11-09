//! Minimal systemd `sd_notify` (READY=1 / WATCHDOG=1 / STATUS=) over `$NOTIFY_SOCKET`.
//! No-op when not started by systemd (or on non-Linux hosts used for tests).

#[cfg(target_os = "linux")]
pub fn notify(msg: &str) {
    use std::os::linux::net::SocketAddrExt;
    use std::os::unix::net::{SocketAddr, UnixDatagram};
    let Some(path) = std::env::var_os("NOTIFY_SOCKET") else {
        return;
    };
    let path = path.to_string_lossy().into_owned();
    let addr = if let Some(name) = path.strip_prefix('@') {
        SocketAddr::from_abstract_name(name.as_bytes())
    } else {
        SocketAddr::from_pathname(&path)
    };
    if let (Ok(addr), Ok(sock)) = (addr, UnixDatagram::unbound()) {
        let _ = sock.send_to_addr(msg.as_bytes(), &addr);
    }
}

#[cfg(not(target_os = "linux"))]
pub fn notify(_msg: &str) {}

/// Watchdog interval requested by systemd (`$WATCHDOG_USEC`), if any.
pub fn watchdog_interval() -> Option<std::time::Duration> {
    let us: u64 = std::env::var("WATCHDOG_USEC").ok()?.parse().ok()?;
    Some(std::time::Duration::from_micros(us))
}
