//! Prometheus text exposition on a plain TCP listener, served from the main loop.
//! `GET /metrics` (or `/`) returns the page; anything else is 404.

use std::fmt::Write as _;
use std::io::{self, Read, Write};
use std::net::TcpListener;
use std::time::Duration;

pub struct MetricsServer {
    listener: TcpListener,
}

impl MetricsServer {
    pub fn bind(addr: &str) -> io::Result<Self> {
        let listener = TcpListener::bind(addr)?;
        listener.set_nonblocking(true)?;
        Ok(MetricsServer { listener })
    }

    /// Answer every waiting scrape. `render` runs once per request.
    pub fn poll(&self, render: impl Fn() -> String) {
        while let Ok((mut s, _)) = self.listener.accept() {
            // Short timeouts: a slow client must not stall RX polling or TX arming.
            let _ = s.set_nonblocking(false);
            let _ = s.set_read_timeout(Some(Duration::from_millis(100)));
            let _ = s.set_write_timeout(Some(Duration::from_millis(100)));
            let mut buf = [0u8; 1024];
            let n = s.read(&mut buf).unwrap_or(0);
            let req = String::from_utf8_lossy(&buf[..n]);
            let path = req.split_whitespace().nth(1).unwrap_or("");
            let (status, body) = if req.starts_with("GET ") && (path == "/metrics" || path == "/") {
                ("200 OK", render())
            } else {
                ("404 Not Found", "not found\n".to_string())
            };
            let _ = write!(
                s,
                "HTTP/1.1 {status}\r\nContent-Type: text/plain; version=0.0.4\r\nContent-Length: {}\r\nConnection: close\r\n\r\n{body}",
                body.len()
            );
        }
    }
}

/// Builds the text format: one HELP/TYPE header per metric, then its samples.
#[derive(Default)]
pub struct Page(String);

impl Page {
    pub fn counter(&mut self, name: &str, help: &str, v: u64) -> &mut Self {
        self.family(name, "counter", help, &[("", v as f64)])
    }

    pub fn gauge(&mut self, name: &str, help: &str, v: f64) -> &mut Self {
        self.family(name, "gauge", help, &[("", v)])
    }

    /// `labels` is the inside of `{...}`, e.g. `server="a:1700"`; empty for none.
    pub fn family(&mut self, name: &str, kind: &str, help: &str, samples: &[(&str, f64)]) -> &mut Self {
        let _ = writeln!(self.0, "# HELP sx1302_{name} {help}");
        let _ = writeln!(self.0, "# TYPE sx1302_{name} {kind}");
        for (labels, v) in samples {
            if labels.is_empty() {
                let _ = writeln!(self.0, "sx1302_{name} {v}");
            } else {
                let _ = writeln!(self.0, "sx1302_{name}{{{labels}}} {v}");
            }
        }
        self
    }

    pub fn finish(self) -> String {
        self.0
    }
}

/// Label value escaping from the exposition format.
pub fn label(v: &str) -> String {
    v.replace('\\', "\\\\").replace('"', "\\\"").replace('\n', "\\n")
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::net::TcpStream;

    #[test]
    fn page_format() {
        let mut p = Page::default();
        p.counter("rx_total", "Frames.", 3).family(
            "server_up",
            "gauge",
            "Link.",
            &[(&format!("server=\"{}\"", label("a\"b")), 1.0)],
        );
        let s = p.finish();
        assert!(s.contains("# TYPE sx1302_rx_total counter\nsx1302_rx_total 3\n"));
        assert!(s.contains("sx1302_server_up{server=\"a\\\"b\"} 1\n"));
    }

    #[test]
    fn serves_metrics_and_404() {
        let m = MetricsServer::bind("127.0.0.1:0").unwrap();
        let addr = m.listener.local_addr().unwrap();
        let get = |path: &str| {
            let mut c = TcpStream::connect(addr).unwrap();
            write!(c, "GET {path} HTTP/1.1\r\nHost: x\r\n\r\n").unwrap();
            std::thread::sleep(Duration::from_millis(20));
            m.poll(|| "sx1302_up 1\n".into());
            let mut out = String::new();
            c.read_to_string(&mut out).unwrap();
            out
        };
        let ok = get("/metrics");
        assert!(ok.starts_with("HTTP/1.1 200 OK") && ok.ends_with("sx1302_up 1\n"));
        assert!(get("/nope").starts_with("HTTP/1.1 404"));
    }
}
