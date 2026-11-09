use serde::{Deserialize, Serialize};
use std::time::Duration;

#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct TransferStats {
    pub count: u64,
    pub bytes: u64,
    pub errors: u64,
    pub total_ns: u128,
    pub max_ns: u128,
}

impl TransferStats {
    pub fn record(&mut self, bytes: usize, dt: Duration, ok: bool) {
        self.count += 1;
        self.bytes += bytes as u64;
        if !ok {
            self.errors += 1;
        }
        let ns = dt.as_nanos();
        self.total_ns += ns;
        if ns > self.max_ns {
            self.max_ns = ns;
        }
    }
}

#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct SpiMetrics {
    pub reads: TransferStats,
    pub writes: TransferStats,
}

impl SpiMetrics {
    pub fn record_read(&mut self, bytes: usize, dt: Duration, ok: bool) {
        self.reads.record(bytes, dt, ok);
    }
    pub fn record_write(&mut self, bytes: usize, dt: Duration, ok: bool) {
        self.writes.record(bytes, dt, ok);
    }
}
