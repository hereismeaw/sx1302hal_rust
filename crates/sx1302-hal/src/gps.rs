//! NMEA RMC from a GPS module that is not fitted yet. The daemon opens the
//! port only when `gps` is set in the config.

use crate::classb::gps_ms;

/// One valid `$--RMC` fix. Time is GPS milliseconds (UTC from the sentence plus leap seconds).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Rmc {
    pub gps_ms: u64,
    /// UTC Unix milliseconds from the sentence, before the leap-second conversion.
    pub unix_ms: u64,
    pub lat_deg: f64,
    pub lon_deg: f64,
}

/// Parse one line. `None` unless the checksum matches and status is `A`.
pub fn parse_rmc(line: &str) -> Option<Rmc> {
    let line = line.trim();
    let (body, ck) = line.split_once('*')?;
    let body = body.strip_prefix('$')?;
    let expect = u8::from_str_radix(ck.get(..2)?, 16).ok()?;
    if body.bytes().fold(0u8, |a, b| a ^ b) != expect {
        return None;
    }
    let f: Vec<&str> = body.split(',').collect();
    if f.len() < 10 || !f[0].ends_with("RMC") || f[2] != "A" {
        return None;
    }
    let unix_ms = unix_ms_rmc(f[1], f[9])?;
    let lat = nmea_deg(f[3], f[4], "S")?;
    let lon = nmea_deg(f[5], f[6], "W")?;
    Some(Rmc { gps_ms: gps_ms(unix_ms), unix_ms, lat_deg: lat, lon_deg: lon })
}

/// Signed 24-bit beacon coordinate. `span` is 90 for latitude and 180 for longitude.
pub fn coord24(deg: f64, span: f64) -> i32 {
    let v = (deg / span * 8_388_608.0).round() as i32;
    v.clamp(-8_388_608, 8_388_607)
}

fn nmea_deg(raw: &str, hemi: &str, neg: &str) -> Option<f64> {
    if raw.is_empty() {
        return Some(0.0);
    }
    let dot = raw.find('.')?;
    if dot < 2 {
        return None;
    }
    let (deg, min) = raw.split_at(dot - 2);
    let deg: f64 = deg.parse().ok()?;
    let min: f64 = min.parse().ok()?;
    let mut v = deg + min / 60.0;
    if hemi == neg {
        v = -v;
    }
    Some(v)
}

/// `hhmmss.ss` and `ddmmyy` to Unix UTC milliseconds.
fn unix_ms_rmc(t: &str, date: &str) -> Option<u64> {
    if t.len() < 6 || date.len() != 6 {
        return None;
    }
    let hh: u32 = t[0..2].parse().ok()?;
    let mm: u32 = t[2..4].parse().ok()?;
    let ss: u32 = t[4..6].parse().ok()?;
    let frac_ms: u32 = if let Some(frac) = t.get(7..) {
        let mut d = frac.to_string();
        d.truncate(3);
        while d.len() < 3 {
            d.push('0');
        }
        d.parse().ok()?
    } else {
        0
    };
    let day: u32 = date[0..2].parse().ok()?;
    let month: u32 = date[2..4].parse().ok()?;
    let yy: i32 = date[4..6].parse().ok()?;
    let year = if yy >= 80 { 1900 + yy } else { 2000 + yy };
    let days = days_from_civil(year, month, day)?;
    let sec = days * 86_400 + i64::from(hh) * 3600 + i64::from(mm) * 60 + i64::from(ss);
    Some((sec as u64) * 1000 + u64::from(frac_ms))
}

fn days_from_civil(y: i32, m: u32, d: u32) -> Option<i64> {
    if !(1..=12).contains(&m) || !(1..=31).contains(&d) {
        return None;
    }
    let y = if m <= 2 { y - 1 } else { y };
    let era = if y >= 0 { y } else { y - 399 } / 400;
    let yoe = (y - era * 400) as u64;
    let mp = if m > 2 { m - 3 } else { m + 9 };
    let doy = (153 * mp as u64 + 2) / 5 + d as u64 - 1;
    let doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;
    Some(era as i64 * 146_097 + doe as i64 - 719_468)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn frame(body: &str) -> String {
        let x = body.bytes().fold(0u8, |a, b| a ^ b);
        format!("${body}*{x:02X}\r\n")
    }

    #[test]
    fn rmc_epoch_and_bangkok_sample() {
        // 2024-01-01 00:00:00 UTC. gps = unix - 315964800 + 18.
        let line = frame("GNRMC,000000.00,A,1345.0000,N,10030.0000,E,0.0,0.0,010124,,,A");
        let r = parse_rmc(&line).unwrap();
        assert_eq!(r.unix_ms, 1_704_067_200_000);
        assert_eq!(r.gps_ms, (1_704_067_200 - 315_964_800 + 18) * 1000);
        assert!((r.lat_deg - 13.75).abs() < 1e-6);
        assert!((r.lon_deg - 100.5).abs() < 1e-6);
        assert!(parse_rmc(&frame("GNRMC,000000.00,V,1345.0000,N,10030.0000,E,0.0,0.0,010124,,,A")).is_none());
        assert!(parse_rmc("$GNRMC,000000.00,A,0000.00,N,00000.00,E,0,0,010124,,,A*00").is_none());
    }

    #[test]
    fn equator_is_zero() {
        assert_eq!(coord24(0.0, 90.0), 0);
        assert_eq!(coord24(90.0, 90.0), 8_388_607);
        assert_eq!(coord24(-180.0, 180.0), -8_388_608);
    }
}
