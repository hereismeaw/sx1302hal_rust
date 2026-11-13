//! UTC formatting without a date crate.

/// Howard Hinnant's `civil_from_days`: days since 1970-01-01 -> (year, month, day).
fn civil_from_days(days: i64) -> (i64, i64, i64) {
    let z = days + 719_468;
    let era = z.div_euclid(146_097);
    let doe = z - era * 146_097;
    let yoe = (doe - doe / 1460 + doe / 36_524 - doe / 146_096) / 365;
    let doy = doe - (365 * yoe + yoe / 4 - yoe / 100);
    let mp = (5 * doy + 2) / 153;
    let d = doy - (153 * mp + 2) / 5 + 1;
    let m = if mp < 10 { mp + 3 } else { mp - 9 };
    (yoe + era * 400 + i64::from(m <= 2), m, d)
}

/// GWMP `stat.time`: "YYYY-MM-DD hh:mm:ss GMT".
pub fn format_utc(secs: u64) -> String {
    let (y, m, d) = civil_from_days((secs / 86_400) as i64);
    let tod = secs % 86_400;
    format!(
        "{y:04}-{m:02}-{d:02} {:02}:{:02}:{:02} GMT",
        tod / 3600,
        (tod / 60) % 60,
        tod % 60
    )
}

/// GWMP `rxpk.time`: ISO 8601 UTC with microseconds, "2013-03-31T16:21:17.528000Z".
pub fn format_iso(unix_ms: u64) -> String {
    let secs = unix_ms / 1000;
    let (y, m, d) = civil_from_days((secs / 86_400) as i64);
    let tod = secs % 86_400;
    format!(
        "{y:04}-{m:02}-{d:02}T{:02}:{:02}:{:02}.{:06}Z",
        tod / 3600,
        (tod / 60) % 60,
        tod % 60,
        (unix_ms % 1000) * 1000
    )
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn formats() {
        assert_eq!(format_utc(0), "1970-01-01 00:00:00 GMT");
        assert_eq!(format_utc(1_709_210_096), "2024-02-29 12:34:56 GMT");
        assert_eq!(format_iso(1_709_210_096_789), "2024-02-29T12:34:56.789000Z");
    }
}
