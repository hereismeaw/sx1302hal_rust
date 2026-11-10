//! Class B beacon (TS001) and GPS-millisecond scheduling.
//!
//! AS923-1 (RP002): 923.4 MHz, DR3 SF9/BW125, CR 4/5, non-inverted IQ, 17-byte
//! implicit frame, no LoRa CRC. Beacon instants are GPS epoch + n·128 s + 1.5 ms.
//! shortcut: no GPS module is fitted. Until a PPS edge is paired with an RMC
//! fix, map host UTC to the concentrator counter. Do not enable `class_b`
//! beside a GPS-locked beacon network while this gateway is still on host time.

/// Seconds between beacons.
pub const BEACON_PERIOD_S: u64 = 128;
/// TX offset after the GPS second so overlapping gateways stay aligned (TS001).
pub const BEACON_DELAY_US: u32 = 1_500;
/// UTC−GPS, seconds. GPS does not count leap seconds. 18 since 2017-01-01.
const LEAP_S: u64 = 18;
/// Unix seconds of 1980-01-06 00:00:00 UTC.
const GPS_EPOCH_UNIX_S: u64 = 315_964_800;
/// AS923-1 beacon / default ping-slot channel.
pub const AS923_1_BEACON_HZ: u32 = 923_400_000;

/// GPS milliseconds from a Unix UTC millisecond timestamp.
pub fn gps_ms(unix_ms: u64) -> u64 {
    unix_ms.saturating_add(LEAP_S * 1000).saturating_sub(GPS_EPOCH_UNIX_S * 1000)
}

/// Next GPS-second boundary (milliseconds) that is a multiple of 128 s and
/// strictly after `gps_now_ms`. Add [`BEACON_DELAY_US`] when scheduling TX.
pub fn next_beacon_gps_ms(gps_now_ms: u64) -> u64 {
    let period = BEACON_PERIOD_S * 1000;
    (gps_now_ms / period + 1) * period
}

/// Concentrator `tmst` (µs) at `target_gps_ms`, from one paired sample.
/// The delta must fit in about ±35 minutes (`i32` microseconds).
pub fn count_at(count_us: u32, gps_now_ms: u64, target_gps_ms: u64) -> u32 {
    let delta_us = (target_gps_ms as i64 - gps_now_ms as i64) * 1000;
    count_us.wrapping_add_signed(delta_us as i32)
}

/// CRC-16 used by the beacon (LoRaMac `RegionCommonComputeCrc`).
pub fn crc16(data: &[u8]) -> u16 {
    let mut crc: u16 = 0;
    for &b in data {
        crc = crc.rotate_right(8);
        crc ^= u16::from(b);
        crc ^= (crc & 0xff) >> 4;
        crc ^= crc << 12;
        crc ^= (crc & 0xff) << 5;
    }
    crc
}

/// 17-byte AS923 beacon. `InfoDesc = 3` (NetID + GatewayID), 24 bits each, little-endian.
pub fn beacon_payload(gps_s: u32, net_id: u32, gateway_id: u32) -> [u8; 17] {
    let mut info = [0u8; 7];
    info[0] = 3;
    info[1..4].copy_from_slice(&net_id.to_le_bytes()[..3]);
    info[4..7].copy_from_slice(&gateway_id.to_le_bytes()[..3]);
    finish_beacon(gps_s, &info)
}

/// `InfoDesc = 0`: antenna WGS84, used once a GPS module has a fix.
pub fn beacon_antenna(gps_s: u32, lat_deg: f64, lon_deg: f64) -> [u8; 17] {
    let mut info = [0u8; 7];
    info[0] = 0;
    let lat = crate::gps::coord24(lat_deg, 90.0);
    let lon = crate::gps::coord24(lon_deg, 180.0);
    info[1..4].copy_from_slice(&lat.to_le_bytes()[..3]);
    info[4..7].copy_from_slice(&lon.to_le_bytes()[..3]);
    finish_beacon(gps_s, &info)
}

fn finish_beacon(gps_s: u32, gw: &[u8; 7]) -> [u8; 17] {
    let mut p = [0u8; 17];
    p[2..6].copy_from_slice(&gps_s.to_le_bytes());
    let c1 = crc16(&p[..6]).to_le_bytes();
    p[6..8].copy_from_slice(&c1);
    p[8..15].copy_from_slice(gw);
    let c2 = crc16(&p[8..15]).to_le_bytes();
    p[15..17].copy_from_slice(&c2);
    p
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn crc_and_beacon_fields() {
        assert_eq!(crc16(&[0x01]), 0x1021);
        assert_eq!(crc16(&[0, 0, 0, 0, 0, 0]), 0);
        let p = beacon_payload(128, 0x010203, 0x040506);
        assert_eq!(&p[2..6], &128u32.to_le_bytes());
        assert_eq!(u16::from_le_bytes([p[6], p[7]]), crc16(&p[..6]));
        assert_eq!(p[8], 3);
        assert_eq!(&p[9..12], &[0x03, 0x02, 0x01]);
        assert_eq!(&p[12..15], &[0x06, 0x05, 0x04]);
        assert_eq!(u16::from_le_bytes([p[15], p[16]]), crc16(&p[8..15]));
    }

    #[test]
    fn beacon_grid_and_count() {
        // 1980-01-06 00:00:00 UTC + 18 leap seconds of GPS ahead of UTC.
        let unix0 = GPS_EPOCH_UNIX_S * 1000 - LEAP_S * 1000;
        assert_eq!(gps_ms(unix0), 0);
        assert_eq!(next_beacon_gps_ms(0), 128_000);
        assert_eq!(next_beacon_gps_ms(128_000), 256_000);
        assert_eq!(next_beacon_gps_ms(128_000 - 1), 128_000);
        assert_eq!(count_at(1_000, 5_000, 5_002), 1_000 + 2_000);
    }
}
