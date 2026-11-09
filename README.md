# sx1302hal_rust

Native **Rust** HAL for Semtech **SX1302 / SX1303** (WM1302 SPI) on Raspberry Pi.
No C HAL and no FFI at runtime.

Pinned to Semtech `sx1302_hal` **v2.1.0**
(`4b42025d1751e04632c0b04160e0d29dbbb222a5`).

Lab hardware: Raspberry Pi 5 + original SenseCAP M1 (WM1302 SPI), region **AS923-1** (920–925 MHz).

## Status — not fully tested

This is an **early, hardware-lab HAL**. It is **not** a drop-in replacement for Semtech `libloragw` on every board or region.

**Exercised on one SenseCAP M1 + Pi 5 + T-Beam (AS923-1):** SPI detect, GPIO reset, SX1250 clock, OTP EUI, AGC/ARB load (fw 10 / 2), 8×125 kHz RX, live OTA RX/TX with T-Beam, legacy PPS counters, LoRa service + FSK config, SX126 LBT RSSI sample, LED GPIO22, pairing button GPIO27.

**Not finished / not soaked:**
- ChirpStack (or any LNS) join / downlink loop in production
- LBT gating of TX (`txpk`)
- WM1302 TX gain LUT and full image-cal table
- Fine timestamp (this silicon is SX1302; SX1303-only)
- GPS 1PPS (counter stays 0 on this carrier)
- EU868 / US915 full channelizer `start`
- Spectral scan, Class B beacon, timestamped TX
- ATECC608 (I2C `0x60` did not respond; writes remain blocked)
- Multi-HAT GPIO maps (button/LED differ by carrier)

Treat `main` as **alpha**. Do not ship RF without your own air and regulatory checks.

## Why this is better than the C HAL

| Topic | Semtech `libloragw` | This crate |
|---|---|---|
| Language | C, global COM handle | Rust, `Concentrator` owns the SPI fd |
| Register map | `loregs[1044]` + `#define` IDs | Same 1044 fields, typed `Field`, pack/unpack tested |
| Errors | `int` / `fprintf` | `Result` + `thiserror` |
| SPI | 1-byte RMW, small bursts | Same protocol; Linux burst capped at **1024** (avoids `EMSGSIZE`) |
| Region | Caller must get it right | TX refused without `--region`; AS923-1 band-checked |
| GPIO | Board scripts (`reset_lgw.sh`) | SenseCAP map: GPIO18 POWER_EN, 17 RESET, 5 SX126 RESET. **GPIO13 never driven** |
| Aux radio | `SPI_NO_CS` (fails on bcm2712) | LBT via `/dev/spidev0.1` + GPIO6 CS |
| Fine timestamp | Silent on SX1302 | Refused when OTP model is SX1302 |
| ATECC | Helium lock/write in other stacks | Inspect-only; lock/write blocked |
| Packet path | C packet forwarder | `sx1302-daemon` GWMP UDP (`PUSH_DATA` / `PULL_DATA` / `txpk`) |
| Tests | Sparse | Protocol + bitfield + region unit tests |

Register **addresses are not invented**. They are the published Semtech table.

## Crates

- `sx1302-registers` — full `loragw_reg.c` map (1044 fields) + IF/freq formulas
- `sx1302-transport` — Linux `spidev` SPI_IOC_MESSAGE, mux `00/01/02`
- `sx1302-hal` — detect, bringup, `start` AS923-1, RX FIFO, TX_TOP_A, timestamps
- `sx1302-firmware` — AGC/ARB blobs (version 10 / 2)
- `sx1302-radio` — SX1250 opcodes
- `sx126x-radio` — identify + LBT RSSI (no SX126 TX)
- `apps/sx1302-cli`, `sx1302-daemon`, `sx1302-bench`

## Build

```bash
# on the Pi (rustc 1.85+)
source "$HOME/.cargo/env"
cargo test -p sx1302-registers -p sx1302-core -p sx1302-timestamp
cargo build --release -p sx1302-cli -p sx1302-daemon
```

## Hardware loop (SenseCAP M1)

```text
sudo ./target/release/sx1302-cli gpio reset
sudo ./target/release/sx1302-cli detect
sudo ./target/release/sx1302-cli start --region AS923-1
sudo ./target/release/sx1302-cli rx --seconds 10
sudo ./target/release/sx1302-cli tx-pkt --region AS923-1 --payload 47573031
sudo ./target/release/sx1302-cli timestamp
sudo ./target/release/sx1302-cli lbt --freq 923200000
sudo ./target/release/sx1302-daemon --server chirpstack.example:1700
```

GPIO13 (AD5338R) is never opened. GPIO18 is POWER_EN on this carrier (not Seeed BUSY).

## Register API

```rust
use sx1302_registers::{field, pack_into_byte, unpack_from_byte, ids};

let f = field(ids::COMMON_CTRL1_SWAP_IQ_RADIO_A)?; // addr 0x5602 bit 1
let byte = pack_into_byte(f, 0x00, 1)?;            // -> 0x02
assert_eq!(unpack_from_byte(f, byte)?, 1);
```

Regenerate the table after bumping the Semtech pin:

```bash
python tools/gen_loregs.py
```

## Contributors

GitHub counts commits whose **author email is a verified address on the GitHub account**.
This repo is authored as **[@hereismeaw](https://github.com/hereismeaw)** (`Kittiwut Khongkaeo`).

See [CONTRIBUTORS.md](CONTRIBUTORS.md).

## License

Rust code: MIT OR Apache-2.0.  
Semtech firmware and register table: Revised BSD (see `NOTICE` and `firmware/`).
