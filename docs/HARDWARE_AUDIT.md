# Stage 1 hardware audit — Raspberry Pi 5 + SenseCAP M1 HAT

**Date:** 2026-11-09  
**Host IP:** 192.168.0.23 (`sensecap-rpi5-dev`)

## Upstream SHAs

| Source | SHA |
|---|---|
| Semtech `sx1302_hal` master | `4b42025d1751e04632c0b04160e0d29dbbb222a5` (v2.1.0) |

## Observed host (SSH, read-only)

- Board: Raspberry Pi 5 Model B Rev 1.0
- CPU: ARM Cortex-A76 r4p1 × 4, 2400 MHz
- Features: `fp asimd evtstrm aes pmull sha1 sha2 crc32 atomics fphp asimdhp cpuid asimdrdm lrcpc dcpop asimddp`
- OS: Debian GNU/Linux 13 (trixie) (`DEBIAN_VERSION_FULL=13.7`)
- Kernel: `6.18.50+rpt-rpi-2712` aarch64
- User `pi` in groups: `gpio`, `spi`, `i2c`, `sudo`

## Interfaces present

| Node | Notes |
|---|---|
| `/dev/spidev0.0` | SPI0 CE0 — SX1302 CS candidate (Seeed GPIO8) |
| `/dev/spidev0.1` | SPI0 CE1 — **not** used as SX126x CS on Seeed HAT (GPIO6 is GPIO-CS) |
| `/dev/i2c-1` | User I2C (SDA/SCL GPIO2/3) |
| `/dev/i2c-13`, `/dev/i2c-14` | RP1 internal |
| `/dev/gpiochip0` | `pinctrl-rp1` 54 lines — use this chip, never sysfs global numbers |
| USB | No WM1302 USB VID/PID (SPI variant) |

Rust was **not** installed at first inspect.

## GPIO18 conflict (blocking)

| Document | GPIO18 |
|---|---|
| Seeed WM1302 module wiki pin table | `SX1262_BUSY` (must be **input**) |
| Semtech `reset_lgw.sh` + SenseCAPM1-Chirpstack `reset_lgw.sh` | `SX1302_POWER_EN` (**output**) |

GPIO13 `AD5338R_RESET` appears only in Semtech CN490 full-duplex scripts, not on the Seeed HAT table.

**Policy (updated):** operator authorized SenseCAP M1 GPIO drive. GPIO18 is **POWER_EN output**. GPIO17 SX1302_RESET and GPIO5 SX1262_RESET are driven. GPIO13 AD5338R is **never driven**. TX requires `--region`. AS923-1 is 920–925 MHz (default 923.2 MHz).

## SPI protocol (Semtech, not guessed)

From `loragw_spi.c`:

- Mode 0, 8-bit, MSB first, default 2 MHz
- Frame: `[mux][addr_hi|R/W][addr_lo][dummy?][data…]`
- `WRITE_ACCESS = 0x80`
- Mux: SX1302=0x00, RadioA=0x01, RadioB=0x02
- `COMMON_VERSION` address `0x5606`, expected `0x10`

## Hardware SPI detect (2026-11-09, no GPIO writes)

`sx1302-cli detect --spi /dev/spidev0.0` on `sensecap-rpi5-dev`:

```json
{
  "identity": {
    "version": 16,
    "matches_sx1302": true,
    "spi_path": "/dev/spidev0.0",
    "mux": "Sx1302"
  },
  "dummy": 0,
  "reset_performed": false
}
```

`COMMON_VERSION` = `0x10` matches Semtech reset-default. Concentrator is already out of reset on this carrier; GPIO18/13/17 were not driven.

## Radio bring-up (2026-11-09, no Pi GPIO writes)

`sx1302-cli bringup --spi /dev/spidev0.0`:

- Internal SX1250 reset via AGC_MCU_RF_EN (PA/LNA off)
- Radio A/B `GET_STATUS` = `0x22` (`chip_mode` 2 = STANDBY_RC)
- Radio A `SET_RX` at 868.5 MHz dummy (Semtech chip_id frequency); SX1302 clock from radio A
- OTP EUI **`0x0016c001f106659f`**
- OTP model_id byte `0x00`

No `SET_TX`. No image calibration (region not locked).

Toolchain on device: `rustc 1.99.1` aarch64. Workspace tests passed. Release CLI/bench built with thin LTO.

## Verification procedure (non-destructive)

1. `sx1302-cli board inspect` — JSON inventory.
2. `sx1302-cli detect` — SPI read of `COMMON_VERSION` **without** GPIO reset.
3. If version is not `0x10`, measure SX1302_RESET (BCM17) with a scope/logic analyzer as **input** first; confirm POWER_EN vs BUSY on BCM18 **as input**.
4. Only then mark pins `VerifiedOnHardware`.
