# Handover — sx1302-hal-rs

**Date:** 2026-11-09  
**Host:** Raspberry Pi 5 (`192.168.0.23`, user `pi`) + original SenseCAP M1 (WM1302 SPI)  
**Workspace:** `sx1302-hal-rs/`  
**Upstream Semtech HAL:** `4b42025d1751e04632c0b04160e0d29dbbb222a5` (v2.1.0)  
**No C HAL / FFI at runtime.**

## What this tree is

Native Rust concentrator HAL. SPI protocol, register addresses, AGC/ARB firmware, AGC mailbox handshake, and TX_TOP fields are copied from published Semtech sources — they are not invented.

## Hardware facts (measured)

| Item | Value |
|---|---|
| Board | Raspberry Pi 5 Model B Rev 1.0, Debian 13, gpiochip0 `pinctrl-rp1` |
| SPI | `/dev/spidev0.0` Mode 0, 2 MHz, mux `00/01/02` |
| `COMMON_VERSION` | `0x10` |
| OTP EUI | `0x0016c001f106659f` |
| OTP model | `0x00` (SX1302) |
| Radios | SX1250 A/B, GET_STATUS `0x22` STANDBY_RC after internal reset |

### GPIO (SenseCAP M1, operator-authorized)

| BCM | Signal | Drive |
|---|---|---|
| 18 | `SX1302_POWER_EN` | output HIGH |
| 17 | `SX1302_RESET` | active HIGH pulse |
| 5 | `SX1262_RESET` | active LOW pulse |
| 6 | SX126 CS | output, identify only |
| 13 | AD5338R | **never drive** |

GPIO18 is POWER_EN on this carrier (SenseCAP / Semtech `reset_lgw.sh`), not Seeed BUSY. After `gpio reset`, `pinctrl set 18 op dh` keeps POWER_EN after the CLI exits.

### Region

**AS923-1 only for `start` / packet TX default:** 920–925 MHz inclusive, default TX **923.2 MHz**. `--region AS923` / `AS923-1` parse to the same enum. EU868/US915 remain for the older `tx` CW helper only.

## Commands

On the Pi (`source ~/.cargo/env`, tree at `/home/pi/sx1302-hal-rs`):

```bash
cargo build --release -p sx1302-cli -p sx1302-daemon -p sx1302-bench

sx1302-cli board inspect
sx1302-cli gpio reset
sx1302-cli detect
sx1302-cli bringup
sx1302-cli start --region AS923-1
sx1302-cli rx
sx1302-cli tx-pkt --region AS923-1 --sf 7 --payload 00
sx1302-cli tx --region AS923-1          # SX1250 SET_TX + PA_EN (not a LoRa frame)
sx1302-cli load-fw --agc firmware/agc_fw_sx1250.bin --arb firmware/arb_fw.bin
sx1302-cli atecc                        # I2C 0x60 read-only
sx1302-cli sx126                        # GPIO6 CS probe
sx1302-daemon                           # start + poll RX JSON lines
```

## Crate map

| Crate | Role |
|---|---|
| `sx1302-core` | errors, Region AS923-1 band, identity |
| `sx1302-transport` | spidev, mux frames (`loragw_spi.c`) |
| `sx1302-registers` | bases + `if_hz_to_reg` / `sx1302_freq_to_reg` |
| `sx1302-radio` | SX1250 reset, SET_RX/SET_TX, host PA |
| `sx1302-firmware` | 8192-byte AGC/ARB `include_bytes!` |
| `sx1302-hal` | detect, bringup, `start_as923_1`, `rx_poll`, `send_lora` |
| `sensecap-board` | gpiochip0 reset sequence |
| `sensecap-secure-element` | ATECC inspect, writes blocked |
| `sx126x-radio` | aux identify |
| `sensecap-button` / `led` | still refuse (GPIO unknown) |

Firmware files: `firmware/agc_fw_sx1250.bin`, `firmware/arb_fw.bin` (from Semtech `.var`, BSD).

## Implemented vs Semtech `lgw_start`

Done:

- GPIO reset, SX1250 reset/clock at 923.2 / 923.4 MHz
- AGC+ARB load + verify, `agc_start` mailbox (version **10**, `agc_params_sx1250`)
- 8 multi-SF IF channels (offsets in `AS923_1_IF_HZ`)
- `sx1302_radio_fe_configure`, `OTP_MODEM_EN_0/1=0xFF`, public syncword 0x34 (SF7–12 peak 6/8)
- correlator ACC_PNR 52 / MSP, `COMMON_GEN`, HOST_RADIO_CTRL released to AGC
- RX FIFO parse (`A5 C0`) + GWMP `rxpk`
- TX_TOP_A LoRa immediate send (`0x5300` payload RAM)

Not a 1:1 of the whole C HAL:

- Fine timestamp / PPS
- SX1250 image calibration
- LoRa service + FSK IF chains
- SX126 LBT / spectral scan
- Pairing button / LED (pin unknown)
- Fine timestamp / PPS, LoRa service + FSK, SX126 LBT, pairing button/LED

## Safety

- GPIO13 never driven
- ATECC: no Lock / Write / GenKey
- TX frequency must sit in the selected region band
- `sx126` GET_STATUS still shares SPI0 CE0; treat the status byte as untrusted until a `SPI_NO_CS` overlay exists

## Hardware run (2026-11-09, this host)

`sx1302-cli start --region AS923-1` **succeeded** after chunking SPI bursts to 1024 bytes (`EMSGSIZE` on 8192-byte firmware write):

- radios RX @ 923.2 / 923.4 MHz
- AGC+ARB loaded and verified
- AGC handshake fw version 10
- 8×125 kHz multi-SF + radio_fe + OTP_MODEM_EN + public 0x34 + COMMON_GEN
- AGC RF_EN `0x0D` (SX1250 RADIO_RST|RADIO_EN|LNA_EN), ctrl0 `0x13`
- FE RSSI_DEC ~70 (alive); T-Beam COM4 TX `st=0` every 2 s at 923.2 SF7 sync 0x34
- T-Beam COM4: IQ invert (LoRaWAN), `st=0`; GW `SWAP_IQ` bit1/bit3 + `RX_CFG0_SWAP_IQ`, `CALIBRATE_IMAGE` 902–928, `lgw_start` order (modem_enable before AGC)
- **Live T-Beam RX:** CRC OK, payload `TBEAM…`, SF7, RSSI ~−82 dBm, SNR ~11 dB (`snr/4`), IF chain 2. ARB `CLK_EN` bit5 + handshake fw 2.
- Parser: skip size-0, freq_offset × 0.11920929 Hz LSB.
- **Live GW TX (OTA):** `tx-pkt --payload 47573031` → T-Beam `rx n=4 … data=47573031` (RSSI ~−24 dBm, SNR ~8 dB). Root cause was `TX_IF_SRC=0` (CW). LoRa needs `TX_IF_SRC=1`; AGC owns PA (`FORCE_HOST_FE=0`). FSM `0x50` emitting then `0x80` free.

RX FIFO parser (`loragw_sx1302_rx.c`): sync `A5 C0`, head 9 + payload + tail 14, checksum. `rx` JSON now includes `packets[]` (freq, SF, SNR, CRC `stat`, payload). Daemon emits GWMP `PUSH_DATA` when `--server host:port` is set.
- `tx-pkt` programmed TX_TOP_A 923.2 MHz SF7 4 bytes
- ATECC 0x60 no response
- SX126 GET_STATUS `0x50` (CE0 still asserted — untrusted)

## Next operator steps

1. `gpio reset` then `start --region AS923-1` on the Pi; confirm AGC handshake (no timeout).
2. Listen with `rx` or `sx1302-daemon` while a node TX at 923.2 MHz SF7 BW125.
3. `tx-pkt` to a known receiver; confirm air packet (spectrum or node).
4. If AGC times out: radio clock not running — run `bringup` first and check GET_STATUS.
5. ChirpStack: `sx1302-daemon --server host:1700` now PUSH_DATA + PULL_DATA and executes `txpk` via `send_lora`.

## SSH

`192.168.0.23` user `pi` (lab password on file). Rust via `~/.cargo/env`.
