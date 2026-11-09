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

## GWMP / ChirpStack + timed downlink (2026-11-09, Claude)

- `send_lora` supports `TxTrigger::Timestamped(tmst)` (TX_TRIG_DELAYED, timer = tmst·32 − TX_START_DELAY 0xBB80), `invert_pol` (CHIRP_INVERT), preamble, SF-dependent CHIRP_LOWPASS / FINE_SYNCH / PPM_OFFSET (SF11/12). TOO_LATE (<1 ms) / TOO_EARLY (>10 s, JoinAccept needs 5–6 s) refused; TX FSM busy → COLLISION_PACKET.
- Daemon: TX_ACK echoes PULL_RESP token with `txpk_ack.error`; honours `imme`/`tmst`/`ipol`/`prea`; `stat.time` = `YYYY-MM-DD hh:mm:ss GMT`, real rxnb/rxok/rxfw/dwnb/txnb/ackr; forwards CRC-OK only, de-duplicates multi-SF vs LoRa-service copies; logs PULL_ACK.
- `tools/gwmp_echo_ns.py`: fake NS on the Pi answering each `TBEAM` uplink with RX1 (`tmst+1 s`, `ipol=true`). T-Beam firmware now listens with inverted IQ after each TX and prints `start=` (uplink end → downlink start).
- **Measured:** 4/4 downlinks `start=999.72 ms` (target 1000 ms), CRC OK at T-Beam, TX_ACK `NONE`.
- **ChirpStack 192.168.0.101:1700:** PULL_ACK + PUSH_ACK 100 %; gateway `0016c001f106659f` registered in tenant *Southern IoT* as "Meaw - sx1302-hal-rs dev (Pi5 SenseCAP M1)"; `lastSeenAt` updates from our `stat`.
- **OTAA end-to-end (07:24–07:27 UTC):** `tools/tbeam-otaa-node` (RadioLib 7.8 LoRaWAN 1.0.3, keys in git-ignored `src/keys.h`) joined ChirpStack device `70b3d57ed0087e58` (app "DEV sx1302-hal-rs", profile "DEV sx1302-hal-rs T-Beam OTAA") through this gateway. JoinAccept 33 B on RX1 (+5 s), confirmed uplinks on 922.6–924.0 MHz all ACKed in RX1, ADR moved SF9→SF7, queued app downlink (fPort 10, `Hello Rust`) delivered. A downlink right after the 2^27 µs `tmst` wrap also landed.
- Still open: `powe` → power LUT mapping (fixed index 22); RX `tmst` wraps at 2^27 µs (TX math is wrap-safe, but expand to 32-bit for NS-side stats); RX timestamp correction per SF (legacy end-of-packet ts).

## Channel plan, JIT, FSK, 32-bit tmst, SF timestamp correction (2026-11-09, Claude)

- **Channel plan** (`ChannelPlan`, `crates/sx1302-hal/src/plan.rs`, JSON in `configs/sx1302-daemon.json`, installed as `/etc/sx1302-daemon.json`): radio A 922.9 MHz (922.6/922.8/923.0/923.2), radio B 923.7 MHz (923.4/923.6/923.8/924.0) = ChirpStack `as923` plan; LoRa service chain 924.3 MHz SF7 BW250 (radio B); FSK 922.3 MHz 50 kb/s (radio A). Validation: inside 920–925 MHz and |IF|+BW/2 ≤ 800 kHz. `Concentrator::start(&plan)`; `start_as923_1()` = default plan (no service/FSK).
- **Radio B fix:** with `single_input_mode` on radio B, RSSI was ~40 dB low (-83 vs -42 dBm, same node). Differential input on B fixes it; A identical either way. Default `single_input_mode: [true, false]`.
- **32-bit tmst:** `TsWrap` expands the 27-bit µs counter (modular distance, handles packets finishing between the counter read and the FIFO read). TX math unchanged (`tmst·32 mod 2^32` = raw counter). Verified with ChirpStack downlinks at tmst > 2^27.
- **RX timestamp correction:** Semtech `legacy_timestamp_correction` ported (SF7 12 B: -468 µs, grows with SF); FSK `680000/dr - 20`. TX start delay now per Semtech `sx1302_tx_set_start_delay` (48000 − 19 − filter − 64 ticks).
- **JIT queue** (`apps/sx1302-daemon/src/jit.rs`): TX_ACK on enqueue, arm 100 ms before emission (arming takes ~3 ms), TOO_LATE < 20 ms, TOO_EARLY > 30 s, COLLISION_PACKET on overlapping air time.
- **FSK:** RX parse (chain 9, `modu:"FSK"`, RSSI linearisation) and TX (`sx1302_send` FSK branch incl. AGC mailbox fdev).
- **Hardware test** (`tools/tbeam-as923-node` cycles 8 cases, `tools/gwmp_echo_ns.py` answers +1 s / +2 s): all cases RX + TX on both radios. Downlink start (T-Beam, uplink end → downlink start, target 1000 ms): SF7 999.26, SF9 1002.28, SF10 1005.35, SF11 1011.83, SF12 1025.99, SF7BW250 998.87, FSK 1001.03. Fits `-1.6 ms + 0.84·Tsym`; the Tsym term is consistent with SX1276 RxDone latency on the node, gateway side follows Semtech's model. SF12 second downlink correctly refused (COLLISION_PACKET).
- **TX power (`powe`)**: Semtech `tx_gain_lut` (SX1250 reference table 12–27 dBm, `power = (pa_en<<6)|pwr_idx`), overridable via `tx_gain_lut` in the daemon config; selection = highest entry ≤ `powe`. The old hard-coded `pa=1 idx=22` was beyond the table's top (27 dBm = pa 1 idx 14) — that caused the CRC failures at close range: after the fix 67/67 test downlinks OK (was 2/45 CRC errors), T-Beam RSSI −50…−60 dBm instead of −20 at the same spot. ChirpStack AS923 sends `powe` 16 → pa 0 idx 20. Max 27 dBm conducted + antenna gain stays below the Thai 920–925 MHz limit of 4 W (36 dBm) EIRP.
- `single_input_mode` default `[false, false]` (Semtech reference; radio A measured identical either way).
- **Semtech reference comparison** (`tools/semtech-ref/`, operator-approved, test only): `lora_pkt_fwd` v2.1.0 on the same board, same plan, same T-Beam suite. Downlink start times identical to the Rust HAL within 0.01 ms for every case (so the SF-dependent excess is node-side SX1276 latency); RSSI identical (LoRa −40…−45, **FSK −85…−87 dBm — the FSK/LoRa gap is Semtech's own formula, not a bug**). Semtech also forwards weak cross-channel ghost copies (−95/−102 dBm); our de-dup drops them.
- **Watchdog**: unit is `Type=notify`, `WatchdogSec=30`. The daemon sends READY=1, then WATCHDOG=1 every 15 s only after a health check (SPI `COMMON_VERSION`=0x10 and 32 MHz counter advancing); a failed check exits → restart with GPIO reset. Verified: SIGSTOP → `Watchdog timeout` → restart → PULL_ACK. `systemctl status` shows STATUS (server link, JIT queue, board temperature).
- **RSSI temperature compensation**: STTS751 found at I2C-1 0x39 (man 0x53, prod 0x00; read-only use). `rssi_offset` / `rssi_tcomp` per radio in the plan (Semtech defaults), applied like `loragw_hal.c` (+1.77 dB at 39 °C).

## Next operator steps

1. `gpio reset` then `start --region AS923-1` on the Pi; confirm AGC handshake (no timeout).
2. Listen with `rx` or `sx1302-daemon` while a node TX at 923.2 MHz SF7 BW125.
3. `tx-pkt` to a known receiver; confirm air packet (spectrum or node).
4. If AGC times out: radio clock not running — run `bringup` first and check GET_STATUS.
5. ChirpStack: `sx1302-daemon --server host:1700` now PUSH_DATA + PULL_DATA and executes `txpk` via `send_lora`.

## SSH

`192.168.0.23` user `pi` (lab password on file). Rust via `~/.cargo/env`.
