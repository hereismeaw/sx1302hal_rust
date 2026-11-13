# sx1302hal_rust

A LoRa concentrator HAL for the Semtech SX1302 (and the SX1303 on a RAK5146 SPI module), written in Rust from scratch.
It talks to the chip over `spidev` directly. There's no C HAL underneath and no FFI.

I built this on one particular box: a Raspberry Pi 5 inside an original
SenseCAP M1 (WM1302 SPI module), running in Thailand on **AS923-1**
(920–925 MHz). It now runs that gateway day to day against ChirpStack. If you
have a different carrier board or region, expect to do some work.

Everything chip-specific (register addresses, the AGC/ARB firmware, SPI
framing, the timing and RSSI formulas) is taken from Semtech's published
`sx1302_hal` v2.1.0 (`4b42025`). None of the register values are guessed.

## What works

Tested on the real hardware with a LilyGO T-Beam as the node:

- Bring-up: GPIO reset, both SX1250 radios, AGC/ARB firmware load, OTP EUI.
- RX on 8 multi-SF channels split over both radios, plus a LoRa "service"
  channel (e.g. SF7/BW250) and an FSK 50 kb/s channel. The channel plan is a
  JSON file.
- TX of LoRa (SF5–12, BW125/250/500) and FSK, either immediately or at a
  given concentrator timestamp. Power comes from the Semtech SX1250 gain
  table (12–27 dBm), so `powe` from the network server is respected.
- A GWMP / Semtech UDP forwarder (`sx1302-daemon`) with a just-in-time
  downlink queue. It answers `TOO_LATE`, `TOO_EARLY` and `COLLISION_PACKET`
  the way `lora_pkt_fwd` does.
- OTAA join, confirmed uplinks and application downlinks through ChirpStack v4.
- 32-bit `tmst`, decode-latency timestamp correction per SF, and RSSI
  temperature compensation from the board's STTS751 sensor.
- The WM1302 HAT LEDs (Config / TX / RX) through the SX1302's own GPIOs.
- A systemd unit with a watchdog. If the daemon hangs or the chip stops
  answering, systemd restarts it with a fresh GPIO reset.

I also ran Semtech's own `lora_pkt_fwd` on the same board with the same
channel plan to compare. Downlink timing matched to within 0.01 ms at every
SF, and RSSI matched for both LoRa and FSK.

## What doesn't (yet)

- **No GPS module is fitted.** Class B stays off unless `"class_b": true`.
  Beacons are AS923-1 (923.4 MHz, SF9, every 128 s). Time is the host clock
  until `"gps"` names an NMEA port (9600 8N1, for example `/dev/serial0`) and
  the module's 1PPS reaches the SX1302. A valid RMC plus a PPS edge replaces
  the host clock and puts the antenna coordinates in the beacon. Leave
  `class_b` off beside a gateway that already beacons on GPS time.
- **Fine timestamps** are SX1303-only. The calculation is ported from
  Semtech and turns on for a RAK5146, but my unit is an SX1302, so it has
  never run on real hardware.
- **LBT / spectral scan** through the SX1262 isn't wired up. Its chip select
  shares SPI0 CE0 on the M1, so it needs a device-tree overlay first. Thailand
  doesn't require LBT.
- **Only AS923-1 is tested end to end.** US915 and FCC-800 plans, hopping
  and the dwell / duty-cycle limits are in the code and pass their unit
  tests. None of them has been on the air.
- **ATECC608** is inspect-only. It didn't answer on this unit, and every
  write or lock path is blocked on purpose.

Please check your own RF output and local regulations before you transmit
with this.

## Hardware notes (SenseCAP M1)

| What | Where |
|---|---|
| SX1302 SPI | `/dev/spidev0.0`, mode 0, 2 MHz |
| SX1302 power enable | BCM 18 (high) |
| SX1302 reset | BCM 17 (active-high pulse) |
| SX1262 reset | BCM 5 (active-low pulse) |
| Temperature sensor | STTS751, I2C-1 `0x39` |
| HAT LEDs | SX1302 GPIO0 (Config), GPIO2 (TX), GPIO4 (RX) |
| **BCM 13** | AD5338R DAC: **never driven** by this code |

`board` in the daemon config selects the carrier. `rak2287` (SX1302 SPI) and `rak5146` (SX1303 SPI) pulse BCM17 only and can use the onboard ZOE-M8Q (`"gps": "/dev/ttyAMA0"`). A PPS+RMC lock measures the edge from the SX1302 counter and steps the kernel clock when it is more than 20 ms off. `hwclock --systohc` runs on a step of a second or more, and at most every 10 minutes after that. The daemon needs permission to set the clock. A missing `hwclock` leaves the kernel clock set and the RTC unchanged. USB SKUs of those two cards are not on `spidev`. `rak5148` is a 2.4 GHz SX1280 USB module and stops at startup with that reason.

LoRaWAN 2.4 GHz (`ISM2400`) is recognized: channels 2403 / 2425 / 2479 MHz, DR0–DR7 = SF12–SF5 at 812 kHz. A downlink in that band is answered `TX_FREQ`. The SX1250 on this HAL cannot tune it.

`US915` lists all 72 RP002 channels: 64 uplinks at 902.3–914.9 MHz step 200 kHz, 8 uplinks at 903.0–914.2 MHz step 1.6 MHz, and 8 downlinks at 923.3–927.5 MHz step 600 kHz. The concentrator listens to one sub-band of eight 125 kHz channels plus the matching 500 kHz uplink. `sx1302-cli start --region US915` loads sub-band 2; `--subband 0`–`7` picks another. The daemon accepts a US915 downlink. The default config stays AS923-1.

`FCC-800` is the IotaComm FCC Part 90 block at 809.0–817.0 MHz. The channel list is the Part 90 12.5 kHz raster, 809.0125–816.9875 MHz (639 channels). Downlink is 45 MHz higher, 854.0125–861.9875 MHz, which matches the FSK range in the TEKTELIC gateway's FCC exhibit. That exhibit calls the 800 MHz radio FSK, not LoRa. This project adds its own LoRa plan for that block (it is not a LoRa Alliance or IotaComm plan): 40 uplinks of 125 kHz at 809.1–816.9 MHz step 200 kHz, in five sub-bands of eight, each with one 500 kHz uplink at 809.8 MHz + n × 1.6 MHz. The gateway answers 45 MHz above the uplink, in 854–862 MHz; a downlink inside 809–817 MHz is answered `TX_FREQ`. `sx1302-cli start --region FCC-800 --subband 0`–`4` loads one sub-band. Semtech's `sx1250_calibrate` has no row for 809 MHz, so the image-calibration bytes (`0xCA`, `0xCD`) come from the 4 MHz-step formula in Semtech's `sx126x_driver`. None of this has been on the air.

Treat the FCC-800 plan as a placeholder. I do not know this band, and the channel centres, the 45 MHz pairing for LoRa and the transmit power still need a review by someone who does. Do not transmit with it until that has happened.

The daemon config takes `"region"` and `"subband"` in place of a full `"plan"`. `"hop"` makes one concentrator listen to several blocks in turn, for example all eight US915 sub-bands: `"region": "US915", "subband": 0, "hop": {"dwell_s": 20, "step_hz": 1600000, "steps": 8}`. AS923-1 has room for two blocks in 920–925 MHz: `"hop": {"dwell_s": 20, "shift_hz": [0, -1600000]}`. Each hop is a GPIO reset and a full start, and uplinks on the other blocks are missed while the gateway sits on one.

`"dwell_ms"` and `"duty_cycle_percent"` (or `--dwell-ms` / `--duty-cycle-percent`) limit transmit air time. Both are off when absent or 0. A downlink refused for duty cycle is answered `DUTY_CYCLE_OVERFLOW` (a ChirpStack `TxAckStatus` name, not in Semtech's PROTOCOL.md). A downlink refused for dwell time is answered `COLLISION_PACKET`, because neither list has a dwell code.

`"servers"` adds more network servers next to `"server"`, for example `"servers": ["eu1.cloud.thethings.network:1700"]`. Every uplink and `stat` goes to each one, and each may send downlinks. The TX_ACK goes back to the server that asked, and all of them share one JIT queue. Each server must know the gateway EUI.

When a server stops answering (no PUSH_ACK for 5 s, or no ACK of any kind for 30 s), its uplinks are kept, up to `"uplink_buffer"` PUSH_DATA messages per server (default 2000, `0` drops them like `lora_pkt_fwd`). The oldest go first when the buffer is full. Once the server answers again, the backlog goes out in receive order, 20 messages a second, before any new uplink. Each late `rxpk` carries a `time` field from when it was received. JoinRequest and Rejoin-Request are never kept, because a late one could make the server start a session the device never sees. Downlinks for late uplinks come too late and are answered `TOO_LATE`. The buffer is in memory, so a daemon restart empties it.

`"metrics": "0.0.0.0:9302"` (or `--metrics`) serves Prometheus text at `/metrics`. It covers RX/TX counters, refused downlinks by reason, the JIT queue, board temperature, the time source, and, per server, link state, ACK ratio, outages, backlog, replayed and dropped uplinks. It is off when absent.

There is no TX on radio B. On the SenseCAP M1, as in every Semtech SX1250 reference config, `radio_1` has `tx_enable: false`. Only radio A is wired to the PA.

The GWMP `stat` object adds `temp` (STTS751, degrees C), `time_src` (`pps` or `host`), and `coll` (downlinks dropped because the transmit queue overlapped). A RAK5146 turns on SX1303 fine timestamp. Semtech's AGC v10 image is the firmware for both the SX1302 and the SX1303 when the front-end is an SX1250.

Two things that cost me time:

- **Radio B must be in differential input mode.** In single-input mode it
  still decodes, but about 40 dB deaf.
- The old hard-coded TX power sat above the top of Semtech's gain table. That
  caused CRC errors at short range. The gain table fixed it.

## Compared with Semtech's C HAL

Semtech's [`sx1302_hal`](https://github.com/Lora-net/sx1302_hal) (`libloragw`
plus `lora_pkt_fwd`) is the reference this code was ported from, and it covers
far more hardware. This table is against v2.1.0.

| | Semtech `sx1302_hal` (C) | This repo (Rust) |
|---|---|---|
| Language | C library, C packet forwarder | Rust, no C and no FFI at run time |
| Host link | SPI, and USB through the STM32 on USB modules | SPI (`spidev`) only |
| Radios | SX1250, SX1255, SX1257 | SX1250 only |
| Boards | Semtech reference designs, set in `global_conf.json` | SenseCAP M1 (tested); RAK2287 and RAK5146 SPI reset paths (not booted) |
| Regions | Whatever the radios and the config allow; example configs for EU868, US915, AS923, CN490 and more | AS923-1 tested on air. US915 in code. FCC-800 provisional (see above) |
| Config | `global_conf.json` | Its own smaller JSON; a `global_conf.json` will not load |
| RX: 8 multi-SF, LoRa service, FSK | Yes | Yes, same register setup |
| TX: LoRa and FSK, immediate or timestamped | Yes, on either radio chain | Yes, chain A only |
| TX power table | `tx_gain_lut` | Same table and selection rule |
| RX timestamp correction, RSSI temperature compensation | Yes | Ported, same formulas |
| Network protocol | Semtech UDP (GWMP) | Semtech UDP (GWMP) |
| Downlink queue | JIT queue | JIT queue with the same error codes |
| Class B beacon | Yes, GPS-timed | AS923-1 beacon in code, never transmitted (no GPS on my unit) |
| GPS | u-blox UBX and NMEA, time sync, crystal error correction | NMEA RMC plus the PPS latch. No UBX, no crystal correction |
| Fine timestamp (SX1303) | Yes | Ported, unit-tested, never run on an SX1303 |
| LBT and spectral scan (SX1261) | Yes | No |
| Full duplex (CN490 reference design) | Yes | No |
| Dwell-time and duty-cycle limits on TX | No | Optional, off by default |
| More than one network server | No, one `server_address` | Yes (`"servers"`) |
| Uplinks kept while the server is unreachable | No, dropped | Yes, in memory, data frames only |
| Metrics endpoint | No | Prometheus text, optional |
| Hopping one concentrator over several channel blocks | No | Optional (`"hop"`) |
| Setting the host clock and RTC from GPS | No | Yes, when a GPS port is configured |
| Service supervision | Left to the integrator | systemd unit with a watchdog and a chip health check |
| Test utilities | `test_loragw_*`, `util_chip_id`, `util_net_downlink`, `util_spectral_scan` | `sx1302-cli`, a fake network server, two T-Beam node firmwares |

What I measured with both on the same SenseCAP M1, same channel plan and the
same T-Beam test node:

- Downlink start times agree within 0.01 ms for SF7–SF12, BW250 and FSK.
- RSSI is the same for LoRa and for FSK.
- `lora_pkt_fwd` also forwards weak copies of a frame heard on neighbouring
  channels. This daemon drops those duplicates.
- The Rust daemon sat at about 2.6 MB of resident memory. I did not measure
  `lora_pkt_fwd`, so that is not a comparison.

When to use which: if your gateway is a Semtech reference design, a USB
module, anything with SX1255/SX1257 radios, or needs LBT, use the C HAL. It
is the tested and supported one. This repo is for people who want a small
Rust code base on an SX1302 + SX1250 SPI board and are ready to check it on
their own hardware.

## Building

On the Pi (Rust 1.85 or newer):

```bash
source "$HOME/.cargo/env"
cargo test --workspace
cargo build --release -p sx1302-cli -p sx1302-daemon
```

## Trying it by hand

```bash
sudo ./target/release/sx1302-cli gpio reset
sudo ./target/release/sx1302-cli detect
sudo ./target/release/sx1302-cli start --region AS923-1
sudo ./target/release/sx1302-cli rx --seconds 10
sudo ./target/release/sx1302-cli tx-pkt --region AS923-1 --payload 47573031
```

## Running it as a gateway

```bash
sudo install -m755 target/release/sx1302-daemon target/release/sx1302-cli /usr/local/bin/
sudo install -m644 configs/sx1302-daemon.json /etc/sx1302-daemon.json
echo 'SX1302_SERVER=your-lns-ttn-or-chirpstack:1700' | sudo tee /etc/default/sx1302-daemon
sudo install -m644 systemd/sx1302-daemon.service /etc/systemd/system/
sudo systemctl daemon-reload && sudo systemctl enable --now sx1302-daemon

systemctl status sx1302-daemon      # status line shows server link, queue, board temperature
journalctl -u sx1302-daemon -f
```

[`configs/sx1302-daemon.json`](configs/sx1302-daemon.json) holds the channel
plan: radio centre frequencies, the 8 multi-SF channels, the service and FSK
channels, and optionally `tx_gain_lut`, `rssi_offset` and `rssi_tcomp`. The
default plan matches ChirpStack's `as923` region (923.2/923.4 MHz plus
922.6–924.0 MHz).

The daemon holds the SPI bus. Stop the service before using `sx1302-cli`.

## Test tools

- [`tools/tbeam-as923-node`](tools/tbeam-as923-node): T-Beam firmware that
  cycles through both radios, SF7–12, BW250 and FSK, then prints when each
  downlink actually arrived.
- [`tools/gwmp_echo_ns.py`](tools/gwmp_echo_ns.py): a tiny fake network server
  that answers each test uplink at +1 s / +2 s.
- [`tools/tbeam-otaa-node`](tools/tbeam-otaa-node): a real OTAA node (RadioLib).
  Put your keys in `src/keys.h` (git-ignored). `OTAA_APP_KEY` alone is LoRaWAN 1.0.x.
  Add a different `OTAA_NWK_KEY` for 1.1. The gateway forwards the PHY unchanged,
  including 1.1 Rejoin-Request. LoRaWAN 1.2 has no public L2 spec; a non-zero
  Major is still forwarded.
- [`tools/semtech-ref`](tools/semtech-ref): how to build and run Semtech's
  `lora_pkt_fwd` on the same board for an A/B comparison.

## Layout

| Crate | What's in it |
|---|---|
| `sx1302-registers` | Semtech's full register table (1044 fields) and frequency helpers |
| `sx1302-transport` | `spidev` access and the SX1302 SPI mux protocol |
| `sx1302-hal` | Start-up, channel plan, RX parsing, TX, timestamps, LEDs |
| `sx1302-radio` | SX1250 commands |
| `sx1302-firmware` | AGC/ARB firmware blobs |
| `sensecap-board` | M1 GPIO reset and the STTS751 sensor |
| `sx126x-radio`, `sensecap-*` | Aux radio, LED and secure-element bits |
| `apps/` | `sx1302-cli`, `sx1302-daemon`, `sx1302-bench` |

After bumping the Semtech version, regenerate the register table with
`python tools/gen_loregs.py`.

## Credits and license

Written by [@hereismeaw](https://github.com/hereismeaw) (Kittiwut Khongkaeo).
See [CONTRIBUTORS.md](CONTRIBUTORS.md).

The Rust code is MIT OR Apache-2.0. Semtech's firmware and register table stay
under their Revised BSD license (see `NOTICE` and `firmware/`).
