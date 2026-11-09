# sx1302hal_rust

A LoRa concentrator HAL for the Semtech SX1302, written in Rust from scratch.
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

- **No GPS on this carrier**, so there's no PPS, no `tmms` scheduling and no
  Class B.
- **Fine timestamps** are SX1303-only. This unit is an SX1302.
- **LBT / spectral scan** through the SX1262 isn't wired up. Its chip select
  shares SPI0 CE0 on the M1, so it needs a device-tree overlay first. Thailand
  doesn't require LBT.
- **Only AS923-1 is set up end to end.** The plan format works for other
  regions, but nobody has tested one.
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
| Pairing button | BCM 27 (active high) |
| Temperature sensor | STTS751, I2C-1 `0x39` |
| HAT LEDs | SX1302 GPIO0 (Config), GPIO2 (TX), GPIO4 (RX) |
| **BCM 13** | AD5338R DAC: **never driven** by this code |

Two things that cost me time:

- **Radio B must be in differential input mode.** In single-input mode it
  still decodes, but about 40 dB deaf.
- The old hard-coded TX power sat above the top of Semtech's gain table. That
  caused CRC errors at short range. The gain table fixed it.

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
echo 'SX1302_SERVER=your-chirpstack:1700' | sudo tee /etc/default/sx1302-daemon
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
- [`tools/tbeam-otaa-node`](tools/tbeam-otaa-node): a real LoRaWAN 1.0.3 OTAA
  node (RadioLib). Put your keys in `src/keys.h`; that file is git-ignored.
- [`tools/semtech-ref`](tools/semtech-ref): how to build and run Semtech's
  `lora_pkt_fwd` on the same board for an A/B comparison.

[`docs/HANDOVER.md`](docs/HANDOVER.md) has the detailed notes and the
measurements behind the claims above.

## Layout

| Crate | What's in it |
|---|---|
| `sx1302-registers` | Semtech's full register table (1044 fields) and frequency helpers |
| `sx1302-transport` | `spidev` access and the SX1302 SPI mux protocol |
| `sx1302-hal` | Start-up, channel plan, RX parsing, TX, timestamps, LEDs |
| `sx1302-radio` | SX1250 commands |
| `sx1302-firmware` | AGC/ARB firmware blobs |
| `sensecap-board` | M1 GPIO reset and the STTS751 sensor |
| `sx126x-radio`, `sensecap-*` | Aux radio, button, LED and secure-element bits |
| `apps/` | `sx1302-cli`, `sx1302-daemon`, `sx1302-bench` |

After bumping the Semtech version, regenerate the register table with
`python tools/gen_loregs.py`.

## Credits and license

Written by [@hereismeaw](https://github.com/hereismeaw) (Kittiwut Khongkaeo).
See [CONTRIBUTORS.md](CONTRIBUTORS.md).

The Rust code is MIT OR Apache-2.0. Semtech's firmware and register table stay
under their Revised BSD license (see `NOTICE` and `firmware/`).
