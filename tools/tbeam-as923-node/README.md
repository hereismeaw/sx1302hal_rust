# T-Beam AS923-1 node (COM4)

LilyGO T-Beam v1.1 (ESP32 + SX1276 + AXP192). Flashed over Silicon Labs CP210x **COM4**.

| Param | Value |
|---|---|
| Frequency | 923.2 MHz (AS923-1) |
| SF / BW / CR | 7 / 125 kHz / 4/5 |
| Sync | `0x34` (LoRaWAN public) |
| IQ | normal (`invertIQ=false`) |
| Payload | `TBEAM` + seq (12 bytes) every 2 s |

```bash
cd tools/tbeam-as923-node
pio run -t upload
pio device monitor -p COM4 -b 115200
```

Expect:

```
tbeam-as923-node 923.2 SF7 BW125 sync=0x34
AXP192 LDO2 (LoRa) on
radio ready, TX every 2s IQ=normal
tx seq=0 st=0
```

`st=0` is RadioLib success. Gateway `sx1302-cli start` then `rx` to decode.
