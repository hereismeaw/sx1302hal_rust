# Semtech reference comparison (test only, not used at runtime)

Runs Semtech `lora_pkt_fwd` (sx1302_hal `4b42025`, v2.1.0) on the same SenseCAP M1
with the same channel plan, so RSSI / timing can be compared against the Rust HAL.

```bash
# on the Pi
mkdir -p ~/ref && cd ~/ref
curl -sL https://github.com/Lora-net/sx1302_hal/archive/4b42025d1751e04632c0b04160e0d29dbbb222a5.tar.gz | tar xz
cd sx1302_hal-*/ && make -j4 && cd packet_forwarder
cp ~/sx1302-hal-rs/tools/semtech-ref/reset_lgw.sh . && chmod +x reset_lgw.sh   # SenseCAP M1 GPIO reset
python3 -I ~/sx1302-hal-rs/tools/semtech-ref/mkconf.py global_conf.json.sx1250.EU868 \
    ~/sx1302-hal-rs/configs/sx1302-daemon.json ~/ref/gc_as923.json
sudo systemctl stop sx1302-daemon
python3 ~/sx1302-hal-rs/tools/gwmp_echo_ns.py 1700 &   # echo NS, T-Beam runs tools/tbeam-as923-node
sudo ./lora_pkt_fwd -c ~/ref/gc_as923.json
sudo systemctl start sx1302-daemon
```

Result 2026-11-09: identical downlink start times (≤ 0.01 ms, SF7–12, BW250, FSK) and
identical RSSI (LoRa −40…−45, FSK −85…−87 dBm) to the Rust HAL.
