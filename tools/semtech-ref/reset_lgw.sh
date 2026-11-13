#!/bin/sh
# Select the carrier explicitly for the official Seeed HAT:
# SX1302_BOARD=wm1302-pi-hat ./lora_pkt_fwd ...
if [ "$1" = start ]; then
    exec /usr/local/bin/sx1302-cli --board "${SX1302_BOARD:-sensecap-m1}" gpio reset >/dev/null
fi
exit 0
