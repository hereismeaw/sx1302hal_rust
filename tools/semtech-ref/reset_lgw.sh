#!/bin/sh
# SenseCAP M1 reset via sx1302-cli (GPIO18 POWER_EN, GPIO17 RESET, GPIO5 SX1262 RESET; GPIO13 never driven)
[ "$1" = start ] && /usr/local/bin/sx1302-cli gpio reset >/dev/null
exit 0
