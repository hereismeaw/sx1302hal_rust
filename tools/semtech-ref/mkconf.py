#!/usr/bin/env python3
"""Make a Semtech lora_pkt_fwd global_conf.json matching configs/sx1302-daemon.json."""
import json
import re
import sys

tmpl_path, plan_path, out_path = sys.argv[1:4]
# Semtech templates carry /* */ comments
g = json.loads(re.sub(r"/\*.*?\*/", "", open(tmpl_path).read(), flags=re.S))
plan = json.load(open(plan_path))["plan"]
c = g["SX130x_conf"]
c["com_type"] = "SPI"
c["com_path"] = "/dev/spidev0.0"
c["lorawan_public"] = True
c["clksrc"] = 0
c["full_duplex"] = False
c.pop("fine_timestamp", None)
for k in ("sx1261_conf",):
    if k in c:
        c[k].setdefault("spectral_scan", {})["enable"] = False
        c[k].setdefault("lbt", {})["enable"] = False
        c[k]["enable"] = False
ra, rb = plan["radio_hz"]
c["radio_0"].update({"enable": True, "freq": ra, "tx_enable": True,
                     "tx_freq_min": 920000000, "tx_freq_max": 925000000,
                     "single_input_mode": False})
c["radio_1"].update({"enable": True, "freq": rb, "tx_enable": False,
                     "single_input_mode": False})
for i in range(8):
    ch = plan["multisf"][i]
    c[f"chan_multiSF_{i}"] = {"enable": True, "radio": ch["radio"],
                              "if": ch["freq_hz"] - plan["radio_hz"][ch["radio"]]}
s = plan["lora_std"]
c["chan_Lora_std"] = {"enable": True, "radio": s["radio"],
                      "if": s["freq_hz"] - plan["radio_hz"][s["radio"]],
                      "bandwidth": s["bw_hz"], "spread_factor": s["sf"],
                      "implicit_hdr": False, "implicit_payload_length": 17,
                      "implicit_crc_en": False, "implicit_coderate": 1}
f = plan["fsk"]
c["chan_FSK"] = {"enable": True, "radio": f["radio"],
                 "if": f["freq_hz"] - plan["radio_hz"][f["radio"]],
                 "bandwidth": 125000, "datarate": f["datarate"]}
gw = g["gateway_conf"]
gw.update({"gateway_ID": "0016C001F106659F", "server_address": "127.0.0.1",
           "serv_port_up": 1700, "serv_port_down": 1700,
           "keepalive_interval": 5, "stat_interval": 30, "push_timeout_ms": 100,
           "forward_crc_valid": True, "forward_crc_error": False,
           "forward_crc_disabled": False})
for k in ("gps_tty_path", "ref_latitude", "ref_longitude", "ref_altitude", "beacon_period"):
    gw.pop(k, None)
g.pop("debug_conf", None)
json.dump(g, open(out_path, "w"), indent=2)
print("radio", ra, rb, "chans", [c[f"chan_multiSF_{i}"]["if"] for i in range(8)],
      "std", c["chan_Lora_std"]["if"], "fsk", c["chan_FSK"]["if"])
