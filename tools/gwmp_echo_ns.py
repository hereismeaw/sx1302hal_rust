#!/usr/bin/env python3
"""Minimal GWMP v2 "network server" for gateway downlink tests.

ACKs PUSH_DATA / PULL_DATA like ChirpStack's gateway bridge and, for every CRC-OK
uplink whose payload starts with b"TBEAM", answers with a Class A style RX1
downlink: PULL_RESP txpk { imme:false, tmst: rx_tmst + DELAY_US, ipol:true }.

Usage: python3 gwmp_echo_ns.py [port] [delay_us]
"""
import base64
import json
import socket
import sys

PORT = int(sys.argv[1]) if len(sys.argv) > 1 else 1700
DELAY_US = int(sys.argv[2]) if len(sys.argv) > 2 else 1_000_000

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(("127.0.0.1", PORT))
pull_addr = None
dl_token = 0x1000
print(f"echo NS on 127.0.0.1:{PORT}, RX1 delay {DELAY_US} us", flush=True)

while True:
    data, addr = sock.recvfrom(65535)
    if len(data) < 4 or data[0] != 2:
        continue
    token, ident = data[1:3], data[3]
    if ident == 0x00:  # PUSH_DATA
        sock.sendto(bytes([2]) + token + b"\x01", addr)
        body = json.loads(data[12:])
        for pk in body.get("rxpk", []):
            payload = base64.b64decode(pk["data"])
            if pk.get("stat") != 1 or not payload.startswith(b"TBEAM") or pull_addr is None:
                continue
            seq = int.from_bytes(payload[6:10], "big")
            dl = b"DL" + seq.to_bytes(4, "big")
            txpk = {
                "imme": False,
                "tmst": (pk["tmst"] + DELAY_US) & 0xFFFFFFFF,
                "freq": pk["freq"],
                "rfch": 0,
                "powe": 14,
                "modu": "LORA",
                "datr": pk["datr"],
                "codr": "4/5",
                "ipol": True,
                "prea": 8,
                "size": len(dl),
                "data": base64.b64encode(dl).decode(),
            }
            dl_token = (dl_token + 1) & 0xFFFF
            sock.sendto(bytes([2]) + dl_token.to_bytes(2, "big") + b"\x03"
                        + json.dumps({"txpk": txpk}).encode(), pull_addr)
            print(f"uplink seq={seq} tmst={pk['tmst']} -> PULL_RESP token={dl_token:#06x}", flush=True)
        if "stat" in body:
            print(f"stat {body['stat']}", flush=True)
    elif ident == 0x02:  # PULL_DATA
        pull_addr = addr
        sock.sendto(bytes([2]) + token + b"\x04", addr)
    elif ident == 0x05:  # TX_ACK
        print(f"TX_ACK token={int.from_bytes(token, 'big'):#06x} {data[12:].decode(errors='replace')}", flush=True)
