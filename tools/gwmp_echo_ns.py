#!/usr/bin/env python3
"""Minimal GWMP v2 "network server" for gateway downlink tests.

ACKs PUSH_DATA / PULL_DATA like ChirpStack's gateway bridge. For every CRC-OK
uplink from tools/tbeam-as923-node (payload b"TB" + case + seq) it answers on
the same frequency / data rate:

* LoRa: two downlinks, tmst + DELAY_US (RX1-like) and tmst + 2*DELAY_US, ipol=true.
  At SF12 the second one overlaps the first and must be refused by the JIT queue.
* FSK: one downlink at tmst + DELAY_US.

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
pending = {}
print(f"echo NS on 127.0.0.1:{PORT}, delay {DELAY_US} us", flush=True)


def send_txpk(txpk, label):
    global dl_token
    dl_token = (dl_token + 1) & 0xFFFF
    pending[dl_token] = label
    sock.sendto(bytes([2]) + dl_token.to_bytes(2, "big") + b"\x03"
                + json.dumps({"txpk": txpk}).encode(), pull_addr)


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
            if pk.get("stat") != 1 or not payload.startswith(b"TB") or pull_addr is None:
                continue
            case = payload[2]
            seq = int.from_bytes(payload[3:5], "big")
            print(f"up case={case} seq={seq} {pk['modu']} {pk['datr']} {pk['freq']} MHz "
                  f"chan={pk['chan']} rfch={pk['rfch']} rssi={pk['rssi']} tmst={pk['tmst']}", flush=True)
            n_dl = 1 if pk["modu"] == "FSK" else 2
            for k in range(1, n_dl + 1):
                dl = b"DL" + bytes([case, k]) + seq.to_bytes(2, "big")
                txpk = {
                    "imme": False,
                    "tmst": (pk["tmst"] + k * DELAY_US) & 0xFFFFFFFF,
                    "freq": pk["freq"],
                    "rfch": 0,
                    "powe": 14,
                    "modu": pk["modu"],
                    "datr": pk["datr"],
                    "size": len(dl),
                    "data": base64.b64encode(dl).decode(),
                }
                if pk["modu"] == "LORA":
                    txpk.update({"codr": "4/5", "ipol": True, "prea": 8})
                else:
                    txpk.update({"fdev": 25000, "prea": 5})
                send_txpk(txpk, f"case={case} seq={seq} dl{k}")
        if "stat" in body:
            print(f"stat {body['stat']}", flush=True)
    elif ident == 0x02:  # PULL_DATA
        pull_addr = addr
        sock.sendto(bytes([2]) + token + b"\x04", addr)
    elif ident == 0x05:  # TX_ACK
        t = int.from_bytes(token, "big")
        err = json.loads(data[12:] or b"{}").get("txpk_ack", {}).get("error", "?")
        print(f"  TX_ACK {pending.pop(t, hex(t))}: {err}", flush=True)
