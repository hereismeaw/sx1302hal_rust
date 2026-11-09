#!/usr/bin/env python3
"""Generate Rust loregs table from Semtech loragw_reg.c (commit 4b42025)."""
from __future__ import annotations

import re
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
SRC = Path(
    r"C:\Users\meawk\.grok\sessions\d%3A%5CProject%5Csx1302-03_hal_rust"
    r"\01a1bc0e-6422-7f43-b51f-f2dc69611cde\web_fetch\14.txt"
)
OUT_DIR = ROOT / "crates" / "sx1302-registers" / "src"

BASES = {
    "SX1302_REG_EXT_MEM_PAGED_BASE_ADDR": 0x0000,
    "SX1302_REG_RX_BUFFER_BASE_ADDR": 0x4000,
    "SX1302_REG_TX_TOP_A_BASE_ADDR": 0x5200,
    "SX1302_REG_TX_TOP_B_BASE_ADDR": 0x5400,
    "SX1302_REG_COMMON_BASE_ADDR": 0x5600,
    "SX1302_REG_GPIO_BASE_ADDR": 0x5640,
    "SX1302_REG_MBIST_BASE_ADDR": 0x56C0,
    "SX1302_REG_RADIO_FE_BASE_ADDR": 0x5700,
    "SX1302_REG_AGC_MCU_BASE_ADDR": 0x5780,
    "SX1302_REG_CLK_CTRL_BASE_ADDR": 0x57C0,
    "SX1302_REG_RX_TOP_BASE_ADDR": 0x5800,
    "SX1302_REG_RX_TOP_LORA_SERVICE_FSK_BASE_ADDR": 0x5B00,
    "SX1302_REG_CAPTURE_RAM_BASE_ADDR": 0x6000,
    "SX1302_REG_ARB_MCU_BASE_ADDR": 0x6080,
    "SX1302_REG_TIMESTAMP_BASE_ADDR": 0x6100,
    "SX1302_REG_OTP_BASE_ADDR": 0x6180,
}

ROW = re.compile(
    r"\{(-?\d+),\s*([^,]+),\s*(-?\d+),\s*(-?\d+),\s*(-?\d+),\s*(-?\d+),\s*(-?\d+),\s*(-?\d+)\}\s*,?\s*//\s*(\S+)"
)


def eval_addr(expr: str) -> int:
    expr = expr.strip()
    if expr.isdigit() or (expr.startswith("0x") or expr.startswith("0X")):
        return int(expr, 0)
    m = re.match(r"([A-Z0-9_]+)\s*\+\s*(-?\d+)$", expr)
    if m:
        return BASES[m.group(1)] + int(m.group(2))
    if expr in BASES:
        return BASES[expr]
    raise ValueError(expr)


def main() -> None:
    text = SRC.read_text(encoding="utf-8", errors="replace")
    rows = []
    for m in ROW.finditer(text):
        page, addr_e, offs, sign, leng, rdon, chck, dflt, name = m.groups()
        addr = eval_addr(addr_e)
        if name == "0" or (addr == 0 and int(leng) == 0):
            continue
        rows.append(
            {
                "name": name,
                "page": int(page),
                "addr": addr,
                "offs": int(offs),
                "sign": int(sign),
                "leng": int(leng),
                "rdon": int(rdon),
                "chck": int(chck),
                "dflt": int(dflt),
            }
        )
    # drop sentinel if parsed
    if rows and rows[-1]["leng"] == 0 and rows[-1]["addr"] == 0:
        rows.pop()
    assert len(rows) == 1044, f"expected 1044 loregs, got {len(rows)}"

    ids = ["//! Register IDs matching Semtech `loragw_reg.h` (`LGW_TOTALREGS` = 1044).\n", "#![allow(non_upper_case_globals)]\n\n"]
    for i, r in enumerate(rows):
        ids.append(f"pub const {r['name']}: u16 = {i};\n")
    (OUT_DIR / "ids.rs").write_text("".join(ids), encoding="utf-8")

    tbl = [
        "//! `loregs[]` from Semtech `loragw_reg.c` (sx1302_hal 4b42025 / v2.1.0).\n",
        "//! Addresses and bitfields are copied, not invented.\n\n",
        "#[derive(Debug, Clone, Copy)]\n",
        "pub struct Field {\n",
        "    pub name: &'static str,\n",
        "    pub page: i8,\n",
        "    pub addr: u16,\n",
        "    pub offs: u8,\n",
        "    pub signed: bool,\n",
        "    pub leng: u8,\n",
        "    pub read_only: bool,\n",
        "    pub checkable: bool,\n",
        "    pub default: i32,\n",
        "}\n\n",
        "pub const LGW_TOTALREGS: usize = 1044;\n\n",
        "pub static LOREGS: [Field; LGW_TOTALREGS] = [\n",
    ]
    for r in rows:
        tbl.append(
            "    Field { name: \"%s\", page: %d, addr: 0x%04X, offs: %d, signed: %s, leng: %d, read_only: %s, checkable: %s, default: %d },\n"
            % (
                r["name"],
                r["page"],
                r["addr"],
                r["offs"],
                "true" if r["sign"] else "false",
                r["leng"],
                "true" if r["rdon"] else "false",
                "true" if r["chck"] else "false",
                r["dflt"],
            )
        )
    tbl.append("];\n")
    (OUT_DIR / "table.rs").write_text("".join(tbl), encoding="utf-8")
    print("wrote", len(rows), "fields")


if __name__ == "__main__":
    main()
