use sx1302_core::SpiMuxTarget;

/// Semtech `loragw_spi.c`: write bit in address high byte.
pub const WRITE_ACCESS: u8 = 0x80;
pub const READ_ACCESS: u8 = 0x00;

pub fn encode_write(mux: SpiMuxTarget, address: u16, data: &[u8]) -> Vec<u8> {
    let mut out = Vec::with_capacity(3 + data.len());
    out.push(mux.as_u8());
    out.push(WRITE_ACCESS | ((address >> 8) as u8 & 0x7F));
    out.push((address & 0xFF) as u8);
    out.extend_from_slice(data);
    out
}

/// Read command + dummy + payload slots (Semtech single-byte read uses 5 bytes).
pub fn encode_read(mux: SpiMuxTarget, address: u16, payload_len: usize) -> Vec<u8> {
    let mut out = vec![0u8; 4 + payload_len];
    out[0] = mux.as_u8();
    out[1] = READ_ACCESS | ((address >> 8) as u8 & 0x7F);
    out[2] = (address & 0xFF) as u8;
    out[3] = 0x00;
    out
}

/// SX1250 SPI frame (`sx1250_spi.c`): `[mux][opcode][payload…]`.
pub fn encode_radio(mux: SpiMuxTarget, opcode: u8, payload: &[u8]) -> Vec<u8> {
    let mut out = Vec::with_capacity(2 + payload.len());
    out.push(mux.as_u8());
    out.push(opcode);
    out.extend_from_slice(payload);
    out
}
