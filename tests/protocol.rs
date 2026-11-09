use sx1302_core::SpiMuxTarget;
use sx1302_transport::{READ_ACCESS, WRITE_ACCESS, encode_read, encode_write};

#[test]
fn semtech_single_write_layout() {
    let f = encode_write(SpiMuxTarget::Sx1302, 0x5606, &[0xAA]);
    assert_eq!(f, vec![0x00, WRITE_ACCESS | 0x56, 0x06, 0xAA]);
}

#[test]
fn semtech_single_read_layout() {
    let f = encode_read(SpiMuxTarget::Sx1302, 0x5606, 1);
    assert_eq!(f[0], 0x00);
    assert_eq!(f[1], READ_ACCESS | 0x56);
    assert_eq!(f[2], 0x06);
    assert_eq!(f[3], 0x00);
    assert_eq!(f.len(), 5);
}
