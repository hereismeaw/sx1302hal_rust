//! Hardware-in-the-loop helpers. Tests that need SPI are ignored on non-Linux.

#[cfg(test)]
mod tests {
    use sx1302_transport::{encode_read, encode_write};
    use sx1302_core::SpiMuxTarget;

    #[test]
    fn burst_header_stable() {
        let w = encode_write(SpiMuxTarget::RadioA, 0x0008, &[1, 2, 3]);
        assert_eq!(w[0], 0x01);
        assert_eq!(&w[3..], &[1, 2, 3]);
        let r = encode_read(SpiMuxTarget::RadioB, 0x0008, 3);
        assert_eq!(r[0], 0x02);
        assert_eq!(r.len(), 7);
    }
}
