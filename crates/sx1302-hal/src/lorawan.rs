//! LoRaWAN PHY header (TS001-1.0.4 and TS001-1.1). The gateway does not do MAC.
//! Both revisions use Major = 0. 1.1 adds MType Rejoin-Request. The payload is
//! forwarded whole either way.
//! shortcut: no public LoRaWAN 1.2 L2 spec yet — a non-zero Major is forwarded unchanged.

/// MHDR MType (TS001-1.1 §4.2.1).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MType {
    JoinRequest,
    JoinAccept,
    UnconfirmedDataUp,
    UnconfirmedDataDown,
    ConfirmedDataUp,
    ConfirmedDataDown,
    /// LoRaWAN 1.1 only. 1.0.x has no defined MType 6.
    RejoinRequest,
    Proprietary,
}

/// What the air frame tells a forwarder. 1.0.x and 1.1 are not separable here
/// except for [`MType::RejoinRequest`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MacRev {
    /// Major 0: TS001-1.0.4 or TS001-1.1.
    R1 { rejoin: bool },
    /// Major 1..=3. Reserved until a later L2 spec (1.2 or otherwise) assigns it.
    Unassigned { major: u8 },
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct PhyHdr {
    pub mtype: MType,
    pub major: u8,
}

impl PhyHdr {
    pub fn parse(payload: &[u8]) -> Option<Self> {
        let mhdr = *payload.first()?;
        let mtype = match mhdr >> 5 {
            0 => MType::JoinRequest,
            1 => MType::JoinAccept,
            2 => MType::UnconfirmedDataUp,
            3 => MType::UnconfirmedDataDown,
            4 => MType::ConfirmedDataUp,
            5 => MType::ConfirmedDataDown,
            6 => MType::RejoinRequest,
            _ => MType::Proprietary,
        };
        Some(Self { mtype, major: mhdr & 0b11 })
    }

    pub fn rev(self) -> MacRev {
        if self.major == 0 {
            MacRev::R1 { rejoin: self.mtype == MType::RejoinRequest }
        } else {
            MacRev::Unassigned { major: self.major }
        }
    }

    /// One log token. `None` for ordinary R1 data and join frames.
    pub fn note(self) -> Option<&'static str> {
        match self.rev() {
            MacRev::R1 { rejoin: true } => Some("lorawan 1.1 rejoin"),
            MacRev::Unassigned { .. } => Some("lorawan major unassigned, forwarded as-is"),
            MacRev::R1 { rejoin: false } => None,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn r1_data_and_rejoin_and_future_major() {
        let up = PhyHdr::parse(&[0x40, 0, 0, 0, 0]).unwrap();
        assert_eq!(up.mtype, MType::UnconfirmedDataUp);
        assert_eq!(up.rev(), MacRev::R1 { rejoin: false });
        assert_eq!(up.note(), None);

        let rejoin = PhyHdr::parse(&[0xC0]).unwrap();
        assert_eq!(rejoin.rev(), MacRev::R1 { rejoin: true });
        assert_eq!(rejoin.note(), Some("lorawan 1.1 rejoin"));

        let later = PhyHdr::parse(&[0x41]).unwrap();
        assert_eq!(later.rev(), MacRev::Unassigned { major: 1 });
        assert!(later.note().is_some());
        assert!(PhyHdr::parse(&[]).is_none());
    }
}
