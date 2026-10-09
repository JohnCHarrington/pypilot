//! CAN transport shared by every NMEA 2000 connector (SocketCAN on Linux,
//! TWAI on ESP32). One bus serves both the N2K data and control halves.

use crate::ConnError;

/// An extended (29-bit) CAN frame, the only kind NMEA 2000 uses.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct CanFrame {
    /// 29-bit identifier: priority, PGN, source and destination.
    pub id: u32,
    /// Up to 8 data bytes.
    pub data: heapless::Vec<u8, 8>,
}

impl CanFrame {
    /// Build a frame; `None` if `id` doesn't fit 29 bits or `data` is longer
    /// than 8 bytes.
    pub fn new(id: u32, data: &[u8]) -> Option<Self> {
        if id >= 1 << 29 {
            return None;
        }
        Some(Self {
            id,
            data: heapless::Vec::from_slice(data).ok()?,
        })
    }
}

/// A CAN bus.
pub trait CanBus {
    /// Queue a frame for transmission.
    async fn send(&mut self, frame: &CanFrame) -> Result<(), ConnError>;

    /// Wait for the next received frame.
    async fn recv(&mut self) -> Result<CanFrame, ConnError>;
}
