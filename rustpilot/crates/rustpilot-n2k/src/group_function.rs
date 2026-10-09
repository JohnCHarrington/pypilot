//! PGN 126208 group functions: parse Commands addressed to RustPilot and
//! build the Acknowledge every Command must get (`docs/n2k-control.md` §3.2
//! and §5 on the n2k branch).

use canboat::{DecodedField, EncodeError, EncodeValue, FieldValue, Frame};

use crate::{PGN_GROUP_FUNCTION, database};

/// 126208 function codes.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum FunctionCode {
    /// Request.
    Request = 0,
    /// Command.
    Command = 1,
    /// Acknowledge.
    Acknowledge = 2,
    /// Read Fields.
    ReadFields = 3,
    /// Read Fields Reply.
    ReadFieldsReply = 4,
    /// Write Fields.
    WriteFields = 5,
    /// Write Fields Reply.
    WriteFieldsReply = 6,
}

/// Whole-PGN result in an Acknowledge (canboat lookup `PGN_ERROR_CODE`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum PgnError {
    /// Accepted.
    Acknowledge = 0,
    /// The target PGN is not supported.
    PgnNotSupported = 1,
    /// The target PGN is temporarily not available.
    PgnNotAvailable = 2,
    /// Access denied (for RustPilot: below the link's control level).
    AccessDenied = 3,
    /// Request or command not supported.
    NotSupported = 4,
    /// Definer tag not supported.
    TagNotSupported = 5,
    /// Read or write not supported.
    ReadOrWriteNotSupported = 6,
}

/// Per-parameter result in an Acknowledge (canboat lookup
/// `PARAMETER_FIELD`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum ParamError {
    /// Accepted.
    Acknowledge = 0,
    /// No such field in the target PGN.
    InvalidField = 1,
    /// Temporarily unavailable, e.g. nav mode with no route.
    TemporaryError = 2,
    /// Value out of range.
    OutOfRange = 3,
    /// Access denied.
    AccessDenied = 4,
    /// Not supported, e.g. a True heading reference.
    NotSupported = 5,
    /// The field can't be written ("cannot set").
    ReadOrWriteNotSupported = 6,
}

/// One parameter of a Command: the target PGN's 1-based field number and
/// its value, decoded by canboat against that field's type and units
/// (degrees for angles, a raw integer for lookups, text for strings).
#[derive(Debug, Clone)]
pub struct Parameter {
    /// 1-based field number in the target PGN.
    pub field: u8,
    /// The decoded value; `None` when the sender wrote "not available".
    pub value: Option<FieldValue>,
}

impl Parameter {
    /// The value as a number (scaled numeric fields, plain integers and
    /// lookups' raw values).
    pub fn as_f64(&self) -> Option<f64> {
        match self.value.as_ref()? {
            FieldValue::Number(n) | FieldValue::Float(n) => Some(*n),
            FieldValue::Integer(i) => Some(*i as f64),
            FieldValue::Lookup { value, .. } => Some(*value as f64),
            _ => None,
        }
    }

    /// The value as text (STRING_LAU fields such as 126998's descriptions).
    pub fn as_str(&self) -> Option<&str> {
        match self.value.as_ref()? {
            FieldValue::String(s) => Some(s),
            _ => None,
        }
    }
}

/// A decoded 126208 Command.
#[derive(Debug, Clone)]
pub struct Command {
    /// Sender's address, where the Acknowledge goes.
    pub src: u8,
    /// Target PGN.
    pub pgn: u32,
    /// Parameters in the order they were sent.
    pub params: Vec<Parameter>,
}

/// Why a 126208 frame is not a usable Command.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum ParseError {
    /// Not PGN 126208.
    NotGroupFunction,
    /// A group function other than Command; carries its function code.
    NotCommand(u8),
    /// canboat could not decode the payload (truncated, unknown target
    /// field).
    Malformed,
}

/// Parse a reassembled 126208 frame as a Command.
pub fn parse_command(frame: &Frame) -> Result<Command, ParseError> {
    if frame.pgn != PGN_GROUP_FUNCTION {
        return Err(ParseError::NotGroupFunction);
    }
    match frame.data.first() {
        Some(&1) => {}
        Some(&fc) => return Err(ParseError::NotCommand(fc)),
        None => return Err(ParseError::Malformed),
    }
    let decoded = database()
        .decode(frame)
        .map_err(|_| ParseError::Malformed)?;
    if decoded.id != "nmeaCommandGroupFunction" {
        return Err(ParseError::Malformed);
    }

    let mut pgn = None;
    let mut count = None;
    let mut params = Vec::new();
    let mut pending_field: Option<u8> = None;
    for f in &decoded.fields {
        match (f.info.id, f.repeat_index) {
            ("pgn", None) => pgn = pgn_of(f),
            ("numberOfParameters", None) => count = int_of(f),
            ("parameter", Some(_)) => pending_field = int_of(f).and_then(|i| u8::try_from(i).ok()),
            ("value", Some(_)) => {
                let field = pending_field.take().ok_or(ParseError::Malformed)?;
                // canboat zero-pads a value cut short by the end of the
                // payload; a Command with a truncated value must be refused.
                if let (Some(off), Some(len)) = (f.bit_offset, f.bit_length)
                    && (off + len) as usize > frame.data.len() * 8
                {
                    return Err(ParseError::Malformed);
                }
                let value = match &f.value {
                    FieldValue::NotAvailable | FieldValue::Reserved { .. } => None,
                    v => Some(v.clone()),
                };
                params.push(Parameter { field, value });
            }
            _ => {}
        }
    }
    let pgn = pgn.ok_or(ParseError::Malformed)?;
    if count.is_some_and(|c| c as usize != params.len()) {
        return Err(ParseError::Malformed);
    }
    Ok(Command {
        src: frame.src,
        pgn,
        params,
    })
}

/// Build the 126208 Acknowledge for a Command from `dst`, sent from our
/// address `src`, with one error code per parameter in the Command's order.
pub fn build_ack(
    src: u8,
    dst: u8,
    pgn: u32,
    pgn_error: PgnError,
    param_errors: &[ParamError],
) -> Result<Frame, EncodeError> {
    let mut b = database().encode("nmeaAcknowledgeGroupFunction")?;
    b.push_by_name("PGN", EncodeValue::Int(pgn.into()))?;
    b.push_by_name("PGN error code", EncodeValue::Int(pgn_error as i64))?;
    b.push_by_name(
        "Transmission interval/Priority error code",
        EncodeValue::Int(0),
    )?;
    for e in param_errors {
        let i = b.add_set_instance(1)?;
        b.push_in_set(1, i, "Parameter", EncodeValue::Int(*e as i64))?;
    }
    b.source(src).destination(dst).priority(3).build()
}

fn int_of(f: &DecodedField) -> Option<i64> {
    match f.value {
        FieldValue::Integer(i) => Some(i),
        FieldValue::Lookup { value, .. } => i64::try_from(value).ok(),
        _ => None,
    }
}

fn pgn_of(f: &DecodedField) -> Option<u32> {
    match f.value {
        FieldValue::Pgn { value, .. } => Some(value),
        _ => int_of(f).and_then(|i| u32::try_from(i).ok()),
    }
}

#[cfg(test)]
mod tests {
    //! Payloads mirror tests/test_n2k_control.py on the n2k branch.
    use super::*;
    use crate::{PGN_CONFIGURATION_INFORMATION, PGN_HEADING_TRACK_CONTROL};

    fn command_payload(pgn: u32, params: &[(u8, &[u8])]) -> Vec<u8> {
        let mut p = vec![1u8];
        p.extend_from_slice(&pgn.to_le_bytes()[..3]);
        p.extend_from_slice(&[0xf8, params.len() as u8]);
        for (index, value) in params {
            p.push(*index);
            p.extend_from_slice(value);
        }
        p
    }

    fn u16_rad(radians: f64) -> [u8; 2] {
        ((radians / 0.0001).round() as u16).to_le_bytes()
    }

    fn frame(payload: Vec<u8>) -> Frame {
        Frame::new(None, 3, PGN_GROUP_FUNCTION, 9, 35, payload)
    }

    #[test]
    fn multiple_parameters() {
        let hdg = u16_rad(core::f64::consts::FRAC_PI_2);
        let p = command_payload(
            PGN_HEADING_TRACK_CONTROL,
            &[(5, &[4]), (7, &[1]), (11, &hdg)],
        );
        let cmd = parse_command(&frame(p)).unwrap();
        assert_eq!(cmd.pgn, PGN_HEADING_TRACK_CONTROL);
        assert_eq!(cmd.src, 9);
        let fields: Vec<u8> = cmd.params.iter().map(|p| p.field).collect();
        assert_eq!(fields, [5, 7, 11]);
        assert_eq!(cmd.params[0].as_f64(), Some(4.0)); // Heading Control
        assert_eq!(cmd.params[1].as_f64(), Some(1.0)); // Magnetic
        assert!((cmd.params[2].as_f64().unwrap() - 90.0).abs() < 0.01); // degrees
    }

    #[test]
    fn signed_and_not_available() {
        let rudder = (-1000i16).to_le_bytes();
        let p = command_payload(
            PGN_HEADING_TRACK_CONTROL,
            &[(10, &rudder), (11, &[0xff, 0xff])],
        );
        let cmd = parse_command(&frame(p)).unwrap();
        let rudder_deg = cmd.params[0].as_f64().unwrap();
        assert!((rudder_deg - (-0.1f64).to_degrees()).abs() < 0.01);
        assert!(cmd.params[1].value.is_none());
    }

    #[test]
    fn text_command() {
        let mut lau = vec![b"PP:ap.mode".len() as u8 + 2, 1];
        lau.extend_from_slice(b"PP:ap.mode");
        let p = command_payload(PGN_CONFIGURATION_INFORMATION, &[(1, &lau)]);
        let cmd = parse_command(&frame(p)).unwrap();
        assert_eq!(cmd.params[0].as_str(), Some("PP:ap.mode"));
    }

    #[test]
    fn truncated_is_malformed() {
        let p = command_payload(PGN_HEADING_TRACK_CONTROL, &[(11, &[0x01])]);
        assert_eq!(parse_command(&frame(p)).unwrap_err(), ParseError::Malformed);
    }

    #[test]
    fn other_function_codes() {
        let mut p = command_payload(PGN_HEADING_TRACK_CONTROL, &[]);
        p[0] = 0;
        assert_eq!(
            parse_command(&frame(p)).unwrap_err(),
            ParseError::NotCommand(0)
        );
    }

    #[test]
    fn ack_round_trip() {
        let f = build_ack(
            35,
            9,
            PGN_HEADING_TRACK_CONTROL,
            PgnError::Acknowledge,
            &[ParamError::Acknowledge, ParamError::NotSupported],
        )
        .unwrap();
        assert_eq!(
            (f.pgn, f.src, f.dst, f.prio),
            (PGN_GROUP_FUNCTION, 35, 9, 3)
        );
        // fc=2, PGN 127237 LE, both error nibbles 0, 2 params, 4 bits each.
        assert_eq!(
            f.data.as_slice(),
            &[0x02, 0x05, 0xf1, 0x01, 0x00, 0x02, 0x50]
        );
        let d = database().decode(&f).unwrap();
        assert_eq!(d.id, "nmeaAcknowledgeGroupFunction");
    }
}
