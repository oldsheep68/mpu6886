

use crate::error::*;

pub(crate) trait Bitfield {
    const BITMASK: u8;

    /// Bit value of a discriminant, shifted to the correct position if
    /// necessary
    fn bits(self) -> u8;
}
/// Accelareration Filter Bandwith selection values
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum AccelBw {
    /// BW filter bypassed
    Hz1046  = 0b1000,
    /// 180 Hz
    Hz218 = 0b0000,
    /// 121 Hz
    Hz99 = 0b0010,
    /// 73 Hz
    Hz45 = 0b0011,
    /// 53 Hz
    Hz21 = 0b0100,
    /// 34 Hz
    Hz10 = 0b0101,
    /// 25 Hz
    Hz5 = 0b0110,
    /// 16 Hz
    Hz420 = 0b0111,
}


impl AccelBw {
    pub fn as_f32(self) -> f32 {
        use AccelBw::*;

        match self {
            Hz1046 => 1046.0, // filter is bypassed
            Hz218 => 218.1,
            Hz99 => 121.0,
            Hz45 => 44.8,
            Hz21 => 21.2,
            Hz10 => 10.2,
            Hz5 => 5.1,
            Hz420 => 420.0,
        }
    }
}

impl Default for AccelBw {
    fn default() -> Self {
        Self::Hz218
    }
}

impl Bitfield for AccelBw {
    const BITMASK: u8 = 0b0000_1111;

    fn bits(self) -> u8 {
        // `A_DLPF_CFG` occupies bits 2:0 in the register
        // ACCEL_FCHOICE_B  occupies bit 3
        self as u8
    }
}

impl TryFrom<u8> for AccelBw {
    type Error = SensorError;

    fn try_from(value: u8) -> Result<Self, Self::Error> {
        use AccelBw::*;

        match value {
            0b1000 => Ok(Hz1046), // filter is bypassed
            0b0000 => Ok(Hz218),
            0b0010 => Ok(Hz99),
            0b0011 => Ok(Hz45),
            0b0100 => Ok(Hz21),
            0b0101 => Ok(Hz10),
            0b0110 => Ok(Hz5),
            0b0111 => Ok(Hz420),
            _ => Err(SensorError::InvalidDiscriminant),
        }
    }
}
