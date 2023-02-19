//! All device constants used in the driver, mostly register addresses.
//!
//! NOTE: Earlier revisions of Datasheet and Register Map has way more info about interrupt usage,
//! particularly rev 3.2
//!
//! #### Sources:
//! * Register map (rev 3.2): https://arduino.ua/docs/RM-MPU-6000A.pdf
//! * Datasheet (rev 3.2): https://www.cdiweb.com/datasheets/invensense/ps-mpu-6000a.pdf


/// Gyro Sensitivity
///
/// Measurements are scaled like this:
/// x * range/2**(resolution-1) or x / (2**(resolution-1) / range)
///
/// Sources:
///     * https://www.nxp.com/docs/en/application-note/AN3461.pdf
///     * https://theccontinuum.com/2012/09/24/arduino-imu-pitch-roll-from-accelerometer/
///     * https://makersportal.com/blog/2019/8/17/arduino-mpu6050-high-frequency-accelerometer-and-gyroscope-data-saver#accel_test
///     * https://github.com/kriswiner/MPU6050/wiki/2014-Invensense-Developer%27s-Conference
///     * rust MPU9250 driver on github
pub const GYRO_SENS: (f32, f32, f32, f32) = (131., 65.5, 32.8, 16.4);

/// Accelerometer Sensitivity
///
/// Measurements are scaled like this:
///
/// x * range/2**(resolution-1) or x / (2**(resolution-1) / range)
/// Sources:
///     * https://www.nxp.com/docs/en/application-note/AN3461.pdf
///     * https://theccontinuum.com/2012/09/24/arduino-imu-pitch-roll-from-accelerometer/
///     * https://makersportal.com/blog/2019/8/17/arduino-mpu6050-high-frequency-accelerometer-and-gyroscope-data-saver#accel_test
///     * https://github.com/kriswiner/MPU6050/wiki/2014-Invensense-Developer%27s-Conference
///     * rust MPU9250 driver on github
pub const ACCEL_SENS: (f32, f32, f32, f32) = (16384., 8192., 4096., 2048.);
/// Temperature Offset
pub const TEMP_OFFSET: f32 = 25.0;
/// Temperature Sensitivity
pub const TEMP_SENSITIVITY: f32 = 326.8;

/// Motion Threshold Register
pub const MOT_THR: u8 = 0x1F;
/// Motion Duration Detection Register
pub const MOT_DUR: u8 = 0x20;
/// High Byte Register Gyro x orientation
pub const GYRO_REGX_H: u8 = 0x43;
/// High Byte Register Gyro y orientation
pub const GYRO_REGY_H: u8 = 0x45;
/// High Byte Register Gyro z orientation
pub const GYRO_REGZ_H: u8 = 0x47;
/// High Byte Register Calc roll
pub const ACC_REGX_H : u8= 0x3b;
/// High Byte Register Calc pitch
pub const ACC_REGY_H : u8= 0x3d;
/// High Byte Register Calc yaw
pub const ACC_REGZ_H : u8= 0x3f;
/// High Byte Register Temperature
pub const TEMP_OUT_H : u8= 0x41;
/// Slave address of mpu6886
pub const DEFAULT_SLAVE_ADDR: u8 = 0x68;
/// Internal register to check slave addr
pub const WHOAMI: u8 = 0x75;

/// Describes a bit block from bit number 'bit' to 'bit'+'length'
pub struct BitBlock {
    pub bit: u8,
    pub length: u8
}

#[allow(non_camel_case_types)]
#[derive(Copy, Clone, Debug)]
/// Register 26: Configuration (DLPF, External signal)
pub struct CONFIG;

impl CONFIG {
    /// Base Address
    pub const ADDR: u8 = 0x1a;
    /// FIFO_MODE 
    /// When set to ‘1’, when the FIFO is full, additional writes will not be written to FIFO.
    /// When set to ‘0’, when the FIFO is full, additional writes will be written to the FIFO, replacing
    /// the oldest data.
    pub const FIFO_MODE : u8 = 6;
    /// external Frame Synchronisation (FSYNC)
    /// Enables the FSYNC pin data to be sampled.
    /// EXT_SYNC_SET FSYNC BIT LOCATION
    /// 0 function disabled
    /// 1 TEMP_OUT_L[0]
    /// 2 GYRO_XOUT_L[0]
    /// 3 GYRO_YOUT_L[0]
    /// 4 GYRO_ZOUT_L[0]
    /// 5 ACCEL_XOUT_L[0]
    /// 6 ACCEL_YOUT_L[0]
    /// 7 ACCEL_ZOUT_L[0]
    /// FSYNC will be latched to capture short strobes. This will be done such that if FSYNC toggles,
    /// the latched value toggles, but won’t toggle again until the new latched value is captured by
    /// the sample rate strobe.
    pub const EXT_SYNC_SET: BitBlock = BitBlock { bit: 5, length: 3};
    /// Digital Low Pass Filter (DLPF) config
    pub const DLPF_CFG: BitBlock = BitBlock { bit: 2, length: 3};
}

#[allow(non_camel_case_types)]
#[derive(Copy, Clone, Debug)]
/// Register 27: Gyro Config
pub struct GYRO_CONFIG;

impl GYRO_CONFIG {
    pub const ADDR: u8 = 0x1b;
    /// Gyro x axis self test bit
    pub const XG_ST: u8 = 7;
    /// Gyro y axis self test bit
    pub const YG_ST: u8 = 6;
    /// Gyro z axis self test bit
    pub const ZG_ST: u8 = 5;
    /// Gyro Config FS_SEL
    /// Gyro Full Scale Select:
    /// 00 = ±250 dps.
    /// 01= ±500 dps.
    /// 10 = ±1000 dps.
    /// 11 = ±2000 dps
    pub const FS_SEL: BitBlock = BitBlock { bit: 4, length: 2 };
    /// Used to bypass DLPF as shown in Table 16 in datasheet.
    /// The DLPF is configured by DLPF_CFG, when FCHOICE_B [1:0] = 2b’00. The gyroscope and temperature sensor are
    ///filtered according to the value of DLPF_CFG and FCHOICE_B as shown in the table below.
    /// 
    ///                      | GYROSCOPE                | TEMPERATURE SENSOR
    /// FCHOICE_B  DLPF_CFG  3-DB BW  NOISE BW    RATE
    /// <1><0>               (HZ)     (HZ)        (KHZ)    3-DB BW (HZ)
    /// X 1          X        8173     8595.1       32        4000
    /// 1 0          X        3281     3451.0       32        4000
    /// 0 0          0         250      306.6        8        4000
    /// 0 0          1         176      177.0        1         188
    /// 0 0          2          92      108.6        1          98
    /// 0 0          3          41       59.0        1          42
    /// 0 0          4          20       30.5        1          20
    /// 0 0          5          10       15.6        1          10
    /// 0 0          6           5        8.0        1           5
    /// 0 0          7        3281     3451.0        8        4000
    /// Table 16. Configuration
    pub const FCHOICE_B: BitBlock = BitBlock { bit: 1, length: 2 };
}

#[allow(non_camel_case_types)]
#[derive(Copy, Clone, Debug)]
/// Register 28: Accel Config
pub struct ACCEL_CONFIG;

impl ACCEL_CONFIG {
    /// Base Address
    pub const ADDR: u8 = 0x1c;
    /// Accel x axis self test bit
    pub const XA_ST: u8 = 7;
    /// Accel y axis self test bit
    pub const YA_ST: u8 = 6;
    /// Accel z axis self test bit
    pub const ZA_ST: u8 = 5;
    /// Accel Config FS_SEL
    /// Accel full scale select:
    /// :±2g (00), ±4g (01), ±8g (10), ±16g (11
    pub const FS_SEL: BitBlock = BitBlock { bit: 4, length: 2};
}

#[allow(non_camel_case_types)]
#[derive(Copy, Clone, Debug)]
/// Register 55: INT Pin / Bypass Enable Configuration
pub struct INT_PIN_CFG;

impl INT_PIN_CFG {
    /// Base Address
    pub const ADDR: u8 = 0x37;
    /// INT pin logic level
    /// 1 – The logic level for INT/DRDY pin is active low.
    /// 0 – The logic level for INT/DRDY pin is active high.
    pub const INT_LEVEL: u8 = 7;
    /// INT pin config
    /// 1 – INT/DRDY pin is configured as open drain
    /// 0 – INT/DRDY pin is configured as push-pull.
    pub const INT_OPEN: u8 = 6;
    /// Pulse (length)
    /// 1 – INT/DRDY pin level held until interrupt status is cleared.
    /// 0 – INT/DRDY pin indicates interrupt pulse’s width is 50 μs.
    pub const LATCH_INT_EN: u8 = 5;
    /// INT clear conditions
    /// 1 – Interrupt status is cleared if any read operation is performed.
    /// 0 – Interrupt status is cleared only by reading INT_STATUS register
    pub const INT_RD_CLEAR: u8 = 4;
    /// FSYNC PIN logic level
    /// 1 – The logic level for the FSYNC pin as an interupt is active low.
    ///  – The logic level for the FSYNC pin as an interrupt is active high.
    pub const FSYNC_INT_LEVEL: u8 = 3;
    /// FSYNC PIN config
    /// When this bit is equal to 1, the FSYNC pin will trigger an interrupt when it transitions to
    /// the level specified by FSYNC_INT_LEVEL. When this bit is equal to 0, the FSYNC pin is
    /// disabled from causing an interrupt
    pub const FSYNC_INT_EN: u8 = 2;
}

#[allow(non_camel_case_types)]
#[derive(Copy, Clone, Debug)]
/// Register 56: Interrupt Status
pub struct INT_ENABLE;

impl INT_ENABLE {
    /// Base Address
    pub const ADDR: u8 = 0x38;
    /// 1 – Enable WoM interrupt on X-axis accelerometer. Default setting is 0
    pub const WOM_X_INT_EN: u8 = 7;
    /// 1 – Enable WoM interrupt on Y-axis accelerometer. Default setting is 0
    pub const WOM_Y_INT_EN: u8 = 6;
    /// 1 – Enable WoM interrupt on Z-axis accelerometer. Default setting is 0
    pub const WOM_Z_INT_EN: u8 = 5;
    /// Generate iterrupt when FIFO buffer overflow
    /// 0 : disabled
    /// 1: FIFO overflow generates an interrupt
    pub const FIFO_OFLOW_END: u8 = 4;
    /// Gyroscope Drive System Ready interrupt enable
    pub const GDRIVE_INT_EN: u8 = 2;
    /// enables Data Ready interrupt, each time a write operation to all sensor registers completed
    pub const DATA_RDY_EN: u8 = 0;
}

#[allow(non_camel_case_types)]
#[derive(Copy, Clone, Debug)]
/// Register 58: Interrupt Status
pub struct INT_STATUS;

impl INT_STATUS {
    /// Base Address
    pub const ADDR: u8 = 0x3a;
    /// X-axis accelerometer WoM interrupt status. Cleared on Read
    pub const WOM_X_INT: u8 = 7;
    /// Y-axis accelerometer WoM interrupt status. Cleared on Read
    pub const WOM_Y_INT: u8 = 6;
    /// Z-axis accelerometer WoM interrupt status. Cleared on Read
    pub const WOM_Z_INT: u8 = 5;
    /// This bit automatically sets to 1 when a FIFO buffer overflow has been generated. The bit 
    /// clears to 0 after the register has been read.
    pub const FIFO_OFLOW_INT: u8 = 4;
    /// Gyroscope Drive System Ready interrupt.
    pub const GDRIVE_INT: u8 = 2;
    /// Data is ready
    pub const DATA_RDY_INT: u8 = 0;
}



#[allow(non_camel_case_types)]
#[derive(Copy, Clone, Debug)]
/// Register 107: Power Management 1
pub struct PWR_MGMT_1;

impl PWR_MGMT_1 {
    /// Base Address
    pub const ADDR: u8 = 0x6b;
    /// Device Reset bit
    pub const DEVICE_RESET: u8 = 7;
    /// Sleep mode bit (Should be called "Low Power", doesn't actually sleep)
    pub const SLEEP: u8 = 6;
    /// When set to 1, and SLEEP and STANDBY are not set to 1, the chip will cycle between sleep
    /// and taking a single accelerometer sample at a rate determined by SMPLRT_DIV
    pub const CYCLE: u8 = 5;
    /// Gyro standby
    pub const GYRO_STANDBY: u8 = 4;
    /// When set to 1, this bit disables the temperature sensor
    pub const TEMP_DIS: u8 = 3;
    /// Clock Control
    /// 0 Internal 20 MHz oscillator
    /// 1 to t5 all are: Auto selects the best available clock source – PLL if ready, else use the Internal oscillator
    /// 6 Internal 20 MHz oscillator
    /// 7 stop clock
    pub const CLKSEL: BitBlock = BitBlock { bit: 2, length: 3 };
}

#[allow(non_camel_case_types)]
#[derive(Copy, Clone, Debug)]
/// Register 107: Power Management 2
pub struct PWR_MGMT_2;

impl PWR_MGMT_2 {
    /// Base Address
    pub const ADDR: u8 = 0x6c;
    
    /// disable accel axis x
    pub const STBY_XA: u8 = 5;
    /// disable accel axis y
    pub const STBY_YA: u8 = 4;
    /// disable accel axis z
    pub const STBY_ZA: u8 = 3;
    /// disable gyro  axis x
    pub const STBY_XG: u8 = 2;
    /// disable gyro  axis y
    pub const STBY_YG: u8 = 1;
    /// disable gyro  axis z
    pub const STBY_ZG: u8 = 0;
}

// #[allow(non_camel_case_types)]
// #[derive(Copy, Clone, Debug, Eq, PartialEq)]
// /// Wake values
// pub enum LP_WAKE_CTRL {
//     /// 1.25 Hz
//     _1P25 = 0,
//     /// 2.5 Hz
//     _2P5,
//     /// 5 Hz
//     _5,
//     /// 10 Hz
//     _10,
// }

// #[allow(non_camel_case_types)]
// #[derive(Copy, Clone, Debug, Eq, PartialEq)]
// /// Accelerometer High Pass Filter Values
// pub enum ACCEL_HPF {
//     /// Cut off frequency: None
//     _RESET = 0,
//     /// Cut off frequency: 5 Hz
//     _5 = 1,
//     /// Cut off frequency: 2.5 Hz
//     _2P5 = 2,
//     /// Cut off frequency: 1.25 Hz
//     _1P25 = 3,
//     /// Cut off frequency: 0.63 Hz
//     _0P63 = 4,
//     /// When triggered, the filter holds the present sample. The filter output will be the
//     /// difference between the input sample and the held sample
//     _HOLD = 7
// }

// impl From<u8> for ACCEL_HPF {
//     fn from(range: u8) -> Self
//     {
//         match range {
//             0 => ACCEL_HPF::_RESET,
//             1 => ACCEL_HPF::_5,
//             2 => ACCEL_HPF::_2P5,
//             3 => ACCEL_HPF::_1P25,
//             4 => ACCEL_HPF::_0P63,
//             7 => ACCEL_HPF::_HOLD,
//             _ => ACCEL_HPF::_RESET,
//         }
//     }
// }

#[allow(non_camel_case_types)]
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
/// Clock Source Select Values
pub enum CLKSEL {
    /// Internal 20MHz oscillator
    OSCILL = 0,
    /// Auto selects the best available clock source – PLL if ready, else use the Internal oscillator
    AUTOPLL1 = 1,
    /// Auto selects the best available clock source – PLL if ready, else use the Internal oscillator
    AUTOPLL2 = 2,
    /// Auto selects the best available clock source – PLL if ready, else use the Internal oscillator
    AUTOPLL3 = 3,
    /// Auto selects the best available clock source – PLL if ready, else use the Internal oscillator
    AUTOPLL4 = 4,
    /// Auto selects the best available clock source – PLL if ready, else use the Internal oscillator
    AUTOPLL5 = 5,
    /// Internal 20MHz oscillator
    OSCILL6 = 6,
    /// Stops the clock and keeps the timing generator in reset
    STOP = 7,
}

impl From<u8> for CLKSEL {
    fn from(clk: u8) -> Self {
        match clk {
            0 => CLKSEL::OSCILL,
            1 => CLKSEL::AUTOPLL1,
            2 => CLKSEL::AUTOPLL2,
            3 => CLKSEL::AUTOPLL3,
            4 => CLKSEL::AUTOPLL4,
            5 => CLKSEL::AUTOPLL5,
            6 => CLKSEL::OSCILL6,
            7 => CLKSEL::STOP,
            _ => CLKSEL::AUTOPLL1
        }
    }
}

/// Defines accelerometer range/sensivity
#[derive(Debug, Eq, PartialEq, Copy, Clone)]
pub enum AccelRange {
    /// 2G
    G2 = 0,
    /// 4G
    G4,
    /// 8G
    G8,
    /// 16G
    G16,
}

/// Defines gyro range/sensitivity
#[derive(Debug, Eq, PartialEq, Copy, Clone)]
pub enum GyroRange {
    /// 250 degrees
    D250 = 0,
    /// 500 degrees
    D500,
    /// 1000 degrees
    D1000,
    /// 2000 degrees
    D2000,
}

impl From<u8> for GyroRange {
    fn from(range: u8) -> Self
    {
        match range {
            0 => GyroRange::D250,
            1 => GyroRange::D500,
            2 => GyroRange::D1000,
            3 => GyroRange::D2000,
            _ => GyroRange::D250
        }
    }
}

impl From<u8> for AccelRange {
    fn from(range: u8) -> Self
    {
        match range {
            0 => AccelRange::G2,
            1 => AccelRange::G4,
            2 => AccelRange::G8,
            3 => AccelRange::G16,
            _ => AccelRange::G2
        }
    }
}

impl AccelRange {
    // Converts accelerometer range to correction/scaling factor, see register sheet
    pub(crate) fn sensitivity(&self) -> f32 {
        match &self {
            AccelRange::G2 => ACCEL_SENS.0,
            AccelRange::G4 => ACCEL_SENS.1,
            AccelRange::G8 => ACCEL_SENS.2,
            AccelRange::G16 => ACCEL_SENS.3,
        }
    }
}

impl GyroRange {
    // Converts gyro range to correction/scaling factor, see register sheet
    pub(crate) fn sensitivity(&self) -> f32 {
        match &self {
            GyroRange::D250 => GYRO_SENS.0,
            GyroRange::D500 => GYRO_SENS.1,
            GyroRange::D1000 => GYRO_SENS.2,
            GyroRange::D2000 => GYRO_SENS.3,
        }
    }
}
