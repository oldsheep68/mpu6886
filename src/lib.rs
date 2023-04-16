//! # mpu6886 sensor driver.
//!
//! `embedded_hal` based driver with i2c access to mpu6886
//!
//! ### Misc
//! * [Register sheet](https://www.invensense.com/wp-content/uploads/2015/02/MPU-6000-Register-Map1.pdf),
//! * [Data sheet](https://www.invensense.com/wp-content/uploads/2015/02/MPU-6500-Datasheet2.pdf)
//! 
//! To use this driver you must provide a concrete `embedded_hal` implementation.
//! This example uses `linux_embedded_hal`.
//!
//! **More Examples** can be found [here](https://github.com/juliangaal/mpu6886/tree/master/examples).
//! ```no_run
//! use mpu6886::*;
//! use linux_embedded_hal::{I2cdev, Delay};
//! use i2cdev::linux::LinuxI2CError;
//! 
//!
//! fn main() -> Result<(), mpu6886Error<LinuxI2CError>> {
//!     let i2c = I2cdev::new("/dev/i2c-1")
//!         .map_err(mpu6886Error::I2c)?;
//!
//!     let mut delay = Delay;
//!     let mut mpu = mpu6886::new(i2c);
//!
//!     mpu.init(&mut delay)?;
//!
//!     loop {
//!         // get roll and pitch estimate
//!         let acc = mpu.get_acc_angles()?;
//!         println!("r/p: {:?}", acc);
//!
//!         // get sensor temp
//!         let temp = mpu.get_temp()?;
//!         printlnasd!("temp: {:?}c", temp);
//!
//!         // get gyro data, scaled with sensitivity
//!         let gyro = mpu.get_gyro()?;
//!         println!("gyro: {:?}", gyro);
//!
//!         // get accelerometer data, scaled with sensitivity
//!         let acc = mpu.get_acc()?;
//!         println!("acc: {:?}", acc);
//!     }
//! }
//! ```

#![no_std]

mod bits;
pub mod device;
pub mod config;
pub mod error;

use crate::config::*;
use crate::device::*;
use crate::error::*;

use libm::{powf, atan2f, sqrtf};
use nalgebra::{Vector3, Vector2};
use embedded_hal::{
    blocking::delay::DelayMs,
    blocking::i2c::{Write, WriteRead},
};
//use esp_println::println;
/// PI, f32
pub const PI: f32 = core::f32::consts::PI;

/// PI / 180, for conversion to radians
pub const PI_180: f32 = PI / 180.0;
pub const GRAVITY: f32 = 9.806651;

// /// All possible errors in this crate
// #[derive(Debug)]
// pub enum Mpu6886Error<E> {
//     /// I2C bus error
//     I2c(E),

//     /// Invalid chip ID was read
//     InvalidChipId(u8),
// }

/// Handles all operations on/with mpu6886
pub struct Mpu6886<I> {
    i2c: I,
    slave_addr: u8,
    acc_sensitivity: f32,
    gyro_sensitivity: f32,
}

impl<I, E> Mpu6886<I>
where
    I: Write<Error = E> + WriteRead<Error = E>, 
{
    /// Side effect free constructor with default sensitivies, no calibration
    pub fn new(i2c: I) -> Self {
        Mpu6886 {
            i2c,
            slave_addr: DEFAULT_SLAVE_ADDR,
            acc_sensitivity: ACCEL_SENS.0,
            gyro_sensitivity: GYRO_SENS.0,
        }
    }

    /// custom sensitivity
    pub fn new_with_sens(i2c: I, arange: AccelRange, grange: GyroRange) -> Self {
        Mpu6886 {
            i2c,
            slave_addr: DEFAULT_SLAVE_ADDR,
            acc_sensitivity: arange.sensitivity(),
            gyro_sensitivity: grange.sensitivity(),
        }
    }

    /// Same as `new`, but the chip address can be specified (e.g. 0x69, if the A0 pin is pulled up)
    pub fn new_with_addr(i2c: I, slave_addr: u8) -> Self {
        Mpu6886 {
            i2c,
            slave_addr,
            acc_sensitivity: ACCEL_SENS.0,
            gyro_sensitivity: GYRO_SENS.0,
        }
    }

    /// Combination of `new_with_sens` and `new_with_addr`
    pub fn new_with_addr_and_sens(i2c: I, slave_addr: u8, arange: AccelRange, grange: GyroRange) -> Self {
        Mpu6886 {
            i2c,
            slave_addr,
            acc_sensitivity: arange.sensitivity(),
            gyro_sensitivity: grange.sensitivity(),
        }
    }

    /// Wakes mpu6886 with all sensors enabled (default)
    fn wake<D: DelayMs<u8>>(&mut self, delay: &mut D) -> Result<(), Mpu6886Error<E>> {
        // mpu6886 has sleep enabled by default -> set bit 0 to wake
        // Set clock source to be PLL with x-axis gyroscope reference, bits 2:0 = 001 (See Register Map )
        self.write_byte(PWR_MGMT_1::ADDR, 0x01)?;
        delay.delay_ms(100u8);
        Ok(())
    }

    /// From Register map:
    /// "An  internal  8MHz  oscillator,  gyroscope based  clock,or  external  sources  can  be
    /// selected  as the MPU-60X0 clock source.
    /// When the internal 8 MHz oscillator or an external source is chosen as the clock source,
    /// the MPU-60X0 can operate in low power modes with the gyroscopes disabled. Upon power up,
    /// the MPU-60X0clock source defaults to the internal oscillator. However, it is highly
    /// recommended  that  the  device beconfigured  to  use  one  of  the  gyroscopes
    /// (or  an  external  clocksource) as the clock reference for improved stability.
    /// The clock source can be selected according to the following table...."
    pub fn set_clock_source(&mut self, source: CLKSEL) -> Result<(), Mpu6886Error<E>> {
        Ok(self.write_bits(PWR_MGMT_1::ADDR, PWR_MGMT_1::CLKSEL.bit, PWR_MGMT_1::CLKSEL.length, source as u8)?)
    }

    /// get current clock source
    pub fn get_clock_source(&mut self) -> Result<CLKSEL, Mpu6886Error<E>> {
        let source = self.read_bits(PWR_MGMT_1::ADDR, PWR_MGMT_1::CLKSEL.bit, PWR_MGMT_1::CLKSEL.length)?;
        Ok(CLKSEL::from(source))
    }

    /// Init wakes mpu6886 and verifies register addr, e.g. in i2c
    pub fn init<D: DelayMs<u8>>(&mut self, delay: &mut D) -> Result<(), Mpu6886Error<E>> {
        self.wake(delay)?;
        self.verify()?;
        self.set_accel_range(AccelRange::G2)?;
        self.set_gyro_range(GyroRange::D250)?;
        Ok(())
    }

    /// Verifies device to address 0x68 with WHOAMI.addr() Register
    fn verify(&mut self) -> Result<(), Mpu6886Error<E>> {
        let chip_type = self.read_byte(WHOAMI)?;
        if chip_type != 0x19 {
            return Err(Mpu6886Error::InvalidChipId(chip_type));
        }
        Ok(())
    }

    /// setup motion detection
    /// sources:
    /// * https://github.com/kriswiner/mpu6886/blob/a7e0c8ba61a56c5326b2bcd64bc81ab72ee4616b/mpu6886IMU.ino#L486
    /// * https://arduino.stackexchange.com/a/48430
    pub fn setup_motion_detection(&mut self) -> Result<(), Mpu6886Error<E>> {
        self.write_byte(0x6B, 0x00)?;
        // optional? self.write_byte(0x68, 0x07)?; // Reset all internal signal paths in the MPU-6050 by writing 0x07 to register 0x68;
        self.write_byte(INT_PIN_CFG::ADDR, 0x20)?; //write register 0x37 to select how to use the interrupt pin. For an active high, push-pull signal that stays until register (decimal) 58 is read, write 0x20.
        self.write_byte(ACCEL_CONFIG::ADDR, 0x01)?; //Write register 28 (==0x1C) to set the Digital High Pass Filter, bits 3:0. For example set it to 0x01 for 5Hz. (These 3 bits are grey in the data sheet, but they are used! Leaving them 0 means the filter always outputs 0.)
        self.write_byte(MOT_THR, 10)?; //Write the desired Motion threshold to register 0x1F (For example, write decimal 20).
        self.write_byte(MOT_DUR, 40)?; //Set motion detect duration to 1  ms; LSB is 1 ms @ 1 kHz rate
        self.write_byte(0x69, 0x15)?; //to register 0x69, write the motion detection decrement and a few other settings (for example write 0x15 to set both free-fall and motion decrements to 1 and accelerometer start-up delay to 5ms total by adding 1ms. )
        self.write_byte(INT_ENABLE::ADDR, 0x40)?; //write register 0x38, bit 6 (0x40), to enable motion detection interrupt.
        Ok(())
    }

    /// get whether or not WOM has been detected (INT_STATUS) one of (WOM_X_INT, WOM_Y_INT, WOM_Z_INT)
    pub fn get_motion_detected(&mut self) -> Result<bool, Mpu6886Error<E>> {
        let mask = INT_STATUS::WOM_X_INT | INT_STATUS::WOM_Y_INT | INT_STATUS::WOM_Z_INT;
        Ok(self.read_bit(INT_STATUS::ADDR, mask)? != 0)
    }


    /// Set gyro range, and update sensitivity accordingly
    pub fn set_gyro_range(&mut self, range: GyroRange) -> Result<(), Mpu6886Error<E>> {
        self.write_bits(GYRO_CONFIG::ADDR,
                        GYRO_CONFIG::FS_SEL.bit,
                        GYRO_CONFIG::FS_SEL.length,
                        range as u8)?;

        self.gyro_sensitivity = range.sensitivity();
        Ok(())
    }

    /// get current gyro range
    pub fn get_gyro_range(&mut self) -> Result<GyroRange, Mpu6886Error<E>> {
        let byte = self.read_bits(GYRO_CONFIG::ADDR,
                                  GYRO_CONFIG::FS_SEL.bit,
                                  GYRO_CONFIG::FS_SEL.length)?;

        Ok(GyroRange::from(byte))
    }

    /// set accel range, and update sensitivy accordingly
    pub fn set_accel_range(&mut self, range: AccelRange) -> Result<(), Mpu6886Error<E>> {
        self.write_bits(ACCEL_CONFIG::ADDR,
                        ACCEL_CONFIG::FS_SEL.bit,
                        ACCEL_CONFIG::FS_SEL.length,
                        range as u8)?;

        self.acc_sensitivity = range.sensitivity();
        Ok(())
    }

    /// get current accel_range
    pub fn get_accel_range(&mut self) -> Result<AccelRange, Mpu6886Error<E>> {
        let byte = self.read_bits(ACCEL_CONFIG::ADDR,
                                  ACCEL_CONFIG::FS_SEL.bit,
                                  ACCEL_CONFIG::FS_SEL.length)?;

        Ok(AccelRange::from(byte))
    }

    /// reset device
    pub fn reset_device<D: DelayMs<u8>>(&mut self, delay: &mut D) -> Result<(), Mpu6886Error<E>> {
        self.write_bit(PWR_MGMT_1::ADDR, PWR_MGMT_1::DEVICE_RESET, true)?;
        delay.delay_ms(100u8);
        // Note: Reset sets sleep to true! Section register map: resets PWR_MGMT to 0x40
        Ok(())
    }

    /// enable, disable sleep of sensor
    pub fn set_sleep_enabled(&mut self, enable: bool) -> Result<(), Mpu6886Error<E>> {
        Ok(self.write_bit(PWR_MGMT_1::ADDR, PWR_MGMT_1::SLEEP, enable)?)
    }

    /// get sleep status
    pub fn get_sleep_enabled(&mut self) -> Result<bool, Mpu6886Error<E>> {
        Ok(self.read_bit(PWR_MGMT_1::ADDR, PWR_MGMT_1::SLEEP)? != 0)
    }

    /// enable, disable temperature measurement of sensor
    /// TEMP_DIS actually saves "disabled status"
    /// 1 is disabled! -> enable=true : bit=!enable
    pub fn set_temp_enabled(&mut self, enable: bool) -> Result<(), Mpu6886Error<E>> {
        Ok(self.write_bit(PWR_MGMT_1::ADDR, PWR_MGMT_1::TEMP_DIS, !enable)?)
    }

    /// get temperature sensor status
    /// TEMP_DIS actually saves "disabled status"
    /// 1 is disabled! -> 1 == 0 : false, 0 == 0 : true
    pub fn get_temp_enabled(&mut self) -> Result<bool, Mpu6886Error<E>> {
        Ok(self.read_bit(PWR_MGMT_1::ADDR, PWR_MGMT_1::TEMP_DIS)? == 0)
    }

    /// set accel x self test
    pub fn set_accel_x_self_test(&mut self, enable: bool) -> Result<(), Mpu6886Error<E>> {
        Ok(self.write_bit(ACCEL_CONFIG::ADDR, ACCEL_CONFIG::XA_ST, enable)?)
    }

    /// get accel x self test
    pub fn get_accel_x_self_test(&mut self) -> Result<bool, Mpu6886Error<E>> {
        Ok(self.read_bit(ACCEL_CONFIG::ADDR, ACCEL_CONFIG::XA_ST)? != 0)
    }

    /// set accel y self test
    pub fn set_accel_y_self_test(&mut self, enable: bool) -> Result<(), Mpu6886Error<E>> {
        Ok(self.write_bit(ACCEL_CONFIG::ADDR, ACCEL_CONFIG::YA_ST, enable)?)
    }

    /// get accel y self test
    pub fn get_accel_y_self_test(&mut self) -> Result<bool, Mpu6886Error<E>> {
        Ok(self.read_bit(ACCEL_CONFIG::ADDR, ACCEL_CONFIG::YA_ST)? != 0)
    }

    /// set accel z self test
    pub fn set_accel_z_self_test(&mut self, enable: bool) -> Result<(), Mpu6886Error<E>> {
        Ok(self.write_bit(ACCEL_CONFIG::ADDR, ACCEL_CONFIG::ZA_ST, enable)?)
    }

    /// get accel z self test
    pub fn get_accel_z_self_test(&mut self) -> Result<bool, Mpu6886Error<E>> {
        Ok(self.read_bit(ACCEL_CONFIG::ADDR, ACCEL_CONFIG::ZA_ST)? != 0)
    }

    /// Roll and pitch estimation from raw accelerometer readings
    /// NOTE: no yaw! no magnetometer present on mpu6886
    /// https://www.nxp.com/docs/en/application-note/AN3461.pdf equation 28, 29
    pub fn get_acc_angles(&mut self) -> Result<Vector2<f32>, Mpu6886Error<E>> {
        let acc = self.get_acc()?;

        Ok(Vector2::<f32>::new(
            atan2f(acc.y, sqrtf(powf(acc.x, 2.) + powf(acc.z, 2.))),
            atan2f(-acc.x, sqrtf(powf(acc.y, 2.) + powf(acc.z, 2.)))
        ))
    }

    pub fn get_accel_bandwith(&mut self) -> Result<AccelBw, Mpu6886Error<E>> {
        // `ACCEL_UI_FILT_BW` occupies bits 2:0 in the register
        let bw_sel = self.read_bits(ACCEL_CONFIG_2::ADDR, 3, 4)?;
        let bw = AccelBw::try_from(bw_sel)?;

        Ok(bw)
    }

    pub fn set_accel_bw(&mut self, bw: AccelBw) -> Result<(), Mpu6886Error<E>> {
        // TODO: modify register if DEC2_CFG needs to be set elsewhere

        self.write_byte(CONFIG::ADDR, bw.bits())?;
        self.write_byte(GYRO_CONFIG::ADDR, bw.bits())?;
        
        Ok(())
    }

    pub fn get_gyro_bandwith(&mut self) -> Result<GyroBw, Mpu6886Error<E>> {
        // `DLPF_CFG` occupies bits 2:0 in the register of CONFIGURATION
        let bw_sel = self.read_bits(CONFIG::ADDR, 2, 3)?;
        let fchoice_b = self.read_bits(GYRO_CONFIG::ADDR, 1, 2)?;
        let bw = GyroBw::try_from(bw_sel | (fchoice_b << 3))?;

        Ok(bw)
    }

    pub fn set_gyro_bw(&mut self, bw: GyroBw) -> Result<(), Mpu6886Error<E>> {
        // TODO: modify register if DEC2_CFG needs to be set elsewhere
        //self.write_byte(ACCEL_CONFIG_2::ADDR, bw.bits())?;
        
        Ok(())
    }

    /// Converts 2 bytes number in 2 compliment
    /// TODO i16?! whats 0x8000?!
    fn read_word_2c(&self, byte: &[u8]) -> i32 {
        let high: i32 = byte[0] as i32;
        let low: i32 = byte[1] as i32;
        let mut word: i32 = (high << 8) + low;

        if word >= 0x8000 {
            word = -((65535 - word) + 1);
        }

        word
    }



    /// Reads rotation (gyro/acc) from specified register
    fn read_rot(&mut self, reg: u8) -> Result<Vector3<f32>, Mpu6886Error<E>> {
        let mut buf: [u8; 6] = [0; 6];
        self.read_bytes(reg, &mut buf)?;

        Ok(Vector3::<f32>::new(
            self.read_word_2c(&buf[0..2]) as f32,
            self.read_word_2c(&buf[2..4]) as f32,
            self.read_word_2c(&buf[4..6]) as f32
        ))
    }

    /// Accelerometer readings in g
    pub fn get_acc(&mut self) -> Result<Vector3<f32>, Mpu6886Error<E>> {
        let mut acc = self.read_rot(ACC_REGX_H)?;
        acc /= self.acc_sensitivity;

        Ok(acc)
    }

    /// Gyro readings in rad/s
    pub fn get_gyro(&mut self) -> Result<Vector3<f32>, Mpu6886Error<E>> {
        let mut gyro = self.read_rot(GYRO_REGX_H)?;

        gyro *= PI_180 / self.gyro_sensitivity;

        Ok(gyro)
    }

    /// Sensor Temp in degrees celcius
    pub fn get_temp(&mut self) -> Result<f32, Mpu6886Error<E>> {
        let mut buf: [u8; 2] = [0; 2];
        self.read_bytes(TEMP_OUT_H, &mut buf)?;
        let raw_temp = self.read_word_2c(&buf[0..2]) as f32;

        // let high: u16 = buf[0] as u16;
        // let low: u16 = buf[1] as u16;
        // let word = high << 8 | low;
        // let raw_temp = word as f32;

        // According to revision 4.2
        Ok((raw_temp / TEMP_SENSITIVITY) + TEMP_OFFSET)
    }

    /// enable writing data to the fifo output, this function must be called before
    /// reading with read_fifo()
    /// currently only enabling all data gyro and accel is supported by the fifo-read()
    /// enabling gyro will also enabel temperature
    pub fn enable_fifo(&mut self, accel: bool, gyro: bool) -> Result<(), Mpu6886Error<E>> {
        self.write_bit(FIFO_EN, 3, accel)?;
        self.write_bit(FIFO_EN, 4, gyro)?;
        self.write_bit(USER_CTRL, 0, true)?;  // reset signal path
        self.write_bit(USER_CTRL, 2, true)?; // reset fifo path
        self.write_bit(USER_CTRL, 6, true)?; // enable fifo
        Ok(())
    }

    /// Read sensor data from FIFO in one go
    /// currently only enabling all data gyro and accel is supported by the fifo-read()
    /// Vector_0 contains accelerometer data in g 
    /// Vector_1 contains gyro data in °/sec
    /// Vector_2 contains temperature in first position rest 0
    #[inline(always)]
    pub fn read_fifo(&mut self)  -> Result<Vector3<Vector3<f32>>, Mpu6886Error<E>> {
        let mut buf: [u8; 14] = [0; 14];
        self.read_bytes(FIFO_R_W, &mut buf)?;
        if buf[0] != 255 {
            let ax = (self.read_word_2c(&buf[0..2]) as f32)/self.acc_sensitivity;
            let ay = (self.read_word_2c(&buf[2..4]) as f32)/self.acc_sensitivity;
            let az = (self.read_word_2c(&buf[4..6]) as f32)/self.acc_sensitivity;
            let t = (self.read_word_2c(&buf[6..8]) as f32/TEMP_SENSITIVITY) + TEMP_OFFSET;
            let gx = (self.read_word_2c(&buf[8..10]) as f32) / self.gyro_sensitivity;
            let gy = (self.read_word_2c(&buf[10..12]) as f32) / self.gyro_sensitivity;
            let gz = (self.read_word_2c(&buf[12..14]) as f32) / self.gyro_sensitivity;

            Ok(Vector3::<Vector3<f32>>::new(
                Vector3::new(ax,ay,az),
                Vector3::new(gx,gy,gz),
                Vector3::new(t,0.0,0.0),
            ))
        } else {
            Err(Mpu6886Error::SensorError(SensorError::NofFifoData))
        }
    }

    pub fn read_fifo_si(&mut self) -> Result<Vector3<Vector3<f32>>, Mpu6886Error<E>> {
        let mut data = self.read_fifo()?;
        data[0][0] = data[0][0] * GRAVITY;
        data[0][1] = data[0][1] * GRAVITY;
        data[0][2] = data[0][2] * GRAVITY;
        data[1][0] = data[1][0] * PI_180;
        data[1][1] = data[1][1] * PI_180;
        data[1][2] = data[1][2] * PI_180;
        Ok(data)
    }

    /// Writes byte to register
    pub fn write_byte(&mut self, reg: u8, byte: u8) -> Result<(), Mpu6886Error<E>> {
        self.i2c.write(self.slave_addr, &[reg, byte])
            .map_err(Mpu6886Error::I2c)?;
        // delay disabled for dev build
        // TODO: check effects with physical unit
        // self.delay.delay_ms(10u8);
        Ok(())
    }

    /// Enables bit n at register address reg
    pub fn write_bit(&mut self, reg: u8, bit_n: u8, enable: bool) -> Result<(), Mpu6886Error<E>> {
        let mut byte: [u8; 1] = [0; 1];
        self.read_bytes(reg, &mut byte)?;
        bits::set_bit(&mut byte[0], bit_n, enable);
        Ok(self.write_byte(reg, byte[0])?)
    }

    /// Write bits data at reg from start_bit to start_bit+length
    pub fn write_bits(&mut self, reg: u8, start_bit: u8, length: u8, data: u8) -> Result<(), Mpu6886Error<E>> {
        let mut byte: [u8; 1] = [0; 1];
        self.read_bytes(reg, &mut byte)?;
        bits::set_bits(&mut byte[0], start_bit, length, data);
        Ok(self.write_byte(reg, byte[0])?)
    }

    /// Read bit n from register
    fn read_bit(&mut self, reg: u8, bit_n: u8) -> Result<u8, Mpu6886Error<E>> {
        let mut byte: [u8; 1] = [0; 1];
        self.read_bytes(reg, &mut byte)?;
        Ok(bits::get_bit(byte[0], bit_n))
    }

    /// Read bits at register reg, starting with bit start_bit, until start_bit+length
    pub fn read_bits(&mut self, reg: u8, start_bit: u8, length: u8) -> Result<u8, Mpu6886Error<E>> {
        let mut byte: [u8; 1] = [0; 1];
        self.read_bytes(reg, &mut byte)?;
        Ok(bits::get_bits(byte[0], start_bit, length))
    }

    /// Reads byte from register
    pub fn read_byte(&mut self, reg: u8) -> Result<u8, Mpu6886Error<E>> {
        let mut byte: [u8; 1] = [0; 1];
        self.i2c.write_read(self.slave_addr, &[reg], &mut byte)
            .map_err(Mpu6886Error::I2c)?;
        Ok(byte[0])
    }

    /// Reads series of bytes into buf from specified reg
    pub fn read_bytes(&mut self, reg: u8, buf: &mut [u8]) -> Result<(), Mpu6886Error<E>> {
        self.i2c.write_read(self.slave_addr, &[reg], buf)
            .map_err(Mpu6886Error::I2c)?;
        Ok(())
    }
}

