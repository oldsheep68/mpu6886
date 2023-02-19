# `mpu6886` ![crates.io](https://img.shields.io/crates/v/mpu6886.svg) ![CircleCI](https://img.shields.io/circleci/build/github/juliangaal/mpu6886.svg)
> no_std driver for the mpu6886 6-axis IMU

thanks to "Julian Gaal <gjulian@uos.de>", for the major work in mpu6050 
only small adaptions for mpu6886 were done:
* device ID for mpu6886 changed 
* temperature reading adapted
* CLKSEL is redefined
* IRQ STATUS redefined
removed:
* MOT_DETECT_STATUS
* MOT_DETECT_CONTROL
* LP_WAKE_CTRL
* ACCEL_HPF


## What Works most probably now after changes from mpu6050 to mpu6886
Not, almost no tests have been done yet.
* Reading the accelerometer, gyroscope, temperature sensor
    * raw
    * scaled
    * roll/pitch estimation
* Setting Accel/Gyro Ranges/Sensitivity





## Basic usage 
To use this driver you must provide a concrete `embedded_hal` implementation. Here's a 
[`linux_embedded_hal`](https://github.com/rust-embedded/linux-embedded-hal) example
```rust
use mpu6886::*;
use linux_embedded_hal::{I2cdev, Delay};
use i2cdev::linux::LinuxI2CError;

fn main() -> Result<(), mpu6886Error<LinuxI2CError>> {
  let i2c = I2cdev::new("/dev/i2c-1")
          .map_err(mpu6886Error::I2c)?;

  let mut delay = Delay;
  let mut mpu = mpu6886::new(i2c);

  mpu.init(&mut delay)?;

  loop {
    // get roll and pitch estimate
    let acc = mpu.get_acc_angles()?;
    println!("r/p: {:?}", acc);

    // get temp
    let temp = mpu.get_temp()?;
    println!("temp: {:?}c", temp);

    // get gyro data, scaled with sensitivity 
    let gyro = mpu.get_gyro()?;
    println!("gyro: {:?}", gyro);

    // get accelerometer data, scaled with sensitivity
    let acc = mpu.get_acc()?;
    println!("acc: {:?}", acc);
  }
}
```