use mpu6886::{*, device::*};

use linux_embedded_hal::{I2cdev, Delay};
use i2cdev::linux::LinuxI2CError;
use mpu6886::device::{ CLKSEL};

fn main() -> Result<(), Mpu6886Error<LinuxI2CError>> {
    let i2c = I2cdev::new("/dev/i2c-1")
        .map_err(Mpu6886Error::I2c)?;

    let mut delay = Delay;
    let mut mpu = Mpu6886::new(i2c);
    
    mpu.init(&mut delay)?;

    // Test power management
    println!("Test power management");

    // Test gyro config
    println!("Test gyro config");
    assert_eq!(mpu.get_gyro_range()?, GyroRange::D250);
    mpu.set_gyro_range(GyroRange::D500)?;
    assert_eq!(mpu.get_gyro_range()?, GyroRange::D500);

    // Test accel config
    println!("Test accel config");
    assert_eq!(mpu.get_accel_range()?, AccelRange::G2);
    mpu.set_accel_range(AccelRange::G4)?;
    assert_eq!(mpu.get_accel_range()?, AccelRange::G4);


    // test sleep. Default no, in wake()
    println!("Test sleep");
    assert_eq!(mpu.get_sleep_enabled()?, false);
    mpu.set_sleep_enabled(true)?;
    assert_eq!(mpu.get_sleep_enabled()?, true);
    mpu.set_sleep_enabled(false)?;
    assert_eq!(mpu.get_sleep_enabled()?, false);

    // test temp enable/disable
    println!("Test temp enable/disable");
    mpu.set_temp_enabled(false)?;
    assert_eq!(mpu.get_temp_enabled()?, false);
    assert_eq!(mpu.get_temp()?, 36.53);
    mpu.set_temp_enabled(true)?;
    assert_eq!(mpu.get_temp_enabled()?, true);
    assert_ne!(mpu.get_temp()?, 36.53);

    // Test clksel: GXAXIS per default, set in wake()
    println!("Test CLKSEL");
    assert_eq!(mpu.get_clock_source()?, CLKSEL::AUTOPLL1);
    mpu.set_clock_source(CLKSEL::AUTOPLL2)?;
    assert_eq!(mpu.get_clock_source()?, CLKSEL::AUTOPLL2);
    mpu.set_clock_source(CLKSEL::AUTOPLL3)?;
    assert_eq!(mpu.get_clock_source()?, CLKSEL::AUTOPLL3);
    mpu.set_clock_source(CLKSEL::OSCILL)?;
    assert_eq!(mpu.get_clock_source()?, CLKSEL::OSCILL);
    mpu.set_clock_source(CLKSEL::STOP)?;
    assert_eq!(mpu.get_clock_source()?, CLKSEL::STOP);
    mpu.set_clock_source(CLKSEL::AUTOPLL4)?;
    assert_eq!(mpu.get_clock_source()?, CLKSEL::AUTOPLL4);
    mpu.set_clock_source(CLKSEL::AUTOPLL5)?;
    assert_eq!(mpu.get_clock_source()?, CLKSEL::AUTOPLL5);
    mpu.set_clock_source(CLKSEL::OSCILL6)?;
    assert_eq!(mpu.get_clock_source()?, CLKSEL::OSCILL6);

    // reset
    println!("Test reset");
    mpu.reset_device(&mut delay)?;
    assert_eq!(mpu.get_accel_range()?, AccelRange::G2);
    assert_eq!(mpu.get_gyro_range()?, GyroRange::D250);
    assert_eq!(mpu.get_sleep_enabled()?, true);
    assert_eq!(mpu.get_temp_enabled()?, true);

    println!("Test successful");
    Ok(())
}