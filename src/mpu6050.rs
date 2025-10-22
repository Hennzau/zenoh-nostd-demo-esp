//! Inclides code from MIT licensed project 'mpu6050 by juliangaal'

use embedded_hal_async::{delay::DelayNs, i2c::I2c};
use libm::{atan2f, powf, sqrtf};

use crate::mpu6050::constants::*;

mod constants;

const PI_180: f32 = core::f32::consts::PI / 180.0;

pub fn get_bit(byte: u8, n: u8) -> u8 {
    (byte >> n) & 1
}

pub fn get_bits(mut byte: u8, bit_start: u8, length: u8) -> u8 {
    let mask_shift: u8 = if bit_start < length {
        0
    } else {
        bit_start - length + 1
    };

    let mask: u8 = ((1 << length) - 1) << mask_shift;

    byte &= mask as u8;
    byte >>= mask_shift;
    byte
}

pub fn set_bit(byte: &mut u8, n: u8, enable: bool) {
    if enable {
        *byte |= 1_u8 << n;
    } else {
        *byte &= !(1_u8 << n);
    }
}

pub fn set_bits(byte: &mut u8, bit_start: u8, length: u8, mut data: u8) {
    let mask_shift: u8 = if bit_start < length {
        0
    } else {
        bit_start - length + 1
    };

    let mask: u8 = ((1 << length) - 1) << mask_shift;

    data <<= mask_shift;
    data &= mask;

    *byte &= !(mask);
    *byte |= data;
}

#[derive(Debug)]
pub enum Mpu6050Error<E> {
    I2c(E),
    InvalidChipId(u8),
}

pub struct Mpu6050<I2C> {
    i2c: I2C,
    slave_addr: u8,
    acc_sensitivity: f32,
    gyro_sensitivity: f32,
}

pub struct Vector3<T> {
    pub x: T,
    pub y: T,
    pub z: T,
}

impl<T> Vector3<T> {
    pub fn new(x: T, y: T, z: T) -> Self {
        Vector3 { x, y, z }
    }
}

impl<T> core::ops::Div<f32> for Vector3<T>
where
    T: core::ops::Div<f32, Output = T>,
{
    type Output = Vector3<T>;

    fn div(self, rhs: f32) -> Self::Output {
        Vector3::<T>::new(self.x / rhs, self.y / rhs, self.z / rhs)
    }
}

impl<T> core::ops::DivAssign<f32> for Vector3<T>
where
    T: core::ops::DivAssign<f32>,
{
    fn div_assign(&mut self, rhs: f32) {
        self.x /= rhs;
        self.y /= rhs;
        self.z /= rhs;
    }
}

impl<T> core::ops::Mul<f32> for Vector3<T>
where
    T: core::ops::Mul<f32, Output = T>,
{
    type Output = Vector3<T>;

    fn mul(self, rhs: f32) -> Self::Output {
        Vector3::<T>::new(self.x * rhs, self.y * rhs, self.z * rhs)
    }
}

impl<T> core::ops::MulAssign<f32> for Vector3<T>
where
    T: core::ops::MulAssign<f32>,
{
    fn mul_assign(&mut self, rhs: f32) {
        self.x *= rhs;
        self.y *= rhs;
        self.z *= rhs;
    }
}

struct Vector2<T> {
    pub x: T,
    pub y: T,
}

impl<T> Vector2<T> {
    pub fn new(x: T, y: T) -> Self {
        Vector2 { x, y }
    }
}

impl<I2C> core::ops::Div<f32> for Vector2<I2C>
where
    I2C: core::ops::Div<f32, Output = I2C>,
{
    type Output = Vector2<I2C>;

    fn div(self, rhs: f32) -> Self::Output {
        Vector2::<I2C>::new(self.x / rhs, self.y / rhs)
    }
}

impl<I2C, E> Mpu6050<I2C>
where
    I2C: I2c<Error = E>,
{
    pub fn new(i2c: I2C) -> Self {
        Mpu6050 {
            i2c,
            slave_addr: DEFAULT_SLAVE_ADDR,
            acc_sensitivity: ACCEL_SENS.0,
            gyro_sensitivity: GYRO_SENS.0,
        }
    }

    pub fn new_with_sens(i2c: I2C, arange: AccelRange, grange: GyroRange) -> Self {
        Mpu6050 {
            i2c,
            slave_addr: DEFAULT_SLAVE_ADDR,
            acc_sensitivity: arange.sensitivity(),
            gyro_sensitivity: grange.sensitivity(),
        }
    }

    pub fn new_with_addr(i2c: I2C, slave_addr: u8) -> Self {
        Mpu6050 {
            i2c,
            slave_addr,
            acc_sensitivity: ACCEL_SENS.0,
            gyro_sensitivity: GYRO_SENS.0,
        }
    }

    pub fn new_with_addr_and_sens(
        i2c: I2C,
        slave_addr: u8,
        arange: AccelRange,
        grange: GyroRange,
    ) -> Self {
        Mpu6050 {
            i2c,
            slave_addr,
            acc_sensitivity: arange.sensitivity(),
            gyro_sensitivity: grange.sensitivity(),
        }
    }

    async fn wake<D: DelayNs>(&mut self, delay: &mut D) -> Result<(), Mpu6050Error<E>> {
        self.write_byte(PWR_MGMT_1::ADDR, 0x01).await?;
        delay.delay_ms(100u32).await;
        Ok(())
    }

    pub async fn set_clock_source(&mut self, source: CLKSEL) -> Result<(), Mpu6050Error<E>> {
        Ok(self
            .write_bits(
                PWR_MGMT_1::ADDR,
                PWR_MGMT_1::CLKSEL.bit,
                PWR_MGMT_1::CLKSEL.length,
                source as u8,
            )
            .await?)
    }

    pub async fn get_clock_source(&mut self) -> Result<CLKSEL, Mpu6050Error<E>> {
        let source = self
            .read_bits(
                PWR_MGMT_1::ADDR,
                PWR_MGMT_1::CLKSEL.bit,
                PWR_MGMT_1::CLKSEL.length,
            )
            .await?;
        Ok(CLKSEL::from(source))
    }

    pub async fn init<D: DelayNs>(&mut self, delay: &mut D) -> Result<(), Mpu6050Error<E>> {
        self.wake(delay).await?;
        self.verify().await?;
        self.set_accel_range(AccelRange::G2).await?;
        self.set_gyro_range(GyroRange::D250).await?;
        self.set_accel_hpf(ACCEL_HPF::_RESET).await?;
        Ok(())
    }

    async fn verify(&mut self) -> Result<(), Mpu6050Error<E>> {
        let address = self.read_byte(WHOAMI).await?;
        if address != DEFAULT_SLAVE_ADDR {
            return Err(Mpu6050Error::InvalidChipId(address));
        }
        Ok(())
    }
    pub async fn setup_motion_detection(&mut self) -> Result<(), Mpu6050Error<E>> {
        self.write_byte(0x6B, 0x00).await?;
        self.write_byte(INT_PIN_CFG::ADDR, 0x20).await?;
        self.write_byte(ACCEL_CONFIG::ADDR, 0x01).await?;
        self.write_byte(MOT_THR, 10).await?;
        self.write_byte(MOT_DUR, 40).await?;
        self.write_byte(0x69, 0x15).await?;
        self.write_byte(INT_ENABLE::ADDR, 0x40).await?;
        Ok(())
    }

    pub async fn get_motion_detected(&mut self) -> Result<bool, Mpu6050Error<E>> {
        Ok(self.read_bit(INT_STATUS::ADDR, INT_STATUS::MOT_INT).await? != 0)
    }

    pub async fn set_accel_hpf(&mut self, mode: ACCEL_HPF) -> Result<(), Mpu6050Error<E>> {
        Ok(self
            .write_bits(
                ACCEL_CONFIG::ADDR,
                ACCEL_CONFIG::ACCEL_HPF.bit,
                ACCEL_CONFIG::ACCEL_HPF.length,
                mode as u8,
            )
            .await?)
    }

    pub async fn get_accel_hpf(&mut self) -> Result<ACCEL_HPF, Mpu6050Error<E>> {
        let mode: u8 = self
            .read_bits(
                ACCEL_CONFIG::ADDR,
                ACCEL_CONFIG::ACCEL_HPF.bit,
                ACCEL_CONFIG::ACCEL_HPF.length,
            )
            .await?;

        Ok(ACCEL_HPF::from(mode))
    }

    pub async fn set_gyro_range(&mut self, range: GyroRange) -> Result<(), Mpu6050Error<E>> {
        self.write_bits(
            GYRO_CONFIG::ADDR,
            GYRO_CONFIG::FS_SEL.bit,
            GYRO_CONFIG::FS_SEL.length,
            range as u8,
        )
        .await?;

        self.gyro_sensitivity = range.sensitivity();
        Ok(())
    }

    pub async fn get_gyro_range(&mut self) -> Result<GyroRange, Mpu6050Error<E>> {
        let byte = self
            .read_bits(
                GYRO_CONFIG::ADDR,
                GYRO_CONFIG::FS_SEL.bit,
                GYRO_CONFIG::FS_SEL.length,
            )
            .await?;

        Ok(GyroRange::from(byte))
    }

    pub async fn set_accel_range(&mut self, range: AccelRange) -> Result<(), Mpu6050Error<E>> {
        self.write_bits(
            ACCEL_CONFIG::ADDR,
            ACCEL_CONFIG::FS_SEL.bit,
            ACCEL_CONFIG::FS_SEL.length,
            range as u8,
        )
        .await?;

        self.acc_sensitivity = range.sensitivity();
        Ok(())
    }

    pub async fn get_accel_range(&mut self) -> Result<AccelRange, Mpu6050Error<E>> {
        let byte = self
            .read_bits(
                ACCEL_CONFIG::ADDR,
                ACCEL_CONFIG::FS_SEL.bit,
                ACCEL_CONFIG::FS_SEL.length,
            )
            .await?;

        Ok(AccelRange::from(byte))
    }

    pub async fn reset_device<D: DelayNs>(&mut self, delay: &mut D) -> Result<(), Mpu6050Error<E>> {
        self.write_bit(PWR_MGMT_1::ADDR, PWR_MGMT_1::DEVICE_RESET, true)
            .await?;
        delay.delay_ms(100u32);
        Ok(())
    }

    pub async fn set_master_interrupt_enabled(
        &mut self,
        enable: bool,
    ) -> Result<(), Mpu6050Error<E>> {
        Ok(self
            .write_bit(INT_ENABLE::ADDR, INT_ENABLE::I2C_MST_INT_EN, enable)
            .await?)
    }

    pub async fn get_master_interrupt_enabled(&mut self) -> Result<bool, Mpu6050Error<E>> {
        Ok(self
            .read_bit(INT_ENABLE::ADDR, INT_ENABLE::I2C_MST_INT_EN)
            .await?
            != 0)
    }

    pub async fn set_bypass_enabled(&mut self, enable: bool) -> Result<(), Mpu6050Error<E>> {
        Ok(self
            .write_bit(INT_PIN_CFG::ADDR, INT_PIN_CFG::I2C_BYPASS_EN, enable)
            .await?)
    }

    pub async fn get_bypass_enabled(&mut self) -> Result<bool, Mpu6050Error<E>> {
        Ok(self
            .read_bit(INT_PIN_CFG::ADDR, INT_PIN_CFG::I2C_BYPASS_EN)
            .await?
            != 0)
    }

    pub async fn set_sleep_enabled(&mut self, enable: bool) -> Result<(), Mpu6050Error<E>> {
        Ok(self
            .write_bit(PWR_MGMT_1::ADDR, PWR_MGMT_1::SLEEP, enable)
            .await?)
    }

    pub async fn get_sleep_enabled(&mut self) -> Result<bool, Mpu6050Error<E>> {
        Ok(self.read_bit(PWR_MGMT_1::ADDR, PWR_MGMT_1::SLEEP).await? != 0)
    }

    pub async fn set_temp_enabled(&mut self, enable: bool) -> Result<(), Mpu6050Error<E>> {
        Ok(self
            .write_bit(PWR_MGMT_1::ADDR, PWR_MGMT_1::TEMP_DIS, !enable)
            .await?)
    }

    pub async fn get_temp_enabled(&mut self) -> Result<bool, Mpu6050Error<E>> {
        Ok(self
            .read_bit(PWR_MGMT_1::ADDR, PWR_MGMT_1::TEMP_DIS)
            .await?
            == 0)
    }

    pub async fn set_accel_x_self_test(&mut self, enable: bool) -> Result<(), Mpu6050Error<E>> {
        Ok(self
            .write_bit(ACCEL_CONFIG::ADDR, ACCEL_CONFIG::XA_ST, enable)
            .await?)
    }

    pub async fn get_accel_x_self_test(&mut self) -> Result<bool, Mpu6050Error<E>> {
        Ok(self
            .read_bit(ACCEL_CONFIG::ADDR, ACCEL_CONFIG::XA_ST)
            .await?
            != 0)
    }

    pub async fn set_accel_y_self_test(&mut self, enable: bool) -> Result<(), Mpu6050Error<E>> {
        Ok(self
            .write_bit(ACCEL_CONFIG::ADDR, ACCEL_CONFIG::YA_ST, enable)
            .await?)
    }

    pub async fn get_accel_y_self_test(&mut self) -> Result<bool, Mpu6050Error<E>> {
        Ok(self
            .read_bit(ACCEL_CONFIG::ADDR, ACCEL_CONFIG::YA_ST)
            .await?
            != 0)
    }

    pub async fn set_accel_z_self_test(&mut self, enable: bool) -> Result<(), Mpu6050Error<E>> {
        Ok(self
            .write_bit(ACCEL_CONFIG::ADDR, ACCEL_CONFIG::ZA_ST, enable)
            .await?)
    }

    pub async fn get_accel_z_self_test(&mut self) -> Result<bool, Mpu6050Error<E>> {
        Ok(self
            .read_bit(ACCEL_CONFIG::ADDR, ACCEL_CONFIG::ZA_ST)
            .await?
            != 0)
    }

    pub async fn get_acc_angles(&mut self) -> Result<Vector2<f32>, Mpu6050Error<E>> {
        let acc = self.get_acc().await?;

        Ok(Vector2::<f32>::new(
            atan2f(acc.y, sqrtf(powf(acc.x, 2.) + powf(acc.z, 2.))),
            atan2f(-acc.x, sqrtf(powf(acc.y, 2.) + powf(acc.z, 2.))),
        ))
    }

    fn read_word_2c(&self, byte: &[u8]) -> i32 {
        let high: i32 = byte[0] as i32;
        let low: i32 = byte[1] as i32;
        let mut word: i32 = (high << 8) + low;

        if word >= 0x8000 {
            word = -((65535 - word) + 1);
        }

        word
    }

    async fn read_rot(&mut self, reg: u8) -> Result<Vector3<f32>, Mpu6050Error<E>> {
        let mut buf: [u8; 6] = [0; 6];
        self.read_bytes(reg, &mut buf).await?;

        Ok(Vector3::<f32>::new(
            self.read_word_2c(&buf[0..2]) as f32,
            self.read_word_2c(&buf[2..4]) as f32,
            self.read_word_2c(&buf[4..6]) as f32,
        ))
    }

    pub async fn get_acc(&mut self) -> Result<Vector3<f32>, Mpu6050Error<E>> {
        let mut acc = self.read_rot(ACC_REGX_H).await?;
        acc /= self.acc_sensitivity;

        Ok(acc)
    }

    pub async fn get_gyro(&mut self) -> Result<Vector3<f32>, Mpu6050Error<E>> {
        let mut gyro = self.read_rot(GYRO_REGX_H).await?;

        gyro *= PI_180 / self.gyro_sensitivity;

        Ok(gyro)
    }

    pub async fn get_temp(&mut self) -> Result<f32, Mpu6050Error<E>> {
        let mut buf: [u8; 2] = [0; 2];
        self.read_bytes(TEMP_OUT_H, &mut buf).await?;
        let raw_temp = self.read_word_2c(&buf[0..2]) as f32;

        Ok((raw_temp / TEMP_SENSITIVITY) + TEMP_OFFSET)
    }

    pub async fn write_byte(&mut self, reg: u8, byte: u8) -> Result<(), Mpu6050Error<E>> {
        self.i2c
            .write(self.slave_addr, &[reg, byte])
            .await
            .map_err(Mpu6050Error::I2c)?;

        Ok(())
    }

    pub async fn write_bit(
        &mut self,
        reg: u8,
        bit_n: u8,
        enable: bool,
    ) -> Result<(), Mpu6050Error<E>> {
        let mut byte: [u8; 1] = [0; 1];
        self.read_bytes(reg, &mut byte).await?;
        set_bit(&mut byte[0], bit_n, enable);
        Ok(self.write_byte(reg, byte[0]).await?)
    }

    pub async fn write_bits(
        &mut self,
        reg: u8,
        start_bit: u8,
        length: u8,
        data: u8,
    ) -> Result<(), Mpu6050Error<E>> {
        let mut byte: [u8; 1] = [0; 1];
        self.read_bytes(reg, &mut byte).await?;
        set_bits(&mut byte[0], start_bit, length, data);
        Ok(self.write_byte(reg, byte[0]).await?)
    }

    async fn read_bit(&mut self, reg: u8, bit_n: u8) -> Result<u8, Mpu6050Error<E>> {
        let mut byte: [u8; 1] = [0; 1];
        self.read_bytes(reg, &mut byte).await?;
        Ok(get_bit(byte[0], bit_n))
    }

    pub async fn read_bits(
        &mut self,
        reg: u8,
        start_bit: u8,
        length: u8,
    ) -> Result<u8, Mpu6050Error<E>> {
        let mut byte: [u8; 1] = [0; 1];
        self.read_bytes(reg, &mut byte).await?;
        Ok(get_bits(byte[0], start_bit, length))
    }

    pub async fn read_byte(&mut self, reg: u8) -> Result<u8, Mpu6050Error<E>> {
        let mut byte: [u8; 1] = [0; 1];
        self.i2c
            .write_read(self.slave_addr, &[reg], &mut byte)
            .await
            .map_err(Mpu6050Error::I2c)?;
        Ok(byte[0])
    }

    pub async fn read_bytes(&mut self, reg: u8, buf: &mut [u8]) -> Result<(), Mpu6050Error<E>> {
        self.i2c
            .write_read(self.slave_addr, &[reg], buf)
            .await
            .map_err(Mpu6050Error::I2c)?;
        Ok(())
    }
}
