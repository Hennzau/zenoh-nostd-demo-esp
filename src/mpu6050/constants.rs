//! Code from MIT licensed project 'mpu6050 by juliangaal'

//! #### Sources:
//! * Register map (rev 3.2): https://arduino.ua/docs/RM-MPU-6000A.pdf
//! * Datasheet (rev 3.2): https://www.cdiweb.com/datasheets/invensense/ps-mpu-6000a.pdf

pub const GYRO_SENS: (f32, f32, f32, f32) = (131., 65.5, 32.8, 16.4);

pub const ACCEL_SENS: (f32, f32, f32, f32) = (16384., 8192., 4096., 2048.);

pub const TEMP_OFFSET: f32 = 36.53;

pub const TEMP_SENSITIVITY: f32 = 340.;

pub const MOT_THR: u8 = 0x1F;

pub const MOT_DUR: u8 = 0x20;

pub const GYRO_REGX_H: u8 = 0x43;

pub const GYRO_REGY_H: u8 = 0x45;

pub const GYRO_REGZ_H: u8 = 0x47;

pub const ACC_REGX_H: u8 = 0x3b;

pub const ACC_REGY_H: u8 = 0x3d;

pub const ACC_REGZ_H: u8 = 0x3f;

pub const TEMP_OUT_H: u8 = 0x41;

pub const DEFAULT_SLAVE_ADDR: u8 = 0x68;

pub const WHOAMI: u8 = 0x75;

pub struct BitBlock {
    pub bit: u8,
    pub length: u8,
}

#[allow(non_camel_case_types)]
#[derive(Copy, Clone, Debug)]

pub struct CONFIG;

impl CONFIG {
    pub const ADDR: u8 = 0x1a;

    pub const EXT_SYNC_SET: BitBlock = BitBlock { bit: 5, length: 3 };

    pub const DLPF_CFG: BitBlock = BitBlock { bit: 2, length: 3 };
}

#[allow(non_camel_case_types)]
#[derive(Copy, Clone, Debug)]

pub struct GYRO_CONFIG;

impl GYRO_CONFIG {
    pub const ADDR: u8 = 0x1b;

    pub const XG_ST: u8 = 7;

    pub const YG_ST: u8 = 6;

    pub const ZG_ST: u8 = 5;

    pub const FS_SEL: BitBlock = BitBlock { bit: 4, length: 2 };
}

#[allow(non_camel_case_types)]
#[derive(Copy, Clone, Debug)]

pub struct ACCEL_CONFIG;

impl ACCEL_CONFIG {
    pub const ADDR: u8 = 0x1c;

    pub const XA_ST: u8 = 7;

    pub const YA_ST: u8 = 6;

    pub const ZA_ST: u8 = 5;

    pub const FS_SEL: BitBlock = BitBlock { bit: 4, length: 2 };

    pub const ACCEL_HPF: BitBlock = BitBlock { bit: 2, length: 3 };
}

#[allow(non_camel_case_types)]
#[derive(Copy, Clone, Debug)]

pub struct INT_PIN_CFG;

impl INT_PIN_CFG {
    pub const ADDR: u8 = 0x37;

    pub const INT_LEVEL: u8 = 7;

    pub const INT_OPEN: u8 = 6;

    pub const LATCH_INT_EN: u8 = 5;

    pub const INT_RD_CLEAR: u8 = 4;

    pub const FSYNC_INT_LEVEL: u8 = 3;

    pub const FSYNC_INT_EN: u8 = 2;

    pub const I2C_BYPASS_EN: u8 = 1;

    pub const CLKOUT_EN: u8 = 0;
}

#[allow(non_camel_case_types)]
#[derive(Copy, Clone, Debug)]

pub struct INT_ENABLE;

impl INT_ENABLE {
    pub const ADDR: u8 = 0x38;

    pub const FF_EN: u8 = 7;

    pub const MOT_EN: u8 = 6;

    pub const ZMOT_EN: u8 = 5;

    pub const FIFO_OFLOW_END: u8 = 4;

    pub const I2C_MST_INT_EN: u8 = 3;

    pub const DATA_RDY_EN: u8 = 0;
}

#[allow(non_camel_case_types)]
#[derive(Copy, Clone, Debug)]

pub struct INT_STATUS;

impl INT_STATUS {
    pub const ADDR: u8 = 0x3a;

    pub const FF_INT: u8 = 7;

    pub const MOT_INT: u8 = 6;

    pub const ZMOT_INT: u8 = 5;

    pub const FIFO_OFLOW_INT: u8 = 4;

    pub const I2C_MSF_INT: u8 = 3;

    pub const DATA_RDY_INT: u8 = 0;
}

#[allow(non_camel_case_types)]
#[derive(Copy, Clone, Debug)]

pub struct MOT_DETECT_STATUS;

impl MOT_DETECT_STATUS {
    pub const ADDR: u8 = 0x61;

    pub const MOT_XNEG: u8 = 7;

    pub const MOT_XPOS: u8 = 6;

    pub const MOT_YNEG: u8 = 5;

    pub const MOT_YPOS: u8 = 4;

    pub const MOT_ZNEG: u8 = 3;

    pub const MOT_ZPOS: u8 = 2;

    pub const MOT_ZRMOT: u8 = 0;
}

#[allow(non_camel_case_types)]
#[derive(Copy, Clone, Debug)]

pub struct MOT_DETECT_CONTROL;

impl MOT_DETECT_CONTROL {
    pub const ADDR: u8 = 0x69;

    pub const ACCEL_ON_DELAY: BitBlock = BitBlock { bit: 5, length: 2 };

    pub const FF_COUNT: BitBlock = BitBlock { bit: 3, length: 2 };

    pub const MOT_COUNT: BitBlock = BitBlock { bit: 1, length: 2 };
}

#[allow(non_camel_case_types)]
#[derive(Copy, Clone, Debug)]

pub struct PWR_MGMT_1;

impl PWR_MGMT_1 {
    pub const ADDR: u8 = 0x6b;

    pub const DEVICE_RESET: u8 = 7;

    pub const SLEEP: u8 = 6;

    pub const CYCLE: u8 = 5;

    pub const TEMP_DIS: u8 = 3;

    pub const CLKSEL: BitBlock = BitBlock { bit: 2, length: 3 };
}

#[allow(non_camel_case_types)]
#[derive(Copy, Clone, Debug)]

pub struct PWR_MGMT_2;

impl PWR_MGMT_2 {
    pub const ADDR: u8 = 0x6c;

    pub const LP_WAKE_CTRL: BitBlock = BitBlock { bit: 7, length: 2 };

    pub const STBY_XA: u8 = 5;

    pub const STBY_YA: u8 = 4;

    pub const STBY_ZA: u8 = 3;

    pub const STBY_XG: u8 = 2;

    pub const STBY_YG: u8 = 1;

    pub const STBY_ZG: u8 = 0;
}

#[allow(non_camel_case_types)]
#[derive(Copy, Clone, Debug, Eq, PartialEq)]

pub enum LP_WAKE_CTRL {
    _1P25 = 0,

    _2P5,

    _5,

    _10,
}

#[allow(non_camel_case_types)]
#[derive(Copy, Clone, Debug, Eq, PartialEq)]

pub enum ACCEL_HPF {
    _RESET = 0,

    _5 = 1,

    _2P5 = 2,

    _1P25 = 3,

    _0P63 = 4,

    _HOLD = 7,
}

impl From<u8> for ACCEL_HPF {
    fn from(range: u8) -> Self {
        match range {
            0 => ACCEL_HPF::_RESET,
            1 => ACCEL_HPF::_5,
            2 => ACCEL_HPF::_2P5,
            3 => ACCEL_HPF::_1P25,
            4 => ACCEL_HPF::_0P63,
            7 => ACCEL_HPF::_HOLD,
            _ => ACCEL_HPF::_RESET,
        }
    }
}

#[allow(non_camel_case_types)]
#[derive(Copy, Clone, Debug, Eq, PartialEq)]

pub enum CLKSEL {
    OSCILL = 0,

    GXAXIS = 1,

    GYAXIS = 2,

    GZAXIS = 3,

    EXT_32p7 = 4,

    EXT_19P2 = 5,

    RESERV = 6,

    STOP = 7,
}

impl From<u8> for CLKSEL {
    fn from(clk: u8) -> Self {
        match clk {
            0 => CLKSEL::OSCILL,
            1 => CLKSEL::GXAXIS,
            2 => CLKSEL::GYAXIS,
            3 => CLKSEL::GZAXIS,
            4 => CLKSEL::EXT_32p7,
            5 => CLKSEL::EXT_19P2,
            6 => CLKSEL::RESERV,
            7 => CLKSEL::STOP,
            _ => CLKSEL::GXAXIS,
        }
    }
}

#[derive(Debug, Eq, PartialEq, Copy, Clone)]
pub enum AccelRange {
    G2 = 0,

    G4,

    G8,

    G16,
}

#[derive(Debug, Eq, PartialEq, Copy, Clone)]
pub enum GyroRange {
    D250 = 0,

    D500,

    D1000,

    D2000,
}

impl From<u8> for GyroRange {
    fn from(range: u8) -> Self {
        match range {
            0 => GyroRange::D250,
            1 => GyroRange::D500,
            2 => GyroRange::D1000,
            3 => GyroRange::D2000,
            _ => GyroRange::D250,
        }
    }
}

impl From<u8> for AccelRange {
    fn from(range: u8) -> Self {
        match range {
            0 => AccelRange::G2,
            1 => AccelRange::G4,
            2 => AccelRange::G8,
            3 => AccelRange::G16,
            _ => AccelRange::G2,
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
