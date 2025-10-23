pub const MPU9150_RA_MAG_ADDRESS: u8 = 0x0C;
pub const MPU9150_RA_MAG_XOUT_L: u8 = 0x03;
pub const MPU9150_RA_MAG_XOUT_H: u8 = 0x04;
pub const MPU9150_RA_MAG_YOUT_L: u8 = 0x05;
pub const MPU9150_RA_MAG_YOUT_H: u8 = 0x06;
pub const MPU9150_RA_MAG_ZOUT_L: u8 = 0x07;
pub const MPU9150_RA_MAG_ZOUT_H: u8 = 0x08;

pub const MPU6050_ADDRESS_AD0_LOW: u8 = 0x68; // address pin low (GND), default for InvenSense evaluation board
pub const MPU6050_ADDRESS_AD0_HIGH: u8 = 0x69; // address pin high (VCC)
pub const MPU6050_DEFAULT_ADDRESS: u8 = MPU6050_ADDRESS_AD0_LOW;

pub const MPU6050_RA_XG_OFFS_TC: u8 = 0x00; //[7] PWR_MODE, [6:1] XG_OFFS_TC, [0] OTP_BNK_VLD
pub const MPU6050_RA_YG_OFFS_TC: u8 = 0x01; //[7] PWR_MODE, [6:1] YG_OFFS_TC, [0] OTP_BNK_VLD
pub const MPU6050_RA_ZG_OFFS_TC: u8 = 0x02; //[7] PWR_MODE, [6:1] ZG_OFFS_TC, [0] OTP_BNK_VLD
pub const MPU6050_RA_X_FINE_GAIN: u8 = 0x03; //[7:0] X_FINE_GAIN
pub const MPU6050_RA_Y_FINE_GAIN: u8 = 0x04; //[7:0] Y_FINE_GAIN
pub const MPU6050_RA_Z_FINE_GAIN: u8 = 0x05; //[7:0] Z_FINE_GAIN
pub const MPU6050_RA_XA_OFFS_H: u8 = 0x06; //[15:0] XA_OFFS
pub const MPU6050_RA_XA_OFFS_L_TC: u8 = 0x07;
pub const MPU6050_RA_YA_OFFS_H: u8 = 0x08; //[15:0] YA_OFFS
pub const MPU6050_RA_YA_OFFS_L_TC: u8 = 0x09;
pub const MPU6050_RA_ZA_OFFS_H: u8 = 0x0A; //[15:0] ZA_OFFS
pub const MPU6050_RA_ZA_OFFS_L_TC: u8 = 0x0B;
pub const MPU6050_RA_XG_OFFS_USRH: u8 = 0x13; //[15:0] XG_OFFS_USR
pub const MPU6050_RA_XG_OFFS_USRL: u8 = 0x14;
pub const MPU6050_RA_YG_OFFS_USRH: u8 = 0x15; //[15:0] YG_OFFS_USR
pub const MPU6050_RA_YG_OFFS_USRL: u8 = 0x16;
pub const MPU6050_RA_ZG_OFFS_USRH: u8 = 0x17; //[15:0] ZG_OFFS_USR
pub const MPU6050_RA_ZG_OFFS_USRL: u8 = 0x18;
pub const MPU6050_RA_SMPLRT_DIV: u8 = 0x19;
pub const MPU6050_RA_CONFIG: u8 = 0x1A;
pub const MPU6050_RA_GYRO_CONFIG: u8 = 0x1B;
pub const MPU6050_RA_ACCEL_CONFIG: u8 = 0x1C;
pub const MPU6050_RA_FF_THR: u8 = 0x1D;
pub const MPU6050_RA_FF_DUR: u8 = 0x1E;
pub const MPU6050_RA_MOT_THR: u8 = 0x1F;
pub const MPU6050_RA_MOT_DUR: u8 = 0x20;
pub const MPU6050_RA_ZRMOT_THR: u8 = 0x21;
pub const MPU6050_RA_ZRMOT_DUR: u8 = 0x22;
pub const MPU6050_RA_FIFO_EN: u8 = 0x23;
pub const MPU6050_RA_I2C_MST_CTRL: u8 = 0x24;
pub const MPU6050_RA_I2C_SLV0_ADDR: u8 = 0x25;
pub const MPU6050_RA_I2C_SLV0_REG: u8 = 0x26;
pub const MPU6050_RA_I2C_SLV0_CTRL: u8 = 0x27;
pub const MPU6050_RA_I2C_SLV1_ADDR: u8 = 0x28;
pub const MPU6050_RA_I2C_SLV1_REG: u8 = 0x29;
pub const MPU6050_RA_I2C_SLV1_CTRL: u8 = 0x2A;
pub const MPU6050_RA_I2C_SLV2_ADDR: u8 = 0x2B;
pub const MPU6050_RA_I2C_SLV2_REG: u8 = 0x2C;
pub const MPU6050_RA_I2C_SLV2_CTRL: u8 = 0x2D;
pub const MPU6050_RA_I2C_SLV3_ADDR: u8 = 0x2E;
pub const MPU6050_RA_I2C_SLV3_REG: u8 = 0x2F;
pub const MPU6050_RA_I2C_SLV3_CTRL: u8 = 0x30;
pub const MPU6050_RA_I2C_SLV4_ADDR: u8 = 0x31;
pub const MPU6050_RA_I2C_SLV4_REG: u8 = 0x32;
pub const MPU6050_RA_I2C_SLV4_DO: u8 = 0x33;
pub const MPU6050_RA_I2C_SLV4_CTRL: u8 = 0x34;
pub const MPU6050_RA_I2C_SLV4_DI: u8 = 0x35;
pub const MPU6050_RA_I2C_MST_STATUS: u8 = 0x36;
pub const MPU6050_RA_INT_PIN_CFG: u8 = 0x37;
pub const MPU6050_RA_INT_ENABLE: u8 = 0x38;
pub const MPU6050_RA_DMP_INT_STATUS: u8 = 0x39;
pub const MPU6050_RA_INT_STATUS: u8 = 0x3A;
pub const MPU6050_RA_ACCEL_XOUT_H: u8 = 0x3B;
pub const MPU6050_RA_ACCEL_XOUT_L: u8 = 0x3C;
pub const MPU6050_RA_ACCEL_YOUT_H: u8 = 0x3D;
pub const MPU6050_RA_ACCEL_YOUT_L: u8 = 0x3E;
pub const MPU6050_RA_ACCEL_ZOUT_H: u8 = 0x3F;
pub const MPU6050_RA_ACCEL_ZOUT_L: u8 = 0x40;
pub const MPU6050_RA_TEMP_OUT_H: u8 = 0x41;
pub const MPU6050_RA_TEMP_OUT_L: u8 = 0x42;
pub const MPU6050_RA_GYRO_XOUT_H: u8 = 0x43;
pub const MPU6050_RA_GYRO_XOUT_L: u8 = 0x44;
pub const MPU6050_RA_GYRO_YOUT_H: u8 = 0x45;
pub const MPU6050_RA_GYRO_YOUT_L: u8 = 0x46;
pub const MPU6050_RA_GYRO_ZOUT_H: u8 = 0x47;
pub const MPU6050_RA_GYRO_ZOUT_L: u8 = 0x48;
pub const MPU6050_RA_EXT_SENS_DATA_00: u8 = 0x49;
pub const MPU6050_RA_EXT_SENS_DATA_01: u8 = 0x4A;
pub const MPU6050_RA_EXT_SENS_DATA_02: u8 = 0x4B;
pub const MPU6050_RA_EXT_SENS_DATA_03: u8 = 0x4C;
pub const MPU6050_RA_EXT_SENS_DATA_04: u8 = 0x4D;
pub const MPU6050_RA_EXT_SENS_DATA_05: u8 = 0x4E;
pub const MPU6050_RA_EXT_SENS_DATA_06: u8 = 0x4F;
pub const MPU6050_RA_EXT_SENS_DATA_07: u8 = 0x50;
pub const MPU6050_RA_EXT_SENS_DATA_08: u8 = 0x51;
pub const MPU6050_RA_EXT_SENS_DATA_09: u8 = 0x52;
pub const MPU6050_RA_EXT_SENS_DATA_10: u8 = 0x53;
pub const MPU6050_RA_EXT_SENS_DATA_11: u8 = 0x54;
pub const MPU6050_RA_EXT_SENS_DATA_12: u8 = 0x55;
pub const MPU6050_RA_EXT_SENS_DATA_13: u8 = 0x56;
pub const MPU6050_RA_EXT_SENS_DATA_14: u8 = 0x57;
pub const MPU6050_RA_EXT_SENS_DATA_15: u8 = 0x58;
pub const MPU6050_RA_EXT_SENS_DATA_16: u8 = 0x59;
pub const MPU6050_RA_EXT_SENS_DATA_17: u8 = 0x5A;
pub const MPU6050_RA_EXT_SENS_DATA_18: u8 = 0x5B;
pub const MPU6050_RA_EXT_SENS_DATA_19: u8 = 0x5C;
pub const MPU6050_RA_EXT_SENS_DATA_20: u8 = 0x5D;
pub const MPU6050_RA_EXT_SENS_DATA_21: u8 = 0x5E;
pub const MPU6050_RA_EXT_SENS_DATA_22: u8 = 0x5F;
pub const MPU6050_RA_EXT_SENS_DATA_23: u8 = 0x60;
pub const MPU6050_RA_MOT_DETECT_STATUS: u8 = 0x61;
pub const MPU6050_RA_I2C_SLV0_DO: u8 = 0x63;
pub const MPU6050_RA_I2C_SLV1_DO: u8 = 0x64;
pub const MPU6050_RA_I2C_SLV2_DO: u8 = 0x65;
pub const MPU6050_RA_I2C_SLV3_DO: u8 = 0x66;
pub const MPU6050_RA_I2C_MST_DELAY_CTRL: u8 = 0x67;
pub const MPU6050_RA_SIGNAL_PATH_RESET: u8 = 0x68;
pub const MPU6050_RA_MOT_DETECT_CTRL: u8 = 0x69;
pub const MPU6050_RA_USER_CTRL: u8 = 0x6A;
pub const MPU6050_RA_PWR_MGMT_1: u8 = 0x6B;
pub const MPU6050_RA_PWR_MGMT_2: u8 = 0x6C;
pub const MPU6050_RA_BANK_SEL: u8 = 0x6D;
pub const MPU6050_RA_MEM_START_ADDR: u8 = 0x6E;
pub const MPU6050_RA_MEM_R_W: u8 = 0x6F;
pub const MPU6050_RA_DMP_CFG_1: u8 = 0x70;
pub const MPU6050_RA_DMP_CFG_2: u8 = 0x71;
pub const MPU6050_RA_FIFO_COUNTH: u8 = 0x72;
pub const MPU6050_RA_FIFO_COUNTL: u8 = 0x73;
pub const MPU6050_RA_FIFO_R_W: u8 = 0x74;
pub const MPU6050_RA_WHO_AM_I: u8 = 0x75;

pub const MPU6050_TC_PWR_MODE_BIT: u8 = 7;
pub const MPU6050_TC_OFFSET_BIT: u8 = 6;
pub const MPU6050_TC_OFFSET_LENGTH: u8 = 6;
pub const MPU6050_TC_OTP_BNK_VLD_BIT: u8 = 0;

pub const MPU6050_VDDIO_LEVEL_VLOGIC: u8 = 0;
pub const MPU6050_VDDIO_LEVEL_VDD: u8 = 1;

pub const MPU6050_CFG_EXT_SYNC_SET_BIT: u8 = 5;
pub const MPU6050_CFG_EXT_SYNC_SET_LENGTH: u8 = 3;
pub const MPU6050_CFG_DLPF_CFG_BIT: u8 = 2;
pub const MPU6050_CFG_DLPF_CFG_LENGTH: u8 = 3;

pub const MPU6050_EXT_SYNC_DISABLED: u8 = 0x0;
pub const MPU6050_EXT_SYNC_TEMP_OUT_L: u8 = 0x1;
pub const MPU6050_EXT_SYNC_GYRO_XOUT_L: u8 = 0x2;
pub const MPU6050_EXT_SYNC_GYRO_YOUT_L: u8 = 0x3;
pub const MPU6050_EXT_SYNC_GYRO_ZOUT_L: u8 = 0x4;
pub const MPU6050_EXT_SYNC_ACCEL_XOUT_L: u8 = 0x5;
pub const MPU6050_EXT_SYNC_ACCEL_YOUT_L: u8 = 0x6;
pub const MPU6050_EXT_SYNC_ACCEL_ZOUT_L: u8 = 0x7;

pub const MPU6050_DLPF_BW_256: u8 = 0x00;
pub const MPU6050_DLPF_BW_188: u8 = 0x01;
pub const MPU6050_DLPF_BW_98: u8 = 0x02;
pub const MPU6050_DLPF_BW_42: u8 = 0x03;
pub const MPU6050_DLPF_BW_20: u8 = 0x04;
pub const MPU6050_DLPF_BW_10: u8 = 0x05;
pub const MPU6050_DLPF_BW_5: u8 = 0x06;

pub const MPU6050_GCONFIG_FS_SEL_BIT: u8 = 4;
pub const MPU6050_GCONFIG_FS_SEL_LENGTH: u8 = 2;

pub const MPU6050_GYRO_FS_250: u8 = 0x00;
pub const MPU6050_GYRO_FS_500: u8 = 0x01;
pub const MPU6050_GYRO_FS_1000: u8 = 0x02;
pub const MPU6050_GYRO_FS_2000: u8 = 0x03;

pub const MPU6050_ACONFIG_XA_ST_BIT: u8 = 7;
pub const MPU6050_ACONFIG_YA_ST_BIT: u8 = 6;
pub const MPU6050_ACONFIG_ZA_ST_BIT: u8 = 5;
pub const MPU6050_ACONFIG_AFS_SEL_BIT: u8 = 4;
pub const MPU6050_ACONFIG_AFS_SEL_LENGTH: u8 = 2;
pub const MPU6050_ACONFIG_ACCEL_HPF_BIT: u8 = 2;
pub const MPU6050_ACONFIG_ACCEL_HPF_LENGTH: u8 = 3;

pub const MPU6050_ACCEL_FS_2: u8 = 0x00;
pub const MPU6050_ACCEL_FS_4: u8 = 0x01;
pub const MPU6050_ACCEL_FS_8: u8 = 0x02;
pub const MPU6050_ACCEL_FS_16: u8 = 0x03;

pub const MPU6050_DHPF_RESET: u8 = 0x00;
pub const MPU6050_DHPF_5: u8 = 0x01;
pub const MPU6050_DHPF_2P5: u8 = 0x02;
pub const MPU6050_DHPF_1P25: u8 = 0x03;
pub const MPU6050_DHPF_0P63: u8 = 0x04;
pub const MPU6050_DHPF_HOLD: u8 = 0x07;

pub const MPU6050_TEMP_FIFO_EN_BIT: u8 = 7;
pub const MPU6050_XG_FIFO_EN_BIT: u8 = 6;
pub const MPU6050_YG_FIFO_EN_BIT: u8 = 5;
pub const MPU6050_ZG_FIFO_EN_BIT: u8 = 4;
pub const MPU6050_ACCEL_FIFO_EN_BIT: u8 = 3;
pub const MPU6050_SLV2_FIFO_EN_BIT: u8 = 2;
pub const MPU6050_SLV1_FIFO_EN_BIT: u8 = 1;
pub const MPU6050_SLV0_FIFO_EN_BIT: u8 = 0;

pub const MPU6050_MULT_MST_EN_BIT: u8 = 7;
pub const MPU6050_WAIT_FOR_ES_BIT: u8 = 6;
pub const MPU6050_SLV_3_FIFO_EN_BIT: u8 = 5;
pub const MPU6050_I2C_MST_P_NSR_BIT: u8 = 4;
pub const MPU6050_I2C_MST_CLK_BIT: u8 = 3;
pub const MPU6050_I2C_MST_CLK_LENGTH: u8 = 4;

pub const MPU6050_CLOCK_DIV_348: u8 = 0x0;
pub const MPU6050_CLOCK_DIV_333: u8 = 0x1;
pub const MPU6050_CLOCK_DIV_320: u8 = 0x2;
pub const MPU6050_CLOCK_DIV_308: u8 = 0x3;
pub const MPU6050_CLOCK_DIV_296: u8 = 0x4;
pub const MPU6050_CLOCK_DIV_286: u8 = 0x5;
pub const MPU6050_CLOCK_DIV_276: u8 = 0x6;
pub const MPU6050_CLOCK_DIV_267: u8 = 0x7;
pub const MPU6050_CLOCK_DIV_258: u8 = 0x8;
pub const MPU6050_CLOCK_DIV_500: u8 = 0x9;
pub const MPU6050_CLOCK_DIV_471: u8 = 0xA;
pub const MPU6050_CLOCK_DIV_444: u8 = 0xB;
pub const MPU6050_CLOCK_DIV_421: u8 = 0xC;
pub const MPU6050_CLOCK_DIV_400: u8 = 0xD;
pub const MPU6050_CLOCK_DIV_381: u8 = 0xE;
pub const MPU6050_CLOCK_DIV_364: u8 = 0xF;

pub const MPU6050_I2C_SLV_RW_BIT: u8 = 7;
pub const MPU6050_I2C_SLV_ADDR_BIT: u8 = 6;
pub const MPU6050_I2C_SLV_ADDR_LENGTH: u8 = 7;
pub const MPU6050_I2C_SLV_EN_BIT: u8 = 7;
pub const MPU6050_I2C_SLV_BYTE_SW_BIT: u8 = 6;
pub const MPU6050_I2C_SLV_REG_DIS_BIT: u8 = 5;
pub const MPU6050_I2C_SLV_GRP_BIT: u8 = 4;
pub const MPU6050_I2C_SLV_LEN_BIT: u8 = 3;
pub const MPU6050_I2C_SLV_LEN_LENGTH: u8 = 4;

pub const MPU6050_I2C_SLV4_RW_BIT: u8 = 7;
pub const MPU6050_I2C_SLV4_ADDR_BIT: u8 = 6;
pub const MPU6050_I2C_SLV4_ADDR_LENGTH: u8 = 7;
pub const MPU6050_I2C_SLV4_EN_BIT: u8 = 7;
pub const MPU6050_I2C_SLV4_INT_EN_BIT: u8 = 6;
pub const MPU6050_I2C_SLV4_REG_DIS_BIT: u8 = 5;
pub const MPU6050_I2C_SLV4_MST_DLY_BIT: u8 = 4;
pub const MPU6050_I2C_SLV4_MST_DLY_LENGTH: u8 = 5;

pub const MPU6050_MST_PASS_THROUGH_BIT: u8 = 7;
pub const MPU6050_MST_I2C_SLV4_DONE_BIT: u8 = 6;
pub const MPU6050_MST_I2C_LOST_ARB_BIT: u8 = 5;
pub const MPU6050_MST_I2C_SLV4_NACK_BIT: u8 = 4;
pub const MPU6050_MST_I2C_SLV3_NACK_BIT: u8 = 3;
pub const MPU6050_MST_I2C_SLV2_NACK_BIT: u8 = 2;
pub const MPU6050_MST_I2C_SLV1_NACK_BIT: u8 = 1;
pub const MPU6050_MST_I2C_SLV0_NACK_BIT: u8 = 0;

pub const MPU6050_INTCFG_INT_LEVEL_BIT: u8 = 7;
pub const MPU6050_INTCFG_INT_OPEN_BIT: u8 = 6;
pub const MPU6050_INTCFG_LATCH_INT_EN_BIT: u8 = 5;
pub const MPU6050_INTCFG_INT_RD_CLEAR_BIT: u8 = 4;
pub const MPU6050_INTCFG_FSYNC_INT_LEVEL_BIT: u8 = 3;
pub const MPU6050_INTCFG_FSYNC_INT_EN_BIT: u8 = 2;
pub const MPU6050_INTCFG_I2C_BYPASS_EN_BIT: u8 = 1;
pub const MPU6050_INTCFG_CLKOUT_EN_BIT: u8 = 0;

pub const MPU6050_INTMODE_ACTIVEHIGH: u8 = 0x00;
pub const MPU6050_INTMODE_ACTIVELOW: u8 = 0x01;

pub const MPU6050_INTDRV_PUSHPULL: u8 = 0x00;
pub const MPU6050_INTDRV_OPENDRAIN: u8 = 0x01;

pub const MPU6050_INTLATCH_50USPULSE: u8 = 0x00;
pub const MPU6050_INTLATCH_WAITCLEAR: u8 = 0x01;

pub const MPU6050_INTCLEAR_STATUSREAD: u8 = 0x00;
pub const MPU6050_INTCLEAR_ANYREAD: u8 = 0x01;

pub const MPU6050_INTERRUPT_FF_BIT: u8 = 7;
pub const MPU6050_INTERRUPT_MOT_BIT: u8 = 6;
pub const MPU6050_INTERRUPT_ZMOT_BIT: u8 = 5;
pub const MPU6050_INTERRUPT_FIFO_OFLOW_BIT: u8 = 4;
pub const MPU6050_INTERRUPT_I2C_MST_INT_BIT: u8 = 3;
pub const MPU6050_INTERRUPT_PLL_RDY_INT_BIT: u8 = 2;
pub const MPU6050_INTERRUPT_DMP_INT_BIT: u8 = 1;
pub const MPU6050_INTERRUPT_DATA_RDY_BIT: u8 = 0;

// TODO: figure out what these actually do
// UMPL source code is not very obivous
pub const MPU6050_DMPINT_5_BIT: u8 = 5;
pub const MPU6050_DMPINT_4_BIT: u8 = 4;
pub const MPU6050_DMPINT_3_BIT: u8 = 3;
pub const MPU6050_DMPINT_2_BIT: u8 = 2;
pub const MPU6050_DMPINT_1_BIT: u8 = 1;
pub const MPU6050_DMPINT_0_BIT: u8 = 0;

pub const MPU6050_MOTION_MOT_XNEG_BIT: u8 = 7;
pub const MPU6050_MOTION_MOT_XPOS_BIT: u8 = 6;
pub const MPU6050_MOTION_MOT_YNEG_BIT: u8 = 5;
pub const MPU6050_MOTION_MOT_YPOS_BIT: u8 = 4;
pub const MPU6050_MOTION_MOT_ZNEG_BIT: u8 = 3;
pub const MPU6050_MOTION_MOT_ZPOS_BIT: u8 = 2;
pub const MPU6050_MOTION_MOT_ZRMOT_BIT: u8 = 0;

pub const MPU6050_DELAYCTRL_DELAY_ES_SHADOW_BIT: u8 = 7;
pub const MPU6050_DELAYCTRL_I2C_SLV4_DLY_EN_BIT: u8 = 4;
pub const MPU6050_DELAYCTRL_I2C_SLV3_DLY_EN_BIT: u8 = 3;
pub const MPU6050_DELAYCTRL_I2C_SLV2_DLY_EN_BIT: u8 = 2;
pub const MPU6050_DELAYCTRL_I2C_SLV1_DLY_EN_BIT: u8 = 1;
pub const MPU6050_DELAYCTRL_I2C_SLV0_DLY_EN_BIT: u8 = 0;

pub const MPU6050_PATHRESET_GYRO_RESET_BIT: u8 = 2;
pub const MPU6050_PATHRESET_ACCEL_RESET_BIT: u8 = 1;
pub const MPU6050_PATHRESET_TEMP_RESET_BIT: u8 = 0;

pub const MPU6050_DETECT_ACCEL_ON_DELAY_BIT: u8 = 5;
pub const MPU6050_DETECT_ACCEL_ON_DELAY_LENGTH: u8 = 2;
pub const MPU6050_DETECT_FF_COUNT_BIT: u8 = 3;
pub const MPU6050_DETECT_FF_COUNT_LENGTH: u8 = 2;
pub const MPU6050_DETECT_MOT_COUNT_BIT: u8 = 1;
pub const MPU6050_DETECT_MOT_COUNT_LENGTH: u8 = 2;

pub const MPU6050_DETECT_DECREMENT_RESET: u8 = 0x0;
pub const MPU6050_DETECT_DECREMENT_1: u8 = 0x1;
pub const MPU6050_DETECT_DECREMENT_2: u8 = 0x2;
pub const MPU6050_DETECT_DECREMENT_4: u8 = 0x3;

pub const MPU6050_USERCTRL_DMP_EN_BIT: u8 = 7;
pub const MPU6050_USERCTRL_FIFO_EN_BIT: u8 = 6;
pub const MPU6050_USERCTRL_I2C_MST_EN_BIT: u8 = 5;
pub const MPU6050_USERCTRL_I2C_IF_DIS_BIT: u8 = 4;
pub const MPU6050_USERCTRL_DMP_RESET_BIT: u8 = 3;
pub const MPU6050_USERCTRL_FIFO_RESET_BIT: u8 = 2;
pub const MPU6050_USERCTRL_I2C_MST_RESET_BIT: u8 = 1;
pub const MPU6050_USERCTRL_SIG_COND_RESET_BIT: u8 = 0;

pub const MPU6050_PWR1_DEVICE_RESET_BIT: u8 = 7;
pub const MPU6050_PWR1_SLEEP_BIT: u8 = 6;
pub const MPU6050_PWR1_CYCLE_BIT: u8 = 5;
pub const MPU6050_PWR1_TEMP_DIS_BIT: u8 = 3;
pub const MPU6050_PWR1_CLKSEL_BIT: u8 = 2;
pub const MPU6050_PWR1_CLKSEL_LENGTH: u8 = 3;

pub const MPU6050_CLOCK_INTERNAL: u8 = 0x00;
pub const MPU6050_CLOCK_PLL_XGYRO: u8 = 0x01;
pub const MPU6050_CLOCK_PLL_YGYRO: u8 = 0x02;
pub const MPU6050_CLOCK_PLL_ZGYRO: u8 = 0x03;
pub const MPU6050_CLOCK_PLL_EXT32K: u8 = 0x04;
pub const MPU6050_CLOCK_PLL_EXT19M: u8 = 0x05;
pub const MPU6050_CLOCK_KEEP_RESET: u8 = 0x07;

pub const MPU6050_PWR2_LP_WAKE_CTRL_BIT: u8 = 7;
pub const MPU6050_PWR2_LP_WAKE_CTRL_LENGTH: u8 = 2;
pub const MPU6050_PWR2_STBY_XA_BIT: u8 = 5;
pub const MPU6050_PWR2_STBY_YA_BIT: u8 = 4;
pub const MPU6050_PWR2_STBY_ZA_BIT: u8 = 3;
pub const MPU6050_PWR2_STBY_XG_BIT: u8 = 2;
pub const MPU6050_PWR2_STBY_YG_BIT: u8 = 1;
pub const MPU6050_PWR2_STBY_ZG_BIT: u8 = 0;

pub const MPU6050_WAKE_FREQ_1P25: u8 = 0x0;
pub const MPU6050_WAKE_FREQ_2P5: u8 = 0x1;
pub const MPU6050_WAKE_FREQ_5: u8 = 0x2;
pub const MPU6050_WAKE_FREQ_10: u8 = 0x3;

pub const MPU6050_BANKSEL_PRFTCH_EN_BIT: u8 = 6;
pub const MPU6050_BANKSEL_CFG_USER_BANK_BIT: u8 = 5;
pub const MPU6050_BANKSEL_MEM_SEL_BIT: u8 = 4;
pub const MPU6050_BANKSEL_MEM_SEL_LENGTH: u8 = 5;

pub const MPU6050_WHO_AM_I_BIT: u8 = 6;
pub const MPU6050_WHO_AM_I_LENGTH: u8 = 6;

pub const MPU6050_DMP_MEMORY_BANKS: u8 = 8;
pub const MPU6050_DMP_MEMORY_BANK_SIZE: u16 = 256;
pub const MPU6050_DMP_MEMORY_CHUNK_SIZE: u8 = 16;

// When set to 1, this bit resets all internal registers to their
// default values. The bit automatically clears to 0 after the reset is complete.
pub enum DeviceReset {
    /// Reset the device.
    Reset = 0x01,
}

impl MPU6050BitField for DeviceReset {
    fn addr() -> u8 {
        MPU6050_RA_PWR_MGMT_1
    }

    fn location() -> u8 {
        MPU6050_PWR1_DEVICE_RESET_BIT
    }

    fn from(value: u8) -> Self {
        if value & Self::mask() != 0 {
            DeviceReset::Reset
        } else {
            panic!("Invalid device reset value")
        }
    }

    fn to_value(&self) -> u8 {
        match self {
            DeviceReset::Reset => 0x01,
        }
    }
}

/// When set to 1, this bit puts the device into sleep mode.
pub enum SleepMode {
    /// Device is in sleep mode.
    Sleep = 0x01,
    /// Device is not in sleep mode.
    WakeUp = 0x00,
}

impl MPU6050BitField for SleepMode {
    fn addr() -> u8 {
        MPU6050_RA_PWR_MGMT_1
    }

    fn location() -> u8 {
        MPU6050_PWR1_SLEEP_BIT
    }

    fn from(value: u8) -> Self {
        if value & Self::mask() != 0 {
            SleepMode::Sleep
        } else {
            SleepMode::WakeUp
        }
    }

    fn to_value(&self) -> u8 {
        match self {
            SleepMode::Sleep => 0x01,
            SleepMode::WakeUp => 0x00,
        }
    }
}

/// When set to 1, and sleep is disabled, the MPU-60X0 will cycle
/// between sleep mode and waking up to take a single measurement
/// from active sensors at a rate determined by the LP_WAKE_CTRL.
pub enum Cycle {
    Cycle = 0x01,
    NoCycle = 0x00,
}

impl MPU6050BitField for Cycle {
    fn addr() -> u8 {
        MPU6050_RA_PWR_MGMT_1
    }

    fn location() -> u8 {
        MPU6050_PWR1_CYCLE_BIT
    }

    fn from(value: u8) -> Self {
        if value & Self::mask() != 0 {
            Cycle::Cycle
        } else {
            Cycle::NoCycle
        }
    }

    fn to_value(&self) -> u8 {
        match self {
            Cycle::Cycle => 0x01,
            Cycle::NoCycle => 0x00,
        }
    }
}

/// When set to 1, this bit disables the temperature sensor.
pub enum TempDisable {
    /// Disable the temperature sensor.
    Disable = 0x01,
    /// Enable the temperature sensor.
    Enable = 0x00,
}

impl MPU6050BitField for TempDisable {
    fn addr() -> u8 {
        MPU6050_RA_PWR_MGMT_1
    }

    fn location() -> u8 {
        MPU6050_PWR1_TEMP_DIS_BIT
    }

    fn from(value: u8) -> Self {
        if value & Self::mask() != 0 {
            TempDisable::Disable
        } else {
            TempDisable::Enable
        }
    }

    fn to_value(&self) -> u8 {
        match self {
            TempDisable::Disable => 0x01,
            TempDisable::Enable => 0x00,
        }
    }
}

/// Clock source setting.
/// An internal 8MHz oscillator, gyroscope based clock, or external sources can
/// be selected as the MPU-60X0 clock source. When the internal 8 MHz oscillator
/// or an external source is chosen as the clock source, the MPU-60X0 can operate
/// in low power modes with the gyroscopes disabled.
///
/// Upon power up, the MPU-60X0 clock source defaults to the internal oscillator.
/// However, it is highly recommended that the device be configured to use one of
/// the gyroscopes (or an external clock source) as the clock reference for
/// improved stability. The clock source can be selected according to the following table:
///
/// ```
/// CLK_SEL | Clock Source
/// --------+--------------------------------------
/// 0       | Internal oscillator
/// 1       | PLL with X Gyro reference
/// 2       | PLL with Y Gyro reference
/// 3       | PLL with Z Gyro reference
/// 4       | PLL with external 32.768kHz reference
/// 5       | PLL with external 19.2MHz reference
/// 6       | Reserved
/// 7       | Stops the clock and keeps the timing generator in reset
/// ```
pub enum MPUClkSource {
    /// MPU6050 clock source is set to Internal oscillator, 8MHz.
    InternalOscillator = 0x00,
    /// MPU6050 clock source is set to PLL with X Gyro reference.
    PLLWithXGyro = 0x01,
    /// MPU6050 clock source is set to PLL with Y Gyro reference.
    PLLWithYGyro = 0x02,
    /// MPU6050 clock source is set to PLL with Z Gyro reference.
    PLLWithZGyro = 0x03,
    /// MPU6050 clock source is set to PLL with external 32.768kHz reference.
    PLLWithExternal32kHz = 0x04,
    /// MPU6050 clock source is set to PLL with external 19.2MHz reference.
    PLLWithExternal19_2MHz = 0x05,
    /// Reserved clock source.
    StopClockAndResetTimingGenerator = 0x07,
}

impl MPU6050BitField for MPUClkSource {
    fn addr() -> u8 {
        MPU6050_RA_PWR_MGMT_1
    }

    fn location() -> u8 {
        MPU6050_PWR1_CLKSEL_BIT
    }

    fn length() -> u8 {
        MPU6050_PWR1_CLKSEL_LENGTH
    }

    fn mask() -> u8 {
        (1 << MPU6050_PWR1_CLKSEL_LENGTH) - 1
    }

    fn from(value: u8) -> Self {
        match value {
            0x00 => MPUClkSource::InternalOscillator,
            0x01 => MPUClkSource::PLLWithXGyro,
            0x02 => MPUClkSource::PLLWithYGyro,
            0x03 => MPUClkSource::PLLWithZGyro,
            0x04 => MPUClkSource::PLLWithExternal32kHz,
            0x05 => MPUClkSource::PLLWithExternal19_2MHz,
            0x07 => MPUClkSource::StopClockAndResetTimingGenerator,
            _ => panic!("Invalid clock source value"),
        }
    }

    fn to_value(&self) -> u8 {
        match self {
            MPUClkSource::InternalOscillator => 0x00,
            MPUClkSource::PLLWithXGyro => 0x01,
            MPUClkSource::PLLWithYGyro => 0x02,
            MPUClkSource::PLLWithZGyro => 0x03,
            MPUClkSource::PLLWithExternal32kHz => 0x04,
            MPUClkSource::PLLWithExternal19_2MHz => 0x05,
            MPUClkSource::StopClockAndResetTimingGenerator => 0x07,
        }
    }
}

pub enum GyroFullScaleRange {
    /// Full scale range of ±250 degrees/second.
    FS250 = 0x00,
    /// Full scale range of ±500 degrees/second.
    FS500 = 0x01,
    /// Full scale range of ±1000 degrees/second.
    FS1000 = 0x02,
    /// Full scale range of ±2000 degrees/second.
    FS2000 = 0x03,
}

impl MPU6050BitField for GyroFullScaleRange {
    fn addr() -> u8 {
        MPU6050_RA_GYRO_CONFIG
    }

    fn location() -> u8 {
        MPU6050_GCONFIG_FS_SEL_BIT
    }

    fn length() -> u8 {
        MPU6050_GCONFIG_FS_SEL_LENGTH
    }

    fn mask() -> u8 {
        (1 << MPU6050_GCONFIG_FS_SEL_LENGTH) - 1
    }

    fn from(value: u8) -> Self {
        match value {
            0x00 => GyroFullScaleRange::FS250,
            0x01 => GyroFullScaleRange::FS500,
            0x02 => GyroFullScaleRange::FS1000,
            0x03 => GyroFullScaleRange::FS2000,
            _ => panic!("Invalid gyro full scale range value"),
        }
    }

    fn to_value(&self) -> u8 {
        match self {
            GyroFullScaleRange::FS250 => 0x00,
            GyroFullScaleRange::FS500 => 0x01,
            GyroFullScaleRange::FS1000 => 0x02,
            GyroFullScaleRange::FS2000 => 0x03,
        }
    }
}

pub enum AccelFullScaleRange {
    /// Full scale range of ±2g.
    FS2 = 0x00,
    /// Full scale range of ±4g.
    FS4 = 0x01,
    /// Full scale range of ±8g.
    FS8 = 0x02,
    /// Full scale range of ±16g.
    FS16 = 0x03,
}

impl MPU6050BitField for AccelFullScaleRange {
    fn addr() -> u8 {
        MPU6050_RA_ACCEL_CONFIG
    }

    fn location() -> u8 {
        MPU6050_ACONFIG_AFS_SEL_BIT
    }

    fn length() -> u8 {
        MPU6050_ACONFIG_AFS_SEL_LENGTH
    }

    fn mask() -> u8 {
        (1 << MPU6050_ACONFIG_AFS_SEL_LENGTH) - 1
    }

    fn from(value: u8) -> Self {
        match value {
            0x00 => AccelFullScaleRange::FS2,
            0x01 => AccelFullScaleRange::FS4,
            0x02 => AccelFullScaleRange::FS8,
            0x03 => AccelFullScaleRange::FS16,
            _ => panic!("Invalid accel full scale range value"),
        }
    }

    fn to_value(&self) -> u8 {
        match self {
            AccelFullScaleRange::FS2 => 0x00,
            AccelFullScaleRange::FS4 => 0x01,
            AccelFullScaleRange::FS8 => 0x02,
            AccelFullScaleRange::FS16 => 0x03,
        }
    }
}

pub struct WhoAmI {
    pub value: u8,
}

impl MPU6050BitField for WhoAmI {
    fn addr() -> u8 {
        MPU6050_RA_WHO_AM_I
    }

    fn location() -> u8 {
        MPU6050_WHO_AM_I_BIT
    }

    fn length() -> u8 {
        MPU6050_WHO_AM_I_LENGTH
    }

    fn mask() -> u8 {
        (1 << MPU6050_WHO_AM_I_LENGTH) - 1
    }

    fn from(value: u8) -> Self {
        WhoAmI { value }
    }

    fn to_value(&self) -> u8 {
        self.value
    }
}

pub trait MPU6050BitField {
    fn addr() -> u8;
    fn location() -> u8;
    fn length() -> u8 {
        1 // Default length is 1 bit
    }
    fn mask() -> u8 {
        (1 << Self::length()) - 1 // Default mask for the field
    }
    fn from(value: u8) -> Self; // Convert raw value to enum variant
    fn to_value(&self) -> u8;
}
