
// I2C for the Acceleration and the  Gyro                                    //
//softi2c addition
pub const NO_INTERRUPT: u8 = 1;
pub const I2C_TIMEOUT: u32 = 1000;

pub const SDA_PORT: str = PORTB;
pub const SDA_PIN: u8 = 2;
pub const SCL_PORT: str = PORTD;
pub const SCL_PIN: u8 = 3;
pub const FAC: u8 = 1;
pub const I2C_CPUFREQ: u8 = (F_CPU/FAC);

// MPU6050 SETTING for the Acceleration and the Gyro                         //
//-----test variables
pub const MPU6050_AUX_VDDIO: char =          0x01;   // R/W;
pub const MPU6050_SMPLRT_DIV: char =         0x19;   // R/W;
pub const MPU6050_CONFIG: char =             0x1A;   // R/W;
pub const MPU6050_GYRO_CONFIG: char =        0x1B;   // R/W;
pub const MPU6050_ACCEL_CONFIG: char =       0x1C;   // R/W;
pub const MPU6050_FF_THR: char =             0x1D;   // R/W;
pub const MPU6050_FF_DUR: char =             0x1E;   // R/W;
pub const MPU6050_MOT_THR: char =            0x1F;   // R/W;
pub const MPU6050_MOT_DUR: char =            0x20;   // R/W;
pub const MPU6050_ZRMOT_THR: char =          0x21;   // R/W;
pub const MPU6050_ZRMOT_DUR: char =          0x22;   // R/W;
pub const MPU6050_FIFO_EN: char =            0x23;   // R/W;
pub const MPU6050_I2C_MST_CTRL: char =       0x24;   // R/W;
pub const MPU6050_I2C_SLV0_ADDR: char =      0x25;   // R/W;
pub const MPU6050_I2C_SLV0_REG: char =       0x26;   // R/W;
pub const MPU6050_I2C_SLV0_CTRL: char =      0x27;   // R/W;
pub const MPU6050_I2C_SLV1_ADDR: char =      0x28;   // R/W;
pub const MPU6050_I2C_SLV1_REG: char =       0x29;   // R/W;
pub const MPU6050_I2C_SLV1_CTRL: char =      0x2A;   // R/W;
pub const MPU6050_I2C_SLV2_ADDR: char =      0x2B;   // R/W;
pub const MPU6050_I2C_SLV2_REG: char =       0x2C;   // R/W;
pub const MPU6050_I2C_SLV2_CTRL: char =      0x2D;   // R/W;
pub const MPU6050_I2C_SLV3_ADDR: char =      0x2E;   // R/W;
pub const MPU6050_I2C_SLV3_REG: char =       0x2F;   // R/W;
pub const MPU6050_I2C_SLV3_CTRL: char =      0x30;   // R/W;
pub const MPU6050_I2C_SLV4_ADDR: char =      0x31;   // R/W;
pub const MPU6050_I2C_SLV4_REG: char =       0x32;   // R/W;
pub const MPU6050_I2C_SLV4_DO: char =        0x33;   // R/W;
pub const MPU6050_I2C_SLV4_CTRL: char =      0x34;   // R/W;
pub const MPU6050_I2C_SLV4_DI: char =        0x35;   // R;
pub const MPU6050_I2C_MST_STATUS: char =     0x36;   // R;
pub const MPU6050_INT_PIN_CFG: char =        0x37;   // R/W;
pub const MPU6050_INT_ENABLE: char =         0x38;   // R/W;
pub const MPU6050_INT_STATUS: char =         0x3A;   // R;
pub const MPU6050_ACCEL_XOUT_H: char =       0x3B;   // R;
pub const MPU6050_ACCEL_XOUT_L: char =       0x3C;   // R;
pub const MPU6050_ACCEL_YOUT_H: char =       0x3D;   // R;
pub const MPU6050_ACCEL_YOUT_L: char =       0x3E;   // R;
pub const MPU6050_ACCEL_ZOUT_H: char =       0x3F;   // R;
pub const MPU6050_ACCEL_ZOUT_L: char =       0x40;   // R;
pub const MPU6050_TEMP_OUT_H: char =         0x41;   // R;
pub const MPU6050_TEMP_OUT_L: char =         0x42;   // R;
pub const MPU6050_GYRO_XOUT_H: char =        0x43;   // R;
pub const MPU6050_GYRO_XOUT_L: char =        0x44;   // R;
pub const MPU6050_GYRO_YOUT_H: char =        0x45;   // R;
pub const MPU6050_GYRO_YOUT_L: char =        0x46;   // R;
pub const MPU6050_GYRO_ZOUT_H: char =        0x47;   // R;
pub const MPU6050_GYRO_ZOUT_L: char =        0x48;   // R;
pub const MPU6050_EXT_SENS_DATA_00: char =   0x49;   // R;
pub const MPU6050_EXT_SENS_DATA_01: char =   0x4A;   // R;
pub const MPU6050_EXT_SENS_DATA_02: char =   0x4B;   // R;
pub const MPU6050_EXT_SENS_DATA_03: char =   0x4C;   // R;
pub const MPU6050_EXT_SENS_DATA_04: char =   0x4D;   // R;
pub const MPU6050_EXT_SENS_DATA_05: char =   0x4E;   // R;
pub const MPU6050_EXT_SENS_DATA_06: char =   0x4F;   // R;
pub const MPU6050_EXT_SENS_DATA_07: char =   0x50;   // R;
pub const MPU6050_EXT_SENS_DATA_08: char =   0x51;   // R;
pub const MPU6050_EXT_SENS_DATA_09: char =   0x52;   // R;
pub const MPU6050_EXT_SENS_DATA_10: char =   0x53;   // R;
pub const MPU6050_EXT_SENS_DATA_11: char =   0x54;   // R;
pub const MPU6050_EXT_SENS_DATA_12: char =   0x55;   // R;
pub const MPU6050_EXT_SENS_DATA_13: char =   0x56;   // R;
pub const MPU6050_EXT_SENS_DATA_14: char =   0x57;   // R;
pub const MPU6050_EXT_SENS_DATA_15: char =   0x58;   // R;
pub const MPU6050_EXT_SENS_DATA_16: char =   0x59;   // R;
pub const MPU6050_EXT_SENS_DATA_17: char =   0x5A;   // R;
pub const MPU6050_EXT_SENS_DATA_18: char =   0x5B;   // R;
pub const MPU6050_EXT_SENS_DATA_19: char =   0x5C;   // R;
pub const MPU6050_EXT_SENS_DATA_20: char =   0x5D;   // R;
pub const MPU6050_EXT_SENS_DATA_21: char =   0x5E;   // R;
pub const MPU6050_EXT_SENS_DATA_22: char =   0x5F;   // R;
pub const MPU6050_EXT_SENS_DATA_23: char =   0x60;   // R;
pub const MPU6050_MOT_DETECT_STATUS: char =  0x61;   // R;
pub const MPU6050_I2C_SLV0_DO: char =        0x63;   // R/W;
pub const MPU6050_I2C_SLV1_DO: char =        0x64;   // R/W;
pub const MPU6050_I2C_SLV2_DO: char =        0x65;   // R/W;
pub const MPU6050_I2C_SLV3_DO: char =        0x66;   // R/W;
pub const MPU6050_I2C_MST_DELAY_CTRL: char = 0x67;   // R/W;
pub const MPU6050_SIGNAL_PATH_RESET: char =  0x68;   // R/W;
pub const MPU6050_MOT_DETECT_CTRL: char =    0x69;   // R/W;
pub const MPU6050_USER_CTRL: char =          0x6A;   // R/W;
pub const MPU6050_PWR_MGMT_1: char =         0x6B;   // R/W;
pub const MPU6050_PWR_MGMT_2: char =         0x6C;   // R/W;
pub const MPU6050_FIFO_COUNTH: char =        0x72;   // R/W;
pub const MPU6050_FIFO_COUNTL: char =        0x73;   // R/W;
pub const MPU6050_FIFO_R_W: char =           0x74;   // R/W;
pub const MPU6050_WHO_AM_I: char =           0x75;   // R;

pub const MPU6050_D0: u8 = 0;
pub const MPU6050_D1: u8 = 1;
pub const MPU6050_D2: u8 = 2;
pub const MPU6050_D3: u8 = 3;
pub const MPU6050_D4: u8 = 4;
pub const MPU6050_D5: u8 = 5;
pub const MPU6050_D6: u8 = 6;
pub const MPU6050_D7: u8 = 7;
pub const MPU6050_D8: u8 = 8;

// Default I2C address for the MPU-6050 is 0x68, this is the Arno Motion Address.
pub const MPU6050_I2C_ADDRESS: char = 0xD0; //68;


// Kalman for the Acceleration and the  Gyro
pub const RESTRICT_PITCH: bool = true; // Comment out to restrict roll to ±90deg instead - please read: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf

// VIBRATION MOTOR SETTINGS
pub const vibrationMotor_PIN: u8 = 13;	//(MoterV);

// Photo-reflector PINs Setup
pub const PR_CH_NUM: u8 = 8;      // Number of Channels;
pub const PR_LED_PIN: u8 = 8;

// EMS Multiplexer PINs Setup
pub const EMS_EN_PIN: u8 = 7;
pub const EMS_S0_PIN: u8 = 4;
pub const EMS_S1_PIN: u8 = 5;
pub const EMS_S2_PIN: u8 = 6;

// EMS BOOSTER SETUP
pub const MAX_VOL: u8 = 12; //relation_V_Delay.length;
pub const MIN_VOL: u8 = 0;
pub const BOOSTER_SWITCH_PIN: u8 = 9; //switch pin :;
pub const relation_V_Delay: [ u8; 13 ] = [ 60, 40, 30, 22, 17, 13, 10, 7, 5, 4, 3, 2, 1 ];

// EMS PATTERN SETUP
pub const DEF_STIMU_TIME_COUNT: u16 = 200; //200
pub const DEF_STIMU_HIGH_WID: u16 = 200; //3000(3ms)<DEF_STIMU_LOW_WID
pub const DEF_STIMU_LOW_WID: u32 = 22500; //#define DEF_STIMU_LOW_WID 12500
