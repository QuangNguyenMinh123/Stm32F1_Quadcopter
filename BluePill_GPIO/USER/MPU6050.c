#include "MPU6050.h"
#include "I2C.h"
#include "GPIO.h"
#include "CLOCK.h"
#include <math.h>
/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define MPU6050_Raw_DATA_TYPE	signed short int
#define CALIBRATION_TIMES		500

#define XGOFFS_TC        0x00 // Bit 7 PWR_MODE, bits 6:1 XG_OFFS_TC, bit 0 OTP_BNK_VLD
#define YGOFFS_TC        0x01
#define ZGOFFS_TC        0x02
#define X_FINE_GAIN      0x03 // [7:0] fine gain
#define Y_FINE_GAIN      0x04
#define Z_FINE_GAIN      0x05
#define XA_OFFSET_H      0x06 // User-defined trim values for accelerometer
#define XA_OFFSET_L_TC   0x07
#define YA_OFFSET_H      0x08
#define YA_OFFSET_L_TC   0x09
#define ZA_OFFSET_H      0x0A
#define ZA_OFFSET_L_TC   0x0B
#define SELF_TEST_X      0x0D
#define SELF_TEST_Y      0x0E
#define SELF_TEST_Z      0x0F
#define SELF_TEST_A      0x10
#define XG_OFFS_USRH     0x13  // User-defined trim values for gyroscope; supported in MPU-6050?
#define XG_OFFS_USRL     0x14
#define YG_OFFS_USRH     0x15
#define YG_OFFS_USRL     0x16
#define ZG_OFFS_USRH     0x17
#define ZG_OFFS_USRL     0x18
#define SMPLRT_DIV       0x19
#define CONFIG           0x1A
#define GYRO_CONFIG      0x1B
#define ACCEL_CONFIG     0x1C
#define FF_THR           0x1D  // Free-fall
#define FF_DUR           0x1E  // Free-fall
#define MOT_THR          0x1F  // Motion detection threshold bits [7:0]
#define MOT_DUR          0x20  // Duration counter threshold for motion interrupt generation, 1 kHz rate, LSB = 1 ms
#define ZMOT_THR         0x21  // Zero-motion detection threshold bits [7:0]
#define ZRMOT_DUR        0x22  // Duration counter threshold for zero motion interrupt generation, 16 Hz rate, LSB = 64 ms
#define FIFO_EN          0x23
#define I2C_MST_CTRL     0x24
#define I2C_SLV0_ADDR    0x25
#define I2C_SLV0_REG     0x26
#define I2C_SLV0_CTRL    0x27
#define I2C_SLV1_ADDR    0x28
#define I2C_SLV1_REG     0x29
#define I2C_SLV1_CTRL    0x2A
#define I2C_SLV2_ADDR    0x2B
#define I2C_SLV2_REG     0x2C
#define I2C_SLV2_CTRL    0x2D
#define I2C_SLV3_ADDR    0x2E
#define I2C_SLV3_REG     0x2F
#define I2C_SLV3_CTRL    0x30
#define I2C_SLV4_ADDR    0x31
#define I2C_SLV4_REG     0x32
#define I2C_SLV4_DO      0x33
#define I2C_SLV4_CTRL    0x34
#define I2C_SLV4_DI      0x35
#define I2C_MST_STATUS   0x36
#define INT_PIN_CFG      0x37
#define INT_ENABLE       0x38
#define DMP_INT_STATUS   0x39  // Check DMP interrupt
#define INT_STATUS       0x3A
#define ACCEL_XOUT_H     0x3B
#define ACCEL_XOUT_L     0x3C
#define ACCEL_YOUT_H     0x3D
#define ACCEL_YOUT_L     0x3E
#define ACCEL_ZOUT_H     0x3F
#define ACCEL_ZOUT_L     0x40
#define TEMP_OUT_H       0x41
#define TEMP_OUT_L       0x42
#define GYRO_XOUT_H      0x43
#define GYRO_XOUT_L      0x44
#define GYRO_YOUT_H      0x45
#define GYRO_YOUT_L      0x46
#define GYRO_ZOUT_H      0x47
#define GYRO_ZOUT_L      0x48
#define EXT_SENS_DATA_00 0x49
#define EXT_SENS_DATA_01 0x4A
#define EXT_SENS_DATA_02 0x4B
#define EXT_SENS_DATA_03 0x4C
#define EXT_SENS_DATA_04 0x4D
#define EXT_SENS_DATA_05 0x4E
#define EXT_SENS_DATA_06 0x4F
#define EXT_SENS_DATA_07 0x50
#define EXT_SENS_DATA_08 0x51
#define EXT_SENS_DATA_09 0x52
#define EXT_SENS_DATA_10 0x53
#define EXT_SENS_DATA_11 0x54
#define EXT_SENS_DATA_12 0x55
#define EXT_SENS_DATA_13 0x56
#define EXT_SENS_DATA_14 0x57
#define EXT_SENS_DATA_15 0x58
#define EXT_SENS_DATA_16 0x59
#define EXT_SENS_DATA_17 0x5A
#define EXT_SENS_DATA_18 0x5B
#define EXT_SENS_DATA_19 0x5C
#define EXT_SENS_DATA_20 0x5D
#define EXT_SENS_DATA_21 0x5E
#define EXT_SENS_DATA_22 0x5F
#define EXT_SENS_DATA_23 0x60
#define MOT_DETECT_STATUS 0x61
#define I2C_SLV0_DO      0x63
#define I2C_SLV1_DO      0x64
#define I2C_SLV2_DO      0x65
#define I2C_SLV3_DO      0x66
#define I2C_MST_DELAY_CTRL 0x67
#define SIGNAL_PATH_RESET  0x68
#define MOT_DETECT_CTRL   0x69
#define USER_CTRL        0x6A  // Bit 7 enable DMP, bit 3 reset DMP
#define PWR_MGMT_1       0x6B // Device defaults to the SLEEP mode
#define PWR_MGMT_2       0x6C
#define DMP_BANK         0x6D  // Activates a specific bank in the DMP
#define DMP_RW_PNT       0x6E  // Set read/write pointer to a specific start address in specified DMP bank
#define DMP_REG          0x6F  // Register in DMP from which to read or to which to write
#define DMP_REG_1        0x70
#define DMP_REG_2        0x71
#define FIFO_COUNTH      0x72
#define FIFO_COUNTL      0x73
#define FIFO_R_W         0x74
#define WHO_AM_I_MPU6050 0x75
/*******************************************************************************
 * Prototypes
 ******************************************************************************/
enum Ascale {
  AFS_2G = 0,
  AFS_4G,
  AFS_8G,
  AFS_16G
};

enum Gscale {
  GFS_250DPS = 0,
  GFS_500DPS,
  GFS_1000DPS,
  GFS_2000DPS
};
/*******************************************************************************
 * Variables
 ******************************************************************************/
static MPU6050_Raw_DATA_TYPE Accel_X_Raw = 0;
static MPU6050_Raw_DATA_TYPE Accel_Y_Raw = 0;
static MPU6050_Raw_DATA_TYPE Accel_Z_Raw = 0;
static MPU6050_Raw_DATA_TYPE Gyro_Roll_Raw = 0;
static MPU6050_Raw_DATA_TYPE Gyro_Pitch_Raw = 0;
static MPU6050_Raw_DATA_TYPE Gyro_Yaw_Raw = 0;
static uint8_t Buffer_data[6];
static MPU6050_Data_Type MPU6050_RawData;
int Gscale = GFS_500DPS;
int Ascale = AFS_8G;
/*****************************ACCELEROMETER VARIABLE***************************/
double Pitch_Acc = 0.0;
double Roll_Acc = 0.0;
/*********************************GYRO VARIABLE********************************/
static long Gyro_Pitch_Offset = 0.0;
static long Gyro_Roll_Offset = 0.0;
static long Gyro_Yaw_Offset = 0.0;

double pitch = 0.0;
double roll = 0.0;
double yaw = 0.0;

double q[4] = {1.0,0.0,0.0,0.0};
double GyroMeasError;
double beta;
double GyroMeasDrift; 
double zeta;
uint16_t accelCount[3];
uint16_t gyroCount[3];
double deltat = 0.0;
uint32_t lastUpdate = 0, firstUpdate = 0;  
uint32_t Now = 0;
uint32_t count = 0;
double ax, ay, az;
double gyrox, gyroy, gyroz;
double aRes, gRes;
uint32_t delt_t = 0;
/*******************************************************************************
 * Code
 ******************************************************************************/
static double getGres() {
  switch (Gscale)
  {
    // Possible gyro scales (and their register bit settings) are:
    // 250 DPS (00), 500 DPS (01), 1000 DPS (10), and 2000 DPS  (11).
    // Here's a bit of an algorith to calculate DPS/(ADC tick) based on that 2-bit value:
    case GFS_250DPS:
      return 250.0 / 32768.0;
      break;
    case GFS_500DPS:
      return 500.0 / 32768.0;
      break;
    case GFS_1000DPS:
      return 1000.0 / 32768.0;
      break;
    case GFS_2000DPS:
      return 2000.0 / 32768.0;
      break;
  }
}

static double getAres() {
  switch (Ascale)
  {
    // Possible accelerometer scales (and their register bit settings) are:
    // 2 Gs (00), 4 Gs (01), 8 Gs (10), and 16 Gs  (11).
    // Here's a bit of an algorith to calculate DPS/(ADC tick) based on that 2-bit value:
    case AFS_2G:
      return 2.0 / 32768.0;
      break;
    case AFS_4G:
      return 4.0 / 32768.0;
      break;
    case AFS_8G:
      return 8.0 / 32768.0;
      break;
    case AFS_16G:
      return 16.0 / 32768.0;
      break;
  }
}
void MPU6050_Write(uint8_t MPU_Address, uint8_t RegisterAddress, uint8_t Data)
{
	I2C2_WriteToDevice(MPU_Address, RegisterAddress, Data);
}

void MPU6050_Read (uint8_t MPU_Address, uint8_t RegisterAddress, uint8_t *buffer, uint8_t size)
{
	I2C2_ReadDevice(MPU_Address, RegisterAddress, buffer, size);
}

static void writeByte(uint8_t address, uint8_t subAddress, uint8_t data)
{
	I2C2_WriteToDevice(address, subAddress, data);
}

static uint8_t readByte(uint8_t address, uint8_t subAddress)
{
	uint8_t data;
	I2C2_ReadDevice(address,subAddress,&data,1);
	return data;
}

static void readBytes(uint8_t address, uint8_t subAddress, uint8_t count, uint8_t * dest)
{
	I2C2_ReadDevice(address, subAddress, dest, count);
}

void MPU6050SelfTest(double * destination)
{
	GyroMeasError = 3.14159265 * (50.0 / 180.0);    
	GyroMeasDrift = 3.14159265 * (2.0 / 180.0);    
	beta = sqrt(3.0 / 4.0) * GyroMeasError;
	zeta = sqrt(3.0 / 4.0) * GyroMeasDrift;
	uint8_t rawData[4];
	uint8_t selfTest[6];
	double factoryTrim[6];
	writeByte(MPU6050_ADDRESS, ACCEL_CONFIG, 0xF0);
	writeByte(MPU6050_ADDRESS, GYRO_CONFIG,  0xE0);
	delay(MS*250);  // Delay a while to let the device execute the self-test
	rawData[0] = readByte(MPU6050_ADDRESS, SELF_TEST_X);
	rawData[1] = readByte(MPU6050_ADDRESS, SELF_TEST_Y);
	rawData[2] = readByte(MPU6050_ADDRESS, SELF_TEST_Z);
	rawData[3] = readByte(MPU6050_ADDRESS, SELF_TEST_A);
	// Extract the acceleration test results first
	selfTest[0] = (rawData[0] >> 3) | (rawData[3] & 0x30) >> 4 ;
	selfTest[1] = (rawData[1] >> 3) | (rawData[3] & 0x0C) >> 2 ;
	selfTest[2] = (rawData[2] >> 3) | (rawData[3] & 0x03) ;
	// Extract the gyration test results first
	selfTest[3] = rawData[0]  & 0x1F ; // XG_TEST result is a five-bit unsigned integer
	selfTest[4] = rawData[1]  & 0x1F ; // YG_TEST result is a five-bit unsigned integer
	selfTest[5] = rawData[2]  & 0x1F ; // ZG_TEST result is a five-bit unsigned integer
	// Process results to allow final comparison with factory set values
	factoryTrim[0] = (4096.0 * 0.34) * (pow( (0.92 / 0.34) , (((double)selfTest[0] - 1.0) / 30.0)));
	factoryTrim[1] = (4096.0 * 0.34) * (pow( (0.92 / 0.34) , (((double)selfTest[1] - 1.0) / 30.0)));
	factoryTrim[2] = (4096.0 * 0.34) * (pow( (0.92 / 0.34) , (((double)selfTest[2] - 1.0) / 30.0)));
	factoryTrim[3] =  ( 25.0 * 131.0) * (pow( 1.046 , ((double)selfTest[3] - 1.0) ));
	factoryTrim[4] =  (-25.0 * 131.0) * (pow( 1.046 , ((double)selfTest[4] - 1.0) ));
	factoryTrim[5] =  ( 25.0 * 131.0) * (pow( 1.046 , ((double)selfTest[5] - 1.0) ));
	int i = 0;
	for (i = 0; i < 6; i++) {
		destination[i] = 100.0 + 100.0 * ((double)selfTest[i] - factoryTrim[i]) / factoryTrim[i];
	}
}

void calibrateMPU6050(double * dest1, double * dest2) {
	  uint8_t data[12]; // data array to hold accelerometer and gyro x, y, z, data
  uint16_t ii, packet_count, fifo_count;
  uint32_t gyro_bias[3] = {0, 0, 0}, accel_bias[3] = {0, 0, 0};

  // reset device, reset all registers, clear gyro and accelerometer bias registers
  writeByte(MPU6050_ADDRESS, PWR_MGMT_1, 0x80); // Write a one to bit 7 reset bit; toggle reset device
  delay(MS*100);

  // get stable time source
  // Set clock source to be PLL with x-axis gyroscope reference, bits 2:0 = 001
  writeByte(MPU6050_ADDRESS, PWR_MGMT_1, 0x01);
  writeByte(MPU6050_ADDRESS, PWR_MGMT_2, 0x00);
  delay(MS*200);

  // Configure device for bias calculation
  writeByte(MPU6050_ADDRESS, INT_ENABLE, 0x00);   // Disable all interrupts
  writeByte(MPU6050_ADDRESS, FIFO_EN, 0x00);      // Disable FIFO
  writeByte(MPU6050_ADDRESS, PWR_MGMT_1, 0x00);   // Turn on internal clock source
  writeByte(MPU6050_ADDRESS, I2C_MST_CTRL, 0x00); // Disable I2C master
  writeByte(MPU6050_ADDRESS, USER_CTRL, 0x00);    // Disable FIFO and I2C master modes
  writeByte(MPU6050_ADDRESS, USER_CTRL, 0x0C);    // Reset FIFO and DMP
  delay(MS*15);

  // Configure MPU6050 gyro and accelerometer for bias calculation
  writeByte(MPU6050_ADDRESS, CONFIG, 0x01);      // Set low-pass filter to 188 Hz
  writeByte(MPU6050_ADDRESS, SMPLRT_DIV, 0x00);  // Set sample rate to 1 kHz
  writeByte(MPU6050_ADDRESS, GYRO_CONFIG, 0x00);  // Set gyro full-scale to 250 degrees per second, maximum sensitivity
  writeByte(MPU6050_ADDRESS, ACCEL_CONFIG, 0x00); // Set accelerometer full-scale to 2 g, maximum sensitivity

  uint16_t  gyrosensitivity  = 131;   // = 131 LSB/degrees/sec
  uint16_t  accelsensitivity = 16384;  // = 16384 LSB/g

  // Configure FIFO to capture accelerometer and gyro data for bias calculation
  writeByte(MPU6050_ADDRESS, USER_CTRL, 0x40);   // Enable FIFO
  writeByte(MPU6050_ADDRESS, FIFO_EN, 0x78);     // Enable gyro and accelerometer sensors for FIFO  (max size 1024 bytes in MPU-6050)
  delay(MS*80); // accumulate 80 samples in 80 milliseconds = 960 bytes

  // At end of sample accumulation, turn off FIFO sensor read
  writeByte(MPU6050_ADDRESS, FIFO_EN, 0x00);        // Disable gyro and accelerometer sensors for FIFO
  readBytes(MPU6050_ADDRESS, FIFO_COUNTH, 2, &data[0]); // read FIFO sample count
  fifo_count = ((uint16_t)data[0] << 8) | data[1];
  packet_count = fifo_count / 12; // How many sets of full gyro and accelerometer data for averaging

  for (ii = 0; ii < packet_count; ii++) {
    uint16_t accel_temp[3] = {0, 0, 0}, gyro_temp[3] = {0, 0, 0};
    readBytes(MPU6050_ADDRESS, FIFO_R_W, 12, &data[0]); // read data for averaging
    accel_temp[0] = (uint16_t) (((uint16_t)data[0] << 8) | data[1]  ) ;  // Form signed 16-bit integer for each sample in FIFO
    accel_temp[1] = (uint16_t) (((uint16_t)data[2] << 8) | data[3]  ) ;
    accel_temp[2] = (uint16_t) (((uint16_t)data[4] << 8) | data[5]  ) ;
    gyro_temp[0]  = (uint16_t) (((uint16_t)data[6] << 8) | data[7]  ) ;
    gyro_temp[1]  = (uint16_t) (((uint16_t)data[8] << 8) | data[9]  ) ;
    gyro_temp[2]  = (uint16_t) (((uint16_t)data[10] << 8) | data[11]) ;

    accel_bias[0] += (uint32_t) accel_temp[0]; // Sum individual signed 16-bit biases to get accumulated signed 32-bit biases
    accel_bias[1] += (uint32_t) accel_temp[1];
    accel_bias[2] += (uint32_t) accel_temp[2];
    gyro_bias[0]  += (uint32_t) gyro_temp[0];
    gyro_bias[1]  += (uint32_t) gyro_temp[1];
    gyro_bias[2]  += (uint32_t) gyro_temp[2];

  }
  accel_bias[0] /= (uint32_t) packet_count; // Normalize sums to get average count biases
  accel_bias[1] /= (uint32_t) packet_count;
  accel_bias[2] /= (uint32_t) packet_count;
  gyro_bias[0]  /= (uint32_t) packet_count;
  gyro_bias[1]  /= (uint32_t) packet_count;
  gyro_bias[2]  /= (uint32_t) packet_count;

  if (accel_bias[2] > 0L) {
    accel_bias[2] -= (uint32_t) accelsensitivity; // Remove gravity from the z-axis accelerometer bias calculation
  }
  else {
    accel_bias[2] += (uint32_t) accelsensitivity;
  }

  // Construct the gyro biases for push to the hardware gyro bias registers, which are reset to zero upon device startup
  data[0] = (-gyro_bias[0] / 4  >> 8) & 0xFF; // Divide by 4 to get 32.9 LSB per deg/s to conform to expected bias input format
  data[1] = (-gyro_bias[0] / 4)       & 0xFF; // Biases are additive, so change sign on calculated average gyro biases
  data[2] = (-gyro_bias[1] / 4  >> 8) & 0xFF;
  data[3] = (-gyro_bias[1] / 4)       & 0xFF;
  data[4] = (-gyro_bias[2] / 4  >> 8) & 0xFF;
  data[5] = (-gyro_bias[2] / 4)       & 0xFF;

  // Push gyro biases to hardware registers
  writeByte(MPU6050_ADDRESS, XG_OFFS_USRH, data[0]);// might not be supported in MPU6050
  writeByte(MPU6050_ADDRESS, XG_OFFS_USRL, data[1]);
  writeByte(MPU6050_ADDRESS, YG_OFFS_USRH, data[2]);
  writeByte(MPU6050_ADDRESS, YG_OFFS_USRL, data[3]);
  writeByte(MPU6050_ADDRESS, ZG_OFFS_USRH, data[4]);
  writeByte(MPU6050_ADDRESS, ZG_OFFS_USRL, data[5]);

  dest1[0] = (double) gyro_bias[0] / (double) gyrosensitivity; // construct gyro bias in deg/s for later manual subtraction
  dest1[1] = (double) gyro_bias[1] / (double) gyrosensitivity;
  dest1[2] = (double) gyro_bias[2] / (double) gyrosensitivity;

  // Construct the accelerometer biases for push to the hardware accelerometer bias registers. These registers contain
  // factory trim values which must be added to the calculated accelerometer biases; on boot up these registers will hold
  // non-zero values. In addition, bit 0 of the lower byte must be preserved since it is used for temperature
  // compensation calculations. Accelerometer bias registers expect bias input as 2048 LSB per g, so that
  // the accelerometer biases calculated above must be divided by 8.

  uint32_t accel_bias_reg[3] = {0, 0, 0}; // A place to hold the factory accelerometer trim biases
  readBytes(MPU6050_ADDRESS, XA_OFFSET_H, 2, &data[0]); // Read factory accelerometer trim values
  accel_bias_reg[0] = (uint16_t) ((uint16_t)data[0] << 8) | data[1];
  readBytes(MPU6050_ADDRESS, YA_OFFSET_H, 2, &data[0]);
  accel_bias_reg[1] = (uint16_t) ((uint16_t)data[0] << 8) | data[1];
  readBytes(MPU6050_ADDRESS, ZA_OFFSET_H, 2, &data[0]);
  accel_bias_reg[2] = (uint16_t) ((uint16_t)data[0] << 8) | data[1];

  uint32_t mask = 1uL; // Define mask for temperature compensation bit 0 of lower byte of accelerometer bias registers
  uint8_t mask_bit[3] = {0, 0, 0}; // Define array to hold mask bit for each accelerometer bias axis

  for (ii = 0; ii < 3; ii++) {
    if (accel_bias_reg[ii] & mask) mask_bit[ii] = 0x01; // If temperature compensation bit is set, record that fact in mask_bit
  }

  // Construct total accelerometer bias, including calculated average accelerometer bias from above
  accel_bias_reg[0] -= (accel_bias[0] / 8); // Subtract calculated averaged accelerometer bias scaled to 2048 LSB/g (16 g full scale)
  accel_bias_reg[1] -= (accel_bias[1] / 8);
  accel_bias_reg[2] -= (accel_bias[2] / 8);

  data[0] = (accel_bias_reg[0] >> 8) & 0xFF;
  data[1] = (accel_bias_reg[0])      & 0xFF;
  data[1] = data[1] | mask_bit[0]; // preserve temperature compensation bit when writing back to accelerometer bias registers
  data[2] = (accel_bias_reg[1] >> 8) & 0xFF;
  data[3] = (accel_bias_reg[1])      & 0xFF;
  data[3] = data[3] | mask_bit[1]; // preserve temperature compensation bit when writing back to accelerometer bias registers
  data[4] = (accel_bias_reg[2] >> 8) & 0xFF;
  data[5] = (accel_bias_reg[2])      & 0xFF;
  data[5] = data[5] | mask_bit[2]; // preserve temperature compensation bit when writing back to accelerometer bias registers

  // Push accelerometer biases to hardware registers
  writeByte(MPU6050_ADDRESS, XA_OFFSET_H, data[0]); // might not be supported in MPU6050
  writeByte(MPU6050_ADDRESS, XA_OFFSET_L_TC, data[1]);
  writeByte(MPU6050_ADDRESS, YA_OFFSET_H, data[2]);
  writeByte(MPU6050_ADDRESS, YA_OFFSET_L_TC, data[3]);
  writeByte(MPU6050_ADDRESS, ZA_OFFSET_H, data[4]);
  writeByte(MPU6050_ADDRESS, ZA_OFFSET_L_TC, data[5]);

  // Output scaled accelerometer biases for manual subtraction in the main program
  dest2[0] = (double)accel_bias[0] / (double)accelsensitivity;
  dest2[1] = (double)accel_bias[1] / (double)accelsensitivity;
  dest2[2] = (double)accel_bias[2] / (double)accelsensitivity;
}

void initMPU6050 (void)
{
	writeByte(MPU6050_ADDRESS, PWR_MGMT_1, 0x01);  // Set clock source to be PLL with x-axis gyroscope reference, bits 2:0 = 001

	// Configure Gyro and Accelerometer
	// Disable FSYNC and set accelerometer and gyro bandwidth to 44 and 42 Hz, respectively;
	// DLPF_CFG = bits 2:0 = 010; this sets the sample rate at 1 kHz for both
	// Maximum delay time is 4.9 ms corresponding to just over 200 Hz sample rate
	writeByte(MPU6050_ADDRESS, CONFIG, 0x03);

	// Set sample rate = gyroscope output rate/(1 + SMPLRT_DIV)
	writeByte(MPU6050_ADDRESS, SMPLRT_DIV, 0x04);  // Use a 200 Hz rate; the same rate set in CONFIG above

	// Set gyroscope full scale range
	// Range selects FS_SEL and AFS_SEL are 0 - 3, so 2-bit values are left-shifted into positions 4:3
	uint8_t c =  readByte(MPU6050_ADDRESS, GYRO_CONFIG);
	writeByte(MPU6050_ADDRESS, GYRO_CONFIG, c & ~0xE0); // Clear self-test bits [7:5]
	writeByte(MPU6050_ADDRESS, GYRO_CONFIG, c & ~0x18); // Clear AFS bits [4:3]
	writeByte(MPU6050_ADDRESS, GYRO_CONFIG, c | Gscale << 3); // Set full scale range for the gyro

	// Set accelerometer configuration
	c =  readByte(MPU6050_ADDRESS, ACCEL_CONFIG);
	writeByte(MPU6050_ADDRESS, ACCEL_CONFIG, c & ~0xE0); // Clear self-test bits [7:5]
	writeByte(MPU6050_ADDRESS, ACCEL_CONFIG, c & ~0x18); // Clear AFS bits [4:3]
	writeByte(MPU6050_ADDRESS, ACCEL_CONFIG, c | Ascale << 3); // Set full scale range for the accelerometer
}

void MPU6050_getPara (void)
{
	MPU6050_Read (MPU6050_ADDR, MPU6050_ACCEL_OUT, &Buffer_data[0], 6);
	
	Accel_X_Raw = (MPU6050_Raw_DATA_TYPE)(Buffer_data[0] << 8 | Buffer_data [1]);
	Accel_Y_Raw = (MPU6050_Raw_DATA_TYPE)(Buffer_data[2] << 8 | Buffer_data [3]);
	Accel_Z_Raw = (MPU6050_Raw_DATA_TYPE)(Buffer_data[4] << 8 | Buffer_data [5]);
	
	MPU6050_Read (MPU6050_ADDR, GYRO_XOUT_H_REG, &Buffer_data[0], 6);
	Gyro_Roll_Raw = (MPU6050_Raw_DATA_TYPE)(Buffer_data[0] << 8 | Buffer_data [1]);
	Gyro_Pitch_Raw = (MPU6050_Raw_DATA_TYPE)(Buffer_data[2] << 8 | Buffer_data [3]);
	Gyro_Yaw_Raw = (MPU6050_Raw_DATA_TYPE)(Buffer_data[4] << 8 | Buffer_data [5]);
}

void QuaternionUpdate(double ax, double ay, double az, double gyrox, double gyroy, double gyroz)
        {
            double q1 = q[0], q2 = q[1], q3 = q[2], q4 = q[3];         // short name local variable for readability
            double norm;                                               // vector norm
            double f1, f2, f3;                                         // objetive funcyion elements
            double J_11or24, J_12or23, J_13or22, J_14or21, J_32, J_33; // objective function Jacobian elements
            double qDot1, qDot2, qDot3, qDot4;
            double hatDot1, hatDot2, hatDot3, hatDot4;
            double gerrx, gerry, gerrz, gbiasx, gbiasy, gbiasz;        // gyro bias error

            // Auxiliary variables to avoid repeated arithmetic
            double _halfq1 = 0.5f * q1;
            double _halfq2 = 0.5f * q2;
            double _halfq3 = 0.5f * q3;
            double _halfq4 = 0.5f * q4;
            double _2q1 = 2.0 * q1;
            double _2q2 = 2.0 * q2;
            double _2q3 = 2.0 * q3;
            double _2q4 = 2.0 * q4;
            double _2q1q3 = 2.0 * q1 * q3;
            double _2q3q4 = 2.0 * q3 * q4;

            // Normalise accelerometer measurement
            norm = sqrt(ax * ax + ay * ay + az * az);
            if (norm == 0.0) return; // handle NaN
            norm = 1.0/norm;
            ax *= norm;
            ay *= norm;
            az *= norm;
            
            // Compute the objective function and Jacobian
            f1 = _2q2 * q4 - _2q1 * q3 - ax;
            f2 = _2q1 * q2 + _2q3 * q4 - ay;
            f3 = 1.0 - _2q2 * q2 - _2q3 * q3 - az;
            J_11or24 = _2q3;
            J_12or23 = _2q4;
            J_13or22 = _2q1;
            J_14or21 = _2q2;
            J_32 = 2.0 * J_14or21;
            J_33 = 2.0 * J_11or24;
          
            // Compute the gradient (matrix multiplication)
            hatDot1 = J_14or21 * f2 - J_11or24 * f1;
            hatDot2 = J_12or23 * f1 + J_13or22 * f2 - J_32 * f3;
            hatDot3 = J_12or23 * f2 - J_33 *f3 - J_13or22 * f1;
            hatDot4 = J_14or21 * f1 + J_11or24 * f2;
            
            // Normalize the gradient
            norm = sqrt(hatDot1 * hatDot1 + hatDot2 * hatDot2 + hatDot3 * hatDot3 + hatDot4 * hatDot4);
            hatDot1 /= norm;
            hatDot2 /= norm;
            hatDot3 /= norm;
            hatDot4 /= norm;
            
            // Compute estimated gyroscope biases
            gerrx = _2q1 * hatDot2 - _2q2 * hatDot1 - _2q3 * hatDot4 + _2q4 * hatDot3;
            gerry = _2q1 * hatDot3 + _2q2 * hatDot4 - _2q3 * hatDot1 - _2q4 * hatDot2;
            gerrz = _2q1 * hatDot4 - _2q2 * hatDot3 + _2q3 * hatDot2 - _2q4 * hatDot1;
            
            // Compute and remove gyroscope biases
            gbiasx += gerrx * deltat * zeta;
            gbiasy += gerry * deltat * zeta;
            gbiasz += gerrz * deltat * zeta;
            gyrox -= gbiasx;
            gyroy -= gbiasy;
            gyroz -= gbiasz;
            
            // Compute the quaternion derivative
            qDot1 = -_halfq2 * gyrox - _halfq3 * gyroy - _halfq4 * gyroz;
            qDot2 =  _halfq1 * gyrox + _halfq3 * gyroz - _halfq4 * gyroy;
            qDot3 =  _halfq1 * gyroy - _halfq2 * gyroz + _halfq4 * gyrox;
            qDot4 =  _halfq1 * gyroz + _halfq2 * gyroy - _halfq3 * gyrox;

            // Compute then integrate estimated quaternion derivative
            q1 += (qDot1 -(beta * hatDot1)) * deltat;
            q2 += (qDot2 -(beta * hatDot2)) * deltat;
            q3 += (qDot3 -(beta * hatDot3)) * deltat;
            q4 += (qDot4 -(beta * hatDot4)) * deltat;

            // Normalize the quaternion
            norm = sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);    // normalise quaternion
            norm = 1.0/norm;
            q[0] = q1 * norm;
            q[1] = q2 * norm;
            q[2] = q3 * norm;
            q[3] = q4 * norm;
        }
	
void MPU6050_CalculateAngle (void) {
	uint8_t rawData[6];
    readBytes(MPU6050_ADDRESS, ACCEL_XOUT_H, 6, &rawData[0]);  // Read the six raw data registers into data array
    accelCount[0] = (uint16_t)((rawData[0] << 8) | rawData[1]) ;  // Turn the MSB and LSB into a signed 16-bit value
    accelCount[1] = (uint16_t)((rawData[2] << 8) | rawData[3]) ;
    accelCount[2] = (uint16_t)((rawData[4] << 8) | rawData[5]) ;  // Read the x/y/z adc values
    aRes = 8.0 / 32768.0;
    
    // Now we'll calculate the accleration value into actual g's
    ax = (double)accelCount[0] * aRes; // get actual g value, this depends on scale being set
    ay = (double)accelCount[1] * aRes;
    az = (double)accelCount[2] * aRes;

  	readBytes(MPU6050_ADDRESS, GYRO_XOUT_H, 6, &rawData[0]);  // Read the six raw data registers sequentially into data array
  	gyroCount[0] = (uint16_t)((rawData[0] << 8) | rawData[1]) ;  // Turn the MSB and LSB into a signed 16-bit value
  	gyroCount[1] = (uint16_t)((rawData[2] << 8) | rawData[3]) ;
  	gyroCount[2] = (uint16_t)((rawData[4] << 8) | rawData[5]) ;
    gRes = 500.0 / 32768.0;
    // Calculate the gyro value into actual degrees per second
    gyrox = (double)gyroCount[0] * gRes; // get actual gyro value, this depends on scale being set
    gyroy = (double)gyroCount[1] * gRes;
    gyroz = (double)gyroCount[2] * gRes;

  Now = micros();
  deltat = ((Now - lastUpdate) / 1000000.0); // set integration time by time elapsed since last filter update
  lastUpdate = Now;
  QuaternionUpdate(ax, ay, az, gyrox * pi / 180.0, gyroy * pi / 180.0, gyroz * pi / 180.0);
  delt_t = micros() - count;
  if (delt_t > 5000) {
	yaw   = atan2(2.0 * (q[1] * q[2] + q[0] * q[3]), q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3]);
    pitch = -asin(2.0 * (q[1] * q[3] - q[0] * q[2]));
    roll  = atan2(2.0 * (q[0] * q[1] + q[2] * q[3]), q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3]);

    pitch *= 180.0 / pi;
    yaw   *= 180.0 / pi;
    roll  *= 180.0 / pi;
    count = micros();
  }
}

void MPU6050_Calibration (void) {
	uint16_t i_ui16;
	unsigned uint32_t loop_timer = 0U;
	Gyro_Pitch_Offset = 0.0;
	Gyro_Roll_Offset = 0.0;
	Gyro_Yaw_Offset = 0.0;
	for (i_ui16 = 0; i_ui16 < CALIBRATION_TIMES; i_ui16++) {
		loop_timer = micros();
		MPU6050_getPara();
		Gyro_Pitch_Offset += Gyro_Roll_Raw;
		Gyro_Roll_Offset += Gyro_Pitch_Raw;
		Gyro_Yaw_Offset += Gyro_Yaw_Raw;
		while (micros() - loop_timer < 4000) {}
	}
	Gyro_Pitch_Offset 	= Gyro_Pitch_Offset / CALIBRATION_TIMES;
	Gyro_Roll_Offset 	= Gyro_Roll_Offset  / CALIBRATION_TIMES;
	Gyro_Yaw_Offset 	= Gyro_Yaw_Offset  / CALIBRATION_TIMES;	
}
/*******************************************************************************
 * EOF
 ******************************************************************************/
