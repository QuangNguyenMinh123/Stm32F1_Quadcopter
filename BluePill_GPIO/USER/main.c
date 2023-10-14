#include "GPIO.h"
#include "UART.h"
#include "CLOCK.h"
#include "I2C.h"
#include "math.h"
#include "EX_EEPROM.h"
#include "PID.h"
/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define AAA		120
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
#define USER_CTRL        0x6A
#define PWR_MGMT_1       0x6B
#define PWR_MGMT_2       0x6C
#define DMP_BANK         0x6D
#define DMP_RW_PNT       0x6E
#define DMP_REG          0x6F
#define DMP_REG_1        0x70
#define DMP_REG_2        0x71
#define FIFO_COUNTH      0x72
#define FIFO_COUNTL      0x73
#define FIFO_R_W         0x74
#define WHO_AM_I_MPU6050 0x75
#define MPU6050_ADDRESS  0x68
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
void System_Init(void);
void check(int a);
/*******************************************************************************
 * Variables
 ******************************************************************************/
int Gscale = GFS_250DPS;
int Ascale = AFS_2G;
double aRes, gRes;
  
uint16_t accelCount[3]; 
double ax, ay, az;    
uint16_t gyroCount[3];  
double gx, gy, gz;  
double gyroBias[3] = {0, 0, 0}, 
accelBias[3] = {0, 0, 0};
uint16_t tempCount;
double temperature;
double SelfTest[6];

uint32_t delt_t = 0;
uint32_t count = 0;


double GyroMeasError;
double beta;
double GyroMeasDrift;
double zeta;
double pitch, yaw, roll;
double deltat = 0.0;                              // integration interval for both filter schemes
uint32_t lastUpdate = 0, firstUpdate = 0;         // used to calculate integration interval
uint32_t Now = 0;                                 // used to calculate integration interval
double q[4] = {1.0, 0.0, 0.0, 0.0};            // vector to hold quaternion

/*******************************************************************************
 * Code
 ******************************************************************************/
void getGres();
void getAres();
void readAccelData(uint16_t * destination);
void readGyroData(uint16_t * destination);
uint16_t readTempData();
void LowPowerAccelOnlyMPU6050();
void initMPU6050(void);
void MadgwickQuaternionUpdate(double ax, double ay, double az, double gx, double gy, double gz);
void readBytes(uint8_t address, uint8_t subAddress, uint8_t count, uint8_t * dest);
uint8_t readByte(uint8_t address, uint8_t subAddress);
void MPU6050SelfTest(double * destination);
void calibrateMPU6050(double * dest1, double * dest2);
void writeByte(uint8_t address, uint8_t subAddress, uint8_t data)
{
  I2C2_Start();
	I2C2_CallAdress( (uint8_t) ((address<<1) | I2C_WRITE) );
	I2C2_Write(subAddress);
	I2C2_Write(data);
	I2C2_Stop();
}

uint32_t millis(void) {
	return micros() / 1000;
}
int main(void) {
	GyroMeasError = pi * (40.0 / 180.0);
	beta = sqrt(3.0 / 4.0) * GyroMeasError;
	GyroMeasDrift = pi * (2.0 / 180.0);
	zeta = sqrt(3.0 / 4.0) * GyroMeasDrift;
	CLOCK_SystickInit();
	I2C2_Init(I2C_SPEED_100);
	uint8_t c = readByte(MPU6050_ADDRESS, WHO_AM_I_MPU6050);  
  if (c == 0x68) // WHO_AM_I should always be 0x68
  {  
    MPU6050SelfTest(SelfTest); 
    if(SelfTest[0] < 1.0 && SelfTest[1] < 1.0 && SelfTest[2] < 1.0 && SelfTest[3] < 1.0 && SelfTest[4] < 1.0 && SelfTest[5] < 1.0) {

    delay(MS*1000);
  
    calibrateMPU6050(gyroBias, accelBias); // Calibrate gyro and accelerometers, load biases in bias registers  
    delay(MS*1000); 
    
    initMPU6050();
   }
  }
  
  
	while (1) {
		uint8_t buff[6];
		I2C2_Start ();
		I2C2_CallAdress ( (uint8_t) ((MPU6050_ADDRESS<<1) | I2C_WRITE) );
		I2C2_Write (ACCEL_XOUT_H);
		I2C2_Start ();
		I2C2_Read ((uint8_t) ((MPU6050_ADDRESS<<1) | I2C_READ), buff, 6);
		I2C2_Stop ();
		accelCount[0] = (uint16_t)((buff[0] << 8) | buff[1]) ;  // Turn the MSB and LSB into a signed 16-bit value
		accelCount[1] = (uint16_t)((buff[2] << 8) | buff[3]) ;  
		accelCount[2] = (uint16_t)((buff[4] << 8) | buff[5]) ; 
		getAres();
		
		// Now we'll calculate the accleration value into actual g's
		ax = (double)accelCount[0]*aRes;  // get actual g value, this depends on scale being set
		ay = (double)accelCount[1]*aRes;   
		az = (double)accelCount[2]*aRes;  
	   
		I2C2_Start ();
		I2C2_CallAdress ( (uint8_t) ((MPU6050_ADDRESS<<1) | I2C_WRITE) );
		I2C2_Write (GYRO_XOUT_H);
		I2C2_Start ();
		I2C2_Read ((uint8_t) ((MPU6050_ADDRESS<<1) | I2C_READ), buff, 6);
		I2C2_Stop ();
		gyroCount[0] = (uint16_t)((buff[0] << 8) | buff[1]) ;  // Turn the MSB and LSB into a signed 16-bit value
		gyroCount[1] = (uint16_t)((buff[2] << 8) | buff[3]) ;  
		gyroCount[2] = (uint16_t)((buff[4] << 8) | buff[5]) ; 
		getGres();
	 
		// Calculate the gyro value into actual degrees per second
		gx = (double)gyroCount[0]*gRes;  // get actual gyro value, this depends on scale being set
		gy = (double)gyroCount[1]*gRes;  
		gz = (double)gyroCount[2]*gRes;   
		
		Now = micros();
		deltat = ((Now - lastUpdate)/1000000.0); // set integration time by time elapsed since last filter update
		lastUpdate = Now;

		MadgwickQuaternionUpdate(ax, ay, az, gx*pi/180.0, gy*pi/180.0, gz*pi/180.0);

		// Serial print and/or display at 0.5 s rate independent of data rates
		delt_t = millis() - count;
		
		yaw   = atan2(2.0 * (q[1] * q[2] + q[0] * q[3]), q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3]);   
		pitch = -asin(2.0 * (q[1] * q[3] - q[0] * q[2]));
		roll  = atan2(2.0 * (q[0] * q[1] + q[2] * q[3]), q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3]);
		pitch *= 180.0 / pi;
		yaw   *= 180.0 / pi; 
		roll  *= 180.0 / pi;
	}
}

void getGres() {
  switch (Gscale)
  {
    case GFS_250DPS:
          gRes = 250.0/32768.0;
          break;
    case GFS_500DPS:
          gRes = 500.0/32768.0;
          break;
    case GFS_1000DPS:
          gRes = 1000.0/32768.0;
          break;
    case GFS_2000DPS:
          gRes = 2000.0/32768.0;
          break;
  }
}

void getAres() {
  switch (Ascale)
  {
    case AFS_2G:
          aRes = 2.0/32768.0;
          break;
    case AFS_4G:
          aRes = 4.0/32768.0;
          break;
    case AFS_8G:
          aRes = 8.0/32768.0;
          break;
    case AFS_16G:
          aRes = 16.0/32768.0;
          break;
  }
}


void readAccelData(uint16_t * destination)
{
  uint8_t rawData[6];  // x/y/z accel register data stored here
  readBytes(MPU6050_ADDRESS, ACCEL_XOUT_H, 6, &rawData[0]);  // Read the six raw data registers into data array
  destination[0] = (uint16_t)((rawData[0] << 8) | rawData[1]) ;  // Turn the MSB and LSB into a signed 16-bit value
  destination[1] = (uint16_t)((rawData[2] << 8) | rawData[3]) ;  
  destination[2] = (uint16_t)((rawData[4] << 8) | rawData[5]) ; 
}

void readGyroData(uint16_t * destination)
{
  uint8_t rawData[6];  // x/y/z gyro register data stored here
  readBytes(MPU6050_ADDRESS, GYRO_XOUT_H, 6, &rawData[0]);  // Read the six raw data registers sequentially into data array
  destination[0] = (uint16_t)((rawData[0] << 8) | rawData[1]) ;  // Turn the MSB and LSB into a signed 16-bit value
  destination[1] = (uint16_t)((rawData[2] << 8) | rawData[3]) ;  
  destination[2] = (uint16_t)((rawData[4] << 8) | rawData[5]) ; 
}

uint16_t readTempData()
{
  uint8_t rawData[2];  // x/y/z gyro register data stored here
  readBytes(MPU6050_ADDRESS, TEMP_OUT_H, 2, &rawData[0]);  // Read the two raw data registers sequentially into data array 
  return ((uint16_t)rawData[0]) << 8 | rawData[1] ;  // Turn the MSB and LSB into a 16-bit value
}



// Configure the motion detection control for low power accelerometer mode
void LowPowerAccelOnlyMPU6050()
{
  uint8_t c = readByte(MPU6050_ADDRESS, PWR_MGMT_1);
  writeByte(MPU6050_ADDRESS, PWR_MGMT_1, c & ~0x30); // Clear sleep and cycle bits [5:6]
  writeByte(MPU6050_ADDRESS, PWR_MGMT_1, c |  0x30); // Set sleep and cycle bits [5:6] to zero to make sure accelerometer is running

  c = readByte(MPU6050_ADDRESS, PWR_MGMT_2);
  writeByte(MPU6050_ADDRESS, PWR_MGMT_2, c & ~0x38); // Clear standby XA, YA, and ZA bits [3:5]
  writeByte(MPU6050_ADDRESS, PWR_MGMT_2, c |  0x00); // Set XA, YA, and ZA bits [3:5] to zero to make sure accelerometer is running
    
  c = readByte(MPU6050_ADDRESS, ACCEL_CONFIG);
  writeByte(MPU6050_ADDRESS, ACCEL_CONFIG, c & ~0x07); // Clear high-pass filter bits [2:0]
// Set high-pass filter to 0) reset (disable), 1) 5 Hz, 2) 2.5 Hz, 3) 1.25 Hz, 4) 0.63 Hz, or 7) Hold
  writeByte(MPU6050_ADDRESS, ACCEL_CONFIG,  c | 0x00);  // Set ACCEL_HPF to 0; reset mode disbaling high-pass filter

  c = readByte(MPU6050_ADDRESS, CONFIG);
  writeByte(MPU6050_ADDRESS, CONFIG, c & ~0x07); // Clear low-pass filter bits [2:0]
  writeByte(MPU6050_ADDRESS, CONFIG, c |  0x00);  // Set DLPD_CFG to 0; 260 Hz bandwidth, 1 kHz rate
    
  c = readByte(MPU6050_ADDRESS, INT_ENABLE);
  writeByte(MPU6050_ADDRESS, INT_ENABLE, c & ~0xFF);  // Clear all interrupts
  writeByte(MPU6050_ADDRESS, INT_ENABLE, 0x40);  // Enable motion threshold (bits 5) interrupt only
  
// Motion detection interrupt requires the absolute value of any axis to lie above the detection threshold
// for at least the counter duration
  writeByte(MPU6050_ADDRESS, MOT_THR, 0x80); // Set motion detection to 0.256 g; LSB = 2 mg
  writeByte(MPU6050_ADDRESS, MOT_DUR, 0x01); // Set motion detect duration to 1  ms; LSB is 1 ms @ 1 kHz rate
  
  delay (100);  // Add delay for accumulation of samples
  
  c = readByte(MPU6050_ADDRESS, ACCEL_CONFIG);
  writeByte(MPU6050_ADDRESS, ACCEL_CONFIG, c & ~0x07); // Clear high-pass filter bits [2:0]
  writeByte(MPU6050_ADDRESS, ACCEL_CONFIG, c |  0x07);  // Set ACCEL_HPF to 7; hold the initial accleration value as a referance
   
  c = readByte(MPU6050_ADDRESS, PWR_MGMT_2);
  writeByte(MPU6050_ADDRESS, PWR_MGMT_2, c & ~0xC7); // Clear standby XA, YA, and ZA bits [3:5] and LP_WAKE_CTRL bits [6:7]
  writeByte(MPU6050_ADDRESS, PWR_MGMT_2, c |  0x47); // Set wakeup frequency to 5 Hz, and disable XG, YG, and ZG gyros (bits [0:2])  

  c = readByte(MPU6050_ADDRESS, PWR_MGMT_1);
  writeByte(MPU6050_ADDRESS, PWR_MGMT_1, c & ~0x20); // Clear sleep and cycle bit 5
  writeByte(MPU6050_ADDRESS, PWR_MGMT_1, c |  0x20); // Set cycle bit 5 to begin low power accelerometer motion interrupts

}


void initMPU6050(void)
{  
  writeByte(MPU6050_ADDRESS, PWR_MGMT_1, 0x01);
  writeByte(MPU6050_ADDRESS, CONFIG, 0x03);
  writeByte(MPU6050_ADDRESS, SMPLRT_DIV, 0x04);
  uint8_t c =  readByte(MPU6050_ADDRESS, GYRO_CONFIG);
  writeByte(MPU6050_ADDRESS, GYRO_CONFIG, c & ~0xE0);
  writeByte(MPU6050_ADDRESS, GYRO_CONFIG, c & ~0x18);
  writeByte(MPU6050_ADDRESS, GYRO_CONFIG, c | Gscale << 3);
	
  c =  readByte(MPU6050_ADDRESS, ACCEL_CONFIG);
  writeByte(MPU6050_ADDRESS, ACCEL_CONFIG, c & ~0xE0); // Clear self-test bits [7:5] 
  writeByte(MPU6050_ADDRESS, ACCEL_CONFIG, c & ~0x18); // Clear AFS bits [4:3]
  writeByte(MPU6050_ADDRESS, ACCEL_CONFIG, c | Ascale << 3);
	
   writeByte(MPU6050_ADDRESS, INT_PIN_CFG, 0x22);    
   writeByte(MPU6050_ADDRESS, INT_ENABLE, 0x01);
}

void calibrateMPU6050(double * dest1, double * dest2)
{  
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
  packet_count = fifo_count/12;// How many sets of full gyro and accelerometer data for averaging

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
    
  if(accel_bias[2] > 0L) {accel_bias[2] -= (uint32_t) accelsensitivity;}  // Remove gravity from the z-axis accelerometer bias calculation
  else {accel_bias[2] += (uint32_t) accelsensitivity;}
 
// Construct the gyro biases for push to the hardware gyro bias registers, which are reset to zero upon device startup
  data[0] = (-gyro_bias[0]/4  >> 8) & 0xFF; // Divide by 4 to get 32.9 LSB per deg/s to conform to expected bias input format
  data[1] = (-gyro_bias[0]/4)       & 0xFF; // Biases are additive, so change sign on calculated average gyro biases
  data[2] = (-gyro_bias[1]/4  >> 8) & 0xFF;
  data[3] = (-gyro_bias[1]/4)       & 0xFF;
  data[4] = (-gyro_bias[2]/4  >> 8) & 0xFF;
  data[5] = (-gyro_bias[2]/4)       & 0xFF;

// Push gyro biases to hardware registers
  writeByte(MPU6050_ADDRESS, XG_OFFS_USRH, data[0]);// might not be supported in MPU6050
  writeByte(MPU6050_ADDRESS, XG_OFFS_USRL, data[1]);
  writeByte(MPU6050_ADDRESS, YG_OFFS_USRH, data[2]);
  writeByte(MPU6050_ADDRESS, YG_OFFS_USRL, data[3]);
  writeByte(MPU6050_ADDRESS, ZG_OFFS_USRH, data[4]);
  writeByte(MPU6050_ADDRESS, ZG_OFFS_USRL, data[5]);

  dest1[0] = (double) gyro_bias[0]/(double) gyrosensitivity; // construct gyro bias in deg/s for later manual subtraction
  dest1[1] = (double) gyro_bias[1]/(double) gyrosensitivity;
  dest1[2] = (double) gyro_bias[2]/(double) gyrosensitivity;
  
  uint32_t accel_bias_reg[3] = {0, 0, 0}; // A place to hold the factory accelerometer trim biases
  readBytes(MPU6050_ADDRESS, XA_OFFSET_H, 2, &data[0]); // Read factory accelerometer trim values
  accel_bias_reg[0] = (uint16_t) ((uint16_t)data[0] << 8) | data[1];
  readBytes(MPU6050_ADDRESS, YA_OFFSET_H, 2, &data[0]);
  accel_bias_reg[1] = (uint16_t) ((uint16_t)data[0] << 8) | data[1];
  readBytes(MPU6050_ADDRESS, ZA_OFFSET_H, 2, &data[0]);
  accel_bias_reg[2] = (uint16_t) ((uint16_t)data[0] << 8) | data[1];
  
  uint32_t mask = 1uL; // Define mask for temperature compensation bit 0 of lower byte of accelerometer bias registers
  uint8_t mask_bit[3] = {0, 0, 0}; // Define array to hold mask bit for each accelerometer bias axis
  
  for(ii = 0; ii < 3; ii++) {
    if(accel_bias_reg[ii] & mask) mask_bit[ii] = 0x01; // If temperature compensation bit is set, record that fact in mask_bit
  }

  // Construct total accelerometer bias, including calculated average accelerometer bias from above
  accel_bias_reg[0] -= (accel_bias[0]/8); // Subtract calculated averaged accelerometer bias scaled to 2048 LSB/g (16 g full scale)
  accel_bias_reg[1] -= (accel_bias[1]/8);
  accel_bias_reg[2] -= (accel_bias[2]/8);
 
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
   dest2[0] = (double)accel_bias[0]/(double)accelsensitivity; 
   dest2[1] = (double)accel_bias[1]/(double)accelsensitivity;
   dest2[2] = (double)accel_bias[2]/(double)accelsensitivity;
}


// Accelerometer and gyroscope self test; check calibration wrt factory settings
void MPU6050SelfTest(double * destination) // Should return percent deviation from factory trim values, +/- 14 or less deviation is a pass
{
   uint8_t rawData[4];
   uint8_t selfTest[6];
   double factoryTrim[6];
   
   // Configure the accelerometer for self-test
   writeByte(MPU6050_ADDRESS, ACCEL_CONFIG, 0xF0); // Enable self test on all three axes and set accelerometer range to +/- 8 g
   writeByte(MPU6050_ADDRESS, GYRO_CONFIG,  0xE0); // Enable self test on all three axes and set gyro range to +/- 250 degrees/s
   delay(MS*250);  // Delay a while to let the device execute the self-test
   rawData[0] = readByte(MPU6050_ADDRESS, SELF_TEST_X); // X-axis self-test results
   rawData[1] = readByte(MPU6050_ADDRESS, SELF_TEST_Y); // Y-axis self-test results
   rawData[2] = readByte(MPU6050_ADDRESS, SELF_TEST_Z); // Z-axis self-test results
   rawData[3] = readByte(MPU6050_ADDRESS, SELF_TEST_A); // Mixed-axis self-test results
   // Extract the acceleration test results first
   selfTest[0] = (rawData[0] >> 3) | (rawData[3] & 0x30) >> 4 ; // XA_TEST result is a five-bit unsigned integer
   selfTest[1] = (rawData[1] >> 3) | (rawData[3] & 0x0C) >> 4 ; // YA_TEST result is a five-bit unsigned integer
   selfTest[2] = (rawData[2] >> 3) | (rawData[3] & 0x03) >> 4 ; // ZA_TEST result is a five-bit unsigned integer
   // Extract the gyration test results first
   selfTest[3] = rawData[0]  & 0x1F ; // XG_TEST result is a five-bit unsigned integer
   selfTest[4] = rawData[1]  & 0x1F ; // YG_TEST result is a five-bit unsigned integer
   selfTest[5] = rawData[2]  & 0x1F ; // ZG_TEST result is a five-bit unsigned integer   
   // Process results to allow final comparison with factory set values
   factoryTrim[0] = (4096.0*0.34)*(pow( (0.92/0.34) , (((double)selfTest[0] - 1.0)/30.0))); // FT[Xa] factory trim calculation
   factoryTrim[1] = (4096.0*0.34)*(pow( (0.92/0.34) , (((double)selfTest[1] - 1.0)/30.0))); // FT[Ya] factory trim calculation
   factoryTrim[2] = (4096.0*0.34)*(pow( (0.92/0.34) , (((double)selfTest[2] - 1.0)/30.0))); // FT[Za] factory trim calculation
   factoryTrim[3] =  ( 25.0*131.0)*(pow( 1.046 , ((double)selfTest[3] - 1.0) ));             // FT[Xg] factory trim calculation
   factoryTrim[4] =  (-25.0*131.0)*(pow( 1.046 , ((double)selfTest[4] - 1.0) ));             // FT[Yg] factory trim calculation
   factoryTrim[5] =  ( 25.0*131.0)*(pow( 1.046 , ((double)selfTest[5] - 1.0) ));             // FT[Zg] factory trim calculation
   uint32_t i = 0U;
   for (; i < 6; i++) {
     destination[i] = 100.0 + 100.0*((double)selfTest[i] - factoryTrim[i])/factoryTrim[i]; // Report percent differences
   }
   
}

        

uint8_t readByte(uint8_t address, uint8_t subAddress)
{
  uint8_t data;
	I2C2_ReadDevice(address, subAddress, &data, 1);
  return data; 
}

void readBytes(uint8_t address, uint8_t subAddress, uint8_t count, uint8_t * dest)
{  
	I2C2_ReadDevice(address, subAddress, dest, count);
}

void MadgwickQuaternionUpdate(double ax, double ay, double az, double gx, double gy, double gz)
        {
            double q1 = q[0], q2 = q[1], q3 = q[2], q4 = q[3];
            double norm;
            double f1, f2, f3;
            double J_11or24, J_12or23, J_13or22, J_14or21, J_32, J_33;
            double qDot1, qDot2, qDot3, qDot4;
            double hatDot1, hatDot2, hatDot3, hatDot4;
            double gerrx, gerry, gerrz, gbiasx, gbiasy, gbiasz; 

            double _halfq1 = 0.5 * q1;
            double _halfq2 = 0.5 * q2;
            double _halfq3 = 0.5 * q3;
            double _halfq4 = 0.5 * q4;
            double _2q1 = 2.0 * q1;
            double _2q2 = 2.0 * q2;
            double _2q3 = 2.0 * q3;
            double _2q4 = 2.0 * q4;

            norm = sqrt(ax * ax + ay * ay + az * az);
            if (norm == 0.0) return;
            norm = 1.0/norm;
            ax *= norm;
            ay *= norm;
            az *= norm;
            
            f1 = _2q2 * q4 - _2q1 * q3 - ax;
            f2 = _2q1 * q2 + _2q3 * q4 - ay;
            f3 = 1.0 - _2q2 * q2 - _2q3 * q3 - az;
            J_11or24 = _2q3;
            J_12or23 = _2q4;
            J_13or22 = _2q1;
            J_14or21 = _2q2;
            J_32 = 2.0 * J_14or21;
            J_33 = 2.0 * J_11or24;
          
            hatDot1 = J_14or21 * f2 - J_11or24 * f1;
            hatDot2 = J_12or23 * f1 + J_13or22 * f2 - J_32 * f3;
            hatDot3 = J_12or23 * f2 - J_33 *f3 - J_13or22 * f1;
            hatDot4 = J_14or21 * f1 + J_11or24 * f2;
            
            norm = sqrt(hatDot1 * hatDot1 + hatDot2 * hatDot2 + hatDot3 * hatDot3 + hatDot4 * hatDot4);
            hatDot1 /= norm;
            hatDot2 /= norm;
            hatDot3 /= norm;
            hatDot4 /= norm;
            
            gerrx = _2q1 * hatDot2 - _2q2 * hatDot1 - _2q3 * hatDot4 + _2q4 * hatDot3;
            gerry = _2q1 * hatDot3 + _2q2 * hatDot4 - _2q3 * hatDot1 - _2q4 * hatDot2;
            gerrz = _2q1 * hatDot4 - _2q2 * hatDot3 + _2q3 * hatDot2 - _2q4 * hatDot1;
            
            gbiasx += gerrx * deltat * zeta;
            gbiasy += gerry * deltat * zeta;
            gbiasz += gerrz * deltat * zeta;
            gx -= gbiasx;
            gy -= gbiasy;
            gz -= gbiasz;
            
            qDot1 = -_halfq2 * gx - _halfq3 * gy - _halfq4 * gz;
            qDot2 =  _halfq1 * gx + _halfq3 * gz - _halfq4 * gy;
            qDot3 =  _halfq1 * gy - _halfq2 * gz + _halfq4 * gx;
            qDot4 =  _halfq1 * gz + _halfq2 * gy - _halfq3 * gx;

            q1 += (qDot1 -(beta * hatDot1)) * deltat;
            q2 += (qDot2 -(beta * hatDot2)) * deltat;
            q3 += (qDot3 -(beta * hatDot3)) * deltat;
            q4 += (qDot4 -(beta * hatDot4)) * deltat;

            norm = sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);
            norm = 1.0/norm;
            q[0] = q1 * norm;
            q[1] = q2 * norm;
            q[2] = q3 * norm;
            q[3] = q4 * norm;
        }

