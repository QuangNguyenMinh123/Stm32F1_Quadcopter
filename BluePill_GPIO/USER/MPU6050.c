#include "MPU6050.h"
#include "I2C.h"
#include "GPIO.h"
#include <math.h>
/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define MPU6050_Raw_DATA_TYPE	signed short int
#define CALIBRATION_TIMES		1000
#define ALPHA					0.998
/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*******************************************************************************
 * Variables
 ******************************************************************************/
static MPU6050_Raw_DATA_TYPE Accel_X_Raw = 0;
static MPU6050_Raw_DATA_TYPE Accel_Y_Raw = 0;
static MPU6050_Raw_DATA_TYPE Accel_Z_Raw = 0;
static MPU6050_Raw_DATA_TYPE Gyro_Roll_Raw = 0;
static MPU6050_Raw_DATA_TYPE Gyro_Pitch_Raw = 0;
static MPU6050_Raw_DATA_TYPE Gyro_Yaw_Raw = 0;
static uint8_t Buffer_data[14];
static MPU6050_Data_Type MPU6050_RawData;
/*****************************ACCELEROMETER VARIABLE***************************/
 double Pitch_Acc = 0.0;
 double Roll_Acc = 0.0;
/*********************************GYRO VARIABLE********************************/
static long Gyro_Pitch_Offset = 0.0;
static long Gyro_Roll_Offset = 0.0;
static long Gyro_Yaw_Offset = 0.0;

static double Temp_Pitch_Gyro = 0.0;
static double Temp_Roll_Gyro = 0.0;
static double Temp_Yaw_Gyro = 0.0;

 double Pitch_Gyro = 0.0;
 double Roll_Gyro = 0.0;
 double Yaw_Gyro = 0.0;

static bool setGyro = FALSE;

double Pitch;
double Roll;
double Yaw;
/*******************************************************************************
 * Code
 ******************************************************************************/
void MPU6050_Write(uint8_t MPU_Address, uint8_t RegisterAddress, uint8_t Data)
{
	I2C2_WriteToDevice(MPU_Address, RegisterAddress, Data);
}

void MPU6050_Read (uint8_t MPU_Address, uint8_t RegisterAddress, uint8_t *buffer, uint8_t size)
{
	I2C2_ReadDevice(MPU_Address, RegisterAddress, buffer, size);
}

void MPU6050_Init (void)
{
	MPU6050_RawData.Acc_X = 0.0;
	MPU6050_RawData.Acc_Y = 0.0;
	MPU6050_RawData.Acc_Z = 0.0;
	Roll_Gyro 	= 0.0;
	Pitch_Gyro	= 0.0;
	Yaw_Gyro	= 0.0;
	MPU6050_Write(MPU6050_ADDR, PWR_MGMT_1_REG, 0x00);
	MPU6050_Write(MPU6050_ADDR, ACCEL_CONFIG_REG, 0x10);
	MPU6050_Write(MPU6050_ADDR, GYRO_CONFIG_REG, 0x08);
}

void MPU6050_getPara (void)
{
	MPU6050_Read (MPU6050_ADDR, 0x3B, &Buffer_data[0], 14);
	
	Accel_X_Raw = (MPU6050_Raw_DATA_TYPE)(Buffer_data[0] << 8 | Buffer_data [1]);
	Accel_Y_Raw = (MPU6050_Raw_DATA_TYPE)(Buffer_data[2] << 8 | Buffer_data [3]);
	Accel_Z_Raw = (MPU6050_Raw_DATA_TYPE)(Buffer_data[4] << 8 | Buffer_data [5]);
	
	Gyro_Roll_Raw = (MPU6050_Raw_DATA_TYPE)(Buffer_data[8] << 8 | Buffer_data [9]);
	Gyro_Pitch_Raw = (MPU6050_Raw_DATA_TYPE)(Buffer_data[10] << 8 | Buffer_data [11]);
	Gyro_Yaw_Raw = (MPU6050_Raw_DATA_TYPE)(Buffer_data[12] << 8 | Buffer_data [13]);
}	
	
void MPU6050_CalculateAngle (void) {
	MPU6050_getPara();
	
	MPU6050_RawData.Acc_X = ((double) Accel_X_Raw)/ ((double)4096.0);
	MPU6050_RawData.Acc_Y = ((double) Accel_Y_Raw)/ ((double)4096.0);
	MPU6050_RawData.Acc_Z = ((double) Accel_Z_Raw)/ ((double)4096.0);
	
	MPU6050_RawData.Gyro_X = ((double) (Gyro_Roll_Raw - Gyro_Pitch_Offset));
	MPU6050_RawData.Gyro_Y = ((double) (Gyro_Pitch_Raw - Gyro_Roll_Offset));
	MPU6050_RawData.Gyro_Z = ((double) (Gyro_Yaw_Raw - Gyro_Yaw_Offset));
	
	Roll_Acc = RadianToDegree(atan(MPU6050_RawData.Acc_Y/
			sqrt(sqr(MPU6050_RawData.Acc_X) + sqr(MPU6050_RawData.Acc_Z))));
	Pitch_Acc = RadianToDegree(atan(-MPU6050_RawData.Acc_X/
			sqrt(sqr(MPU6050_RawData.Acc_Y) + sqr(MPU6050_RawData.Acc_Z))));
		
	Temp_Roll_Gyro = (MPU6050_RawData.Gyro_X) / 16375.0;
	Temp_Pitch_Gyro = (MPU6050_RawData.Gyro_Y) / 16375.0;
	Temp_Yaw_Gyro = (MPU6050_RawData.Gyro_Z) / 16375.0;
	
	Roll_Gyro 	+= Temp_Roll_Gyro;
	Pitch_Gyro  += Temp_Pitch_Gyro;
	 
	Roll_Gyro -= Pitch_Gyro * sin(Temp_Yaw_Gyro * DEG_TO_RAD);
	Pitch_Gyro += Roll_Gyro * sin(Temp_Yaw_Gyro * DEG_TO_RAD);
	
	if (setGyro) {
		Pitch_Gyro = Pitch_Gyro * ALPHA + Pitch_Acc * (1.0 - ALPHA);
		Roll_Gyro = Roll_Gyro * ALPHA + Roll_Acc * (1.0 - ALPHA);
	} else {
		Pitch_Gyro = Pitch_Acc;
		Roll_Gyro = Roll_Acc;
		setGyro = TRUE;
	}
	
	Pitch = Pitch * 0.9 + Pitch_Gyro * 0.1;
	Roll  = Roll * 0.9 + Roll_Gyro * 0.1;
	Yaw   = Temp_Yaw_Gyro;
}

void MPU6050_Calibration (void) {
	uint16_t i_ui16;
	unsigned uint32_t loop_time = 0U;
	Gyro_Pitch_Offset = 0.0;
	Gyro_Roll_Offset = 0.0;
	Gyro_Yaw_Offset = 0.0;
	Pitch = 0.0;
	Roll = 0.0;
	for (i_ui16 = 0; i_ui16 < CALIBRATION_TIMES; i_ui16++) {
		loop_time = micros();
		MPU6050_getPara();
		MPU6050_RawData.Acc_X = ((double) Accel_X_Raw)/ ((double)4096.0);
		MPU6050_RawData.Acc_Y = ((double) Accel_Y_Raw)/ ((double)4096.0);
		MPU6050_RawData.Acc_Z = ((double) Accel_Z_Raw)/ ((double)4096.0);
		Roll_Acc = RadianToDegree(atan(MPU6050_RawData.Acc_Y/
			sqrt(sqr(MPU6050_RawData.Acc_X) + sqr(MPU6050_RawData.Acc_Z))));
		Pitch_Acc = RadianToDegree(atan(-MPU6050_RawData.Acc_X/
			sqrt(sqr(MPU6050_RawData.Acc_Y) + sqr(MPU6050_RawData.Acc_Z))));
		Pitch += Pitch_Acc;
		Roll  += Roll_Acc;
		
		Gyro_Pitch_Offset += Gyro_Roll_Raw;
		Gyro_Roll_Offset += Gyro_Pitch_Raw;
		Gyro_Yaw_Offset += Gyro_Yaw_Raw;
		while (micros() - loop_time < 4000) {}
	}
	Pitch 				= Pitch / CALIBRATION_TIMES;
	Roll				= Roll  / CALIBRATION_TIMES;
	Gyro_Pitch_Offset 	= Gyro_Pitch_Offset / CALIBRATION_TIMES;
	Gyro_Roll_Offset 	= Gyro_Roll_Offset  / CALIBRATION_TIMES;
	Gyro_Yaw_Offset 	= Gyro_Yaw_Offset  / CALIBRATION_TIMES;	
}

/*******************************************************************************
 * EOF
 ******************************************************************************/
