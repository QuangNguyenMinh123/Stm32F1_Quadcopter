#include "MPU6050.h"
#include "I2C.h"
#include "GPIO.h"
#include <math.h>
/*******************************************************************************
 * Definitions
 ******************************************************************************/

#define ui32					uint32_t
#define CALIBRATION_TIMES		2000
#define DELTA					0.9996
#define WARNING_LED				IO_C13
/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*******************************************************************************
 * Variables
 ******************************************************************************/
 MPU6050_Raw_DATA_TYPE Accel_X_Raw = 0;
 MPU6050_Raw_DATA_TYPE Accel_Y_Raw = 0;
 MPU6050_Raw_DATA_TYPE Accel_Z_Raw = 0;
 MPU6050_Raw_DATA_TYPE Gyro_X_Raw = 0;
 MPU6050_Raw_DATA_TYPE Gyro_Y_Raw = 0;
 MPU6050_Raw_DATA_TYPE Gyro_Z_Raw = 0;
static uint8_t Buffer_data[14];
static MPU6050_Data_Type MPU6050_RawData;
/*****************************ACCELEROMETER VARIABLE***************************/

/*********************************GYRO VARIABLE********************************/
static long Gyro_X_Offset = 0;
static long Gyro_Y_Offset = 0;
static long Gyro_Z_Offset = 0;

static int TotalVector = 0;

double Angle_Pitch = 0.0;
double Angle_Roll = 0.0;
double Yaw_Gyro = 0.0;

 double angle_pitch_acc = 0.0;
 double angle_roll_acc = 0.0;

double angle_pitch_output;
double angle_roll_output;

static bool firstStart = TRUE;
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
	MPU6050_Write(MPU6050_ADDR, PWR_MGMT_1_REG, 0x00);		/* 0x6B */
	MPU6050_Write(MPU6050_ADDR, GYRO_CONFIG_REG, 0x08);		/* 0x1B */
	MPU6050_Write(MPU6050_ADDR, ACCEL_CONFIG_REG, 0x10);	/* 0x1C */
}

void MPU6050_getPara (void)
{
	MPU6050_Read (MPU6050_ADDR, 0x3B, &Buffer_data[0], 14);
	
	Accel_X_Raw = (MPU6050_Raw_DATA_TYPE)(Buffer_data[0] << 8 | Buffer_data [1]);
	Accel_Y_Raw = (MPU6050_Raw_DATA_TYPE)(Buffer_data[2] << 8 | Buffer_data [3]);
	Accel_Z_Raw = (MPU6050_Raw_DATA_TYPE)(Buffer_data[4] << 8 | Buffer_data [5]);
	
	Gyro_X_Raw = (MPU6050_Raw_DATA_TYPE)(Buffer_data[8] << 8 | Buffer_data [9]);
	Gyro_Y_Raw = (MPU6050_Raw_DATA_TYPE)(Buffer_data[10] << 8 | Buffer_data [11]);
	Gyro_Z_Raw = (MPU6050_Raw_DATA_TYPE)(Buffer_data[12] << 8 | Buffer_data [13]);
	
	Gyro_X_Raw 	-= Gyro_X_Offset;
	Gyro_Y_Raw 	-= Gyro_Y_Offset;
	Gyro_Z_Raw 	-= Gyro_Z_Offset;
	
}	
	
void MPU6050_CalculateAngle (void) {
	MPU6050_getPara();
	
	Angle_Pitch	+= Gyro_X_Raw * 0.0000611;
	Angle_Roll 	+= Gyro_Y_Raw * 0.0000611;
	
	Angle_Pitch -= Angle_Roll  * sin(Gyro_Z_Raw * 0.000001066);
	Angle_Roll 	+= Angle_Pitch * sin(Gyro_Z_Raw * 0.000001066);
	
	TotalVector = Accel_X_Raw*Accel_X_Raw + Accel_Y_Raw*Accel_Y_Raw + 
				Accel_Z_Raw*Accel_Z_Raw;
	
	angle_pitch_acc = asin((double)((double)Accel_Y_Raw) / 
			((double)sqrt(TotalVector))) * 57.296;
	angle_roll_acc = asin((double)((double)Accel_X_Raw) / 
			((double)sqrt(TotalVector))) * -57.296;
	
	if (firstStart == TRUE) {
		firstStart = FALSE;
		Angle_Pitch = angle_pitch_acc;
		Angle_Roll = angle_roll_acc;
	}
	else {
		Angle_Pitch = Angle_Pitch * DELTA + angle_pitch_acc * (1.0 - DELTA);
		Angle_Roll = Angle_Roll * DELTA + angle_roll_acc * (1.0 - DELTA);
	}
}

void MPU6050_Calibration (void) {
	uint16_t i_ui16;
	unsigned uint32_t loop_time = 0U;
	Gyro_X_Offset = 0;
	Gyro_Y_Offset = 0;
	Gyro_Z_Offset = 0;
	uint32_t Gyro_X_Cal = 0;
	uint32_t Gyro_Y_Cal = 0;
	uint32_t Gyro_Z_Cal = 0;
	for (i_ui16 = 0; i_ui16 < CALIBRATION_TIMES; i_ui16++) {
		loop_time = micros();
		if (i_ui16 % 20 == 0)
			GPIO_PINToggle(WARNING_LED);
		MPU6050_getPara();
		Gyro_X_Cal += Gyro_X_Raw;
		Gyro_Y_Cal += Gyro_Y_Raw;
		Gyro_Z_Cal += Gyro_Z_Raw;
		while (micros() - loop_time < 4000) {}
	}
	Gyro_X_Offset 	= Gyro_X_Cal / CALIBRATION_TIMES;
	Gyro_Y_Offset 	= Gyro_Y_Cal / CALIBRATION_TIMES;
	Gyro_Z_Offset 	= Gyro_Z_Cal / CALIBRATION_TIMES;	
}

void MPU6050_AngleReset(void) {
	Angle_Pitch = angle_pitch_acc;
	Angle_Roll 	= angle_roll_acc;
}
/*******************************************************************************
 * EOF
 ******************************************************************************/
