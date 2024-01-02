#ifndef _MPU6050_H
#define _MPU6050_H
#include "MPU6050.h"
#include "COMMON.h"
/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define MPU6050_Raw_DATA_TYPE		signed short int
#define MPU6050_DEFAULT_ADR			0x68  	/* MPU6050 default i2c address */
#define MPU6050_ID 					0x68 	/* The correct MPU6050_WHO_AM_I value */
#define MPU6050_DEFAULT_ADR			0x68  	/* MPU6050 default i2c address */
#define MPU6050_HIGH_ADR			0x69	/* MPU6050 i2c address with AD0 on high */
/* Register adress */
#define MPU6050_SELF_TEST_X     	0x0D 	/* Self test factory calibrated values register */
#define MPU6050_SELF_TEST_Y     	0x0E   	/* Self test factory calibrated values register */
#define MPU6050_SELF_TEST_Z      	0x0F 	/* Self test factory calibrated values register */
#define MPU6050_SELF_TEST_A       	0x10	/* Self test factory calibrated values register */
#define MPU6050_SMPLRT_DIV 			0x19  	/* sample rate divisor register	*/
#define MPU6050_CONFIG 				0x1A	/* General configuration register */
#define MPU6050_GYRO_CONFIG 		0x1B 	/* Gyro specfic configuration register */
#define MPU6050_ACCEL_CONFIG   		0x1C 	/* Accelerometer specific configration register */
#define MPU6050_INT_PIN_CONFIG 		0x37 	/* Interrupt pin configuration register */
#define MPU6050_INT_ENABLE 			0x38  	/* Interrupt enable configuration register */
#define MPU6050_INT_STATUS 			0x3A  	/* Interrupt status register */
#define MPU6050_WHO_AM_I 			0x75  	/* Divice ID register */
#define MPU6050_SIGNAL_PATH_RESET 	0x68 	/* Signal path reset register */
#define MPU6050_USER_CTRL 			0x6A   	/* FIFO and I2C Master control register */
#define MPU6050_PWR_MGMT_1 			0x6B   	/* Primary power/sleep control register */
#define MPU6050_PWR_MGMT_2 			0x6C 	/* Secondary power/sleep control register */
#define MPU6050_TEMP_H 				0x41   	/* Temperature data high byte register */
#define MPU6050_TEMP_L 				0x42   	/* Temperature data low byte register */
#define MPU6050_ACCEL_OUT 			0x3B  	/* base address for sensor data reads */
#define MPU6050_MOT_THR 			0x1F   	/* Motion detection threshold bits [7:0] */
#define MPU6050_MOT_DUR				0x20	/* Duration counter threshold for motion int. 1 kHz rate, LSB = 1 ms */
#define MPU6050_ADDR 				0x68
#define MPU6050_ADDRESS				MPU6050_ADDR
#define SMPLRT_DIV_REG 				0x19
#define GYRO_CONFIG_REG 			0x1B
#define ACCEL_CONFIG_REG 			0x1C
#define ACCEL_XOUT_H_REG 			0x3B
#define TEMP_OUT_H_REG 				0x41
#define GYRO_XOUT_H_REG 			0x43
#define PWR_MGMT_1_REG 				0x6B
#define WHO_AM_I_REG 				0x75
/* DMP register addres */
#define MPU6050_DMP_CONFIG_SIZE 		192
#define MPU6050_DMP_CODE_SIZE 			1929
#define MPU6050_DMP_MEMORY_CHUNK_SIZE	16
#define MPU6050_RA_MEM_R_W				0x6F
#define MPU6050_RA_BANK_SEL 			0x6D
#define MPU6050_RA_MEM_START_ADDR   	0x6E
#define MPU6050_RA_I2C_SLV0_ADDR    	0x25
#define MPU6050_RA_USER_CTRL        	0x6A
#define XA_OFFSET_H      				0x06
#define XA_OFFSET_L_TC   				0x07
#define YA_OFFSET_H      				0x08
#define YA_OFFSET_L_TC   				0x09
#define ZA_OFFSET_H      				0x0A
#define ZA_OFFSET_L_TC   				0x0B
/*******************************************************************************
 * Prototypes
 ******************************************************************************/
typedef struct MPU6050_Data_Type{
	double Acc_X;
	double Acc_Y;
	double Acc_Z;
	double Gyro_X;
	double Gyro_Y;
	double Gyro_Z;
}MPU6050_Data_Type;
/*******************************************************************************
 * Global variables
 ******************************************************************************/
extern double Angle_Pitch;
extern double Angle_Roll;
extern double Yaw_Gyro;
extern double angle_pitch_output;
extern double angle_roll_output;
extern double angle_pitch_acc;
extern double angle_roll_acc;
extern MPU6050_Raw_DATA_TYPE Gyro_X_Raw;
extern MPU6050_Raw_DATA_TYPE Gyro_Y_Raw;
extern MPU6050_Raw_DATA_TYPE Gyro_Z_Raw;
/*******************************************************************************
 * API
 ******************************************************************************/
void MPU6050_Write (uint8_t MPU_Address, uint8_t RegisterAddress, uint8_t Data);
void MPU6050_Read (uint8_t MPU_Address, uint8_t RegisterAddress, uint8_t *buffer, uint8_t size);
void MPU6050_Init (void);
void MPU6050_getPara (void);
void MPU6050_CalculateAngle(void);
#endif   /* _MPU6050_H */
