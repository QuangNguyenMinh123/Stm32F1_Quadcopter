#include "GPIO.h"
#include "UART.h"
#include "CLOCK.h"
#include "I2C.h"
#include "MPU6050.h"
#include "math.h"
#include "EX_EEPROM.h"
#include "PID.h"
/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define AAA		120
/*******************************************************************************
 * Prototypes
 ******************************************************************************/
void System_Init(void);
void check(int a);
/*******************************************************************************
 * Variables
 ******************************************************************************/
static unsigned uint32_t loop_timer = 0U;
double gyroBias[3] = {0, 0, 0}, accelBias[3] = {0, 0, 0};
uint8_t checkIMU[AAA] = {0};
double SelfTest[6];
/*******************************************************************************
 * Code
 ******************************************************************************/

int main(void) {
	CLOCK_SystickInit();
	UART1_Init(UART_BAUDRATE_115200);
	I2C2_Init(I2C_SPEED_100);
	check(1);
	MPU6050SelfTest(SelfTest);
	check(2);
	if (SelfTest[0] < 1.0 && SelfTest[1] < 1.0 && SelfTest[2] < 1.0 &&
		SelfTest[3] < 1.0 && SelfTest[4] < 1.0 && SelfTest[5] < 1.0) {
		calibrateMPU6050(gyroBias, accelBias);
		check(3);
		initMPU6050();
		check(4);
	}
		
	while (1) {
		MPU6050_CalculateAngle();
	}
}

void check(int a) {
	uint32_t i = 0;
	uint8_t Buffer = 0;
	UART1_sendStr("Lan ");
	UART1_sendNum(a);
	UART1_sendByte('\n');
	for (i=0;i<AAA;i++) {
		Buffer = 0xff;
		I2C2_Start ();
		I2C2_CallAdress ( (uint8_t) ((MPU6050_ADDRESS<<1) | I2C_WRITE) );
		I2C2_Write (i);
		I2C2_Start ();
		I2C2_Read ((uint8_t) ((MPU6050_ADDRESS<<1) | I2C_READ), &Buffer, 1);
		I2C2_Stop ();
		UART1_sendStr("Reg ");
		UART1_sendNum(i);
		UART1_sendStr(": ");
		UART1_sendNum(Buffer);
		UART1_sendByte('\n');
		
	}
}


/*******************************************************************************
 * EOF
 ******************************************************************************/
