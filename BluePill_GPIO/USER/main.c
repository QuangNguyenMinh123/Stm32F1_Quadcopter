#include "GPIO.h"
#include "UART.h"
#include "CLOCK.h"
#include "I2C.h"
#include "math.h"
#include "MPU6050.h"
#include "EX_EEPROM.h"
#include "PID.h"
/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define ON						1
#define OFF						0
#define TUNING_PID				OFF
/*******************************************************************************
 * Prototypes
 ******************************************************************************/
void System_Init(void);
/*******************************************************************************
 * Variables
 ******************************************************************************/
static unsigned uint32_t loop_timer = 0U;
char a[100]={0};
/*******************************************************************************
 * Code
 ******************************************************************************/

int main(void) {
	System_Init();
	GPIO_B6_PWM(1000);
	GPIO_B7_PWM(1250);
	GPIO_B8_PWM(1500);
	GPIO_B9_PWM(2000);
	EEPROM_Read(0,a,10);
	GPIO_SetPWMMeasurement();
	while (1) {
		loop_timer = micros();
		MPU6050_CalculateAngle();
		while (micros() - loop_timer < 4000);
	}
}

void System_Init(void) {
	CLOCK_SystickInit();
	
	GPIO_SetOutPut(IO_C13, General_Push_Pull);
	GPIO_SetOutPut(IO_B14, General_Push_Pull);
	GPIO_SetOutPut(IO_B15, General_Push_Pull);
	GPIO_PINHigh(IO_C13);
	GPIO_PINHigh(IO_B14);
	GPIO_PINHigh(IO_B15);
	GPIO_SetPWM(IO_B6);
	GPIO_SetPWM(IO_B7);
	GPIO_SetPWM(IO_B8);
	GPIO_SetPWM(IO_B9);
#if (TUNING_PID == ON)
	UART1_Init(UART_BAUDRATE_115200);
#endif
	I2C2_Init(I2C_SPEED_400);
	MPU6050_Init();
	MPU6050_Calibration();
}
