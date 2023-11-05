#include "GPIO.h"
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
#if (TUNING_PID == ON)
#include "UART.h"
#endif
/*******************************************************************************
 * Prototypes
 ******************************************************************************/
void System_Init(void);
/*******************************************************************************
 * Variables
 ******************************************************************************/
static unsigned uint32_t loop_timer = 0U;
/*******************************************************************************
 * Code
 ******************************************************************************/

int main(void) {
	uint8_t loopCounter = 0;
	System_Init();
	GPIO_SetPWM(IO_B6);
	GPIO_SetPWM(IO_B7);
	GPIO_SetPWM(IO_B8);
	GPIO_SetPWM(IO_B9);
	GPIO_B6_PWM(1000);
	GPIO_B7_PWM(1000);
	GPIO_B8_PWM(1000);
	GPIO_B9_PWM(1000);
/*	if (GPIO_PulseWidth.CH4 >= 1950) {
		GPIO_PINLow(IO_B14);
		GPIO_PINLow(IO_B15);
	}
	while (GPIO_PulseWidth.CH4 >= 1100);*/
	delay(5*SEC);
	GPIO_PINLow(IO_B14);
	GPIO_PINLow(IO_B15);
	while (1) {
		loopCounter++;
		loop_timer = micros();
		MPU6050_CalculateAngle();
		PID_GetDesiredAngle();
		if (GPIO_PulseWidth.CH4 < 1000) {
			GPIO_PulseWidth.CH4 = 1000;
		}
		GPIO_B6_PWM(GPIO_PulseWidth.CH4);
		GPIO_B7_PWM(GPIO_PulseWidth.CH4);
		GPIO_B8_PWM(GPIO_PulseWidth.CH4);
		GPIO_B9_PWM(GPIO_PulseWidth.CH4);
		if (loopCounter == 125) {
			GPIO_PINToggle(IO_C13);
			loopCounter = 0;
		}
		while (micros() - loop_timer < 4000);
	}
}

void System_Init(void) {
	CLOCK_SystickInit();
	GPIO_SetPWMMeasurement();
	GPIO_SetOutPut(IO_C13, General_Push_Pull);
	GPIO_SetOutPut(IO_B14, General_Push_Pull);
	GPIO_SetOutPut(IO_B15, General_Push_Pull);
	GPIO_PINHigh(IO_C13);
	GPIO_PINHigh(IO_B14);
	GPIO_PINHigh(IO_B15);
#if (TUNING_PID == ON)
	UART1_Init(UART_BAUDRATE_115200);
#endif
	I2C2_Init(I2C_SPEED_400);
	MPU6050_Init();
	MPU6050_Calibration();
}
