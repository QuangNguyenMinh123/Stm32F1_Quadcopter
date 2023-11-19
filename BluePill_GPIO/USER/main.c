#include "GPIO.h"
#include "CLOCK.h"
#include "I2C.h"
#include "math.h"
#include "MPU6050.h"
#include "EX_EEPROM.h"
#include "PID.h"
#include "system_stm32f10x.h"
/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define ON						1
#define OFF						0
#define TUNING_PID				OFF
#if (TUNING_PID == ON)
#include "UART.h"
#endif
#define PWM_FREQ				250
#define WARNING_LED				IO_C13
#define GREEN_LED1				IO_B14
#define GREEN_LED2				IO_B15
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
	/*SystemCoreClockUpdate();*/
	uint8_t loopCounter = 0;
	System_Init();
	GPIO_SetPWM(IO_B6, PWM_FREQ);
	GPIO_SetPWM(IO_B7, PWM_FREQ);
	GPIO_SetPWM(IO_B8, PWM_FREQ);
	GPIO_SetPWM(IO_B9, PWM_FREQ);
	GPIO_PINLow(WARNING_LED);
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
			GPIO_PINToggle(GREEN_LED1);
			loopCounter = 0;
		}
		while (micros() - loop_timer < 4000);
	}
}

void System_Init(void) {
	int i=0;
	CLOCK_SystickInit();
	GPIO_SetPWMMeasurement();
	GPIO_SetOutPut(IO_C13, General_Push_Pull);
	GPIO_SetOutPut(IO_B14, General_Push_Pull);
	GPIO_SetOutPut(IO_B15, General_Push_Pull);
	GPIO_PINHigh(IO_C13);
	GPIO_PINHigh(IO_B14);
	GPIO_PINHigh(IO_B15);
	
	for (;i<=10;i++)
	{
		GPIO_PINToggle(WARNING_LED);
		delay(100*MS);
	}
	
#if (TUNING_PID == ON)
	UART1_Init(UART_BAUDRATE_115200);
#endif
	I2C2_Init(I2C_SPEED_400);
	MPU6050_Init();
	MPU6050_Calibration();
}
