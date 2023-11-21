#include "GPIO.h"
#include "CLOCK.h"
#include "I2C.h"
#include "MPU6050.h"
#include "PID.h"
/*	This lib used for checking clock speed
#include "system_stm32f10x.h"
*/
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
#define MAX_THROTTLE			2000
#define MIN_THROTTLE			1000
/*******************************************************************************
 * Prototypes
 ******************************************************************************/
typedef enum{
	IDLE,
	TAKE_OFF,
	STAND_BY
}FlyingStateType;
/*******************************************************************************
 * Variables
 ******************************************************************************/
static unsigned uint32_t loop_timer = 0U;
static FlyingStateType FlyingState = IDLE;
static uint8_t loopCounter = 0;
static uint8_t ErrorIdx = 0;
/*******************************************************************************
 * API
 ******************************************************************************/
void System_Init(void);
FlyingStateType GetFlyingState(void);
void FlyingMode_IDLE(void);
void FlyingMode_TAKE_OFF(void);
void FlyingMode_STAND_BY(void);
void CheckTimeOut(void);
void LedWarning_NotInIdleMode(void);
/*******************************************************************************
 * Code
 ******************************************************************************/

int main(void) {
	/*SystemCoreClockUpdate();*/
	
	System_Init();
	GPIO_SetPWM(IO_B6, PWM_FREQ);
	GPIO_SetPWM(IO_B7, PWM_FREQ);
	GPIO_SetPWM(IO_B8, PWM_FREQ);
	GPIO_SetPWM(IO_B9, PWM_FREQ);
	FlyingMode_IDLE();
	while (GetFlyingState() != IDLE) {
		LedWarning_NotInIdleMode();
	}
	GPIO_PINHigh(WARNING_LED);
	while (1) {
		loopCounter++;
		loop_timer = micros();
		FlyingState = GetFlyingState();
		if (FlyingState == IDLE) {
			FlyingMode_IDLE();
		}
		if (FlyingState == TAKE_OFF) {
			FlyingMode_TAKE_OFF();
		}
		if (FlyingState == STAND_BY) {
			FlyingMode_TAKE_OFF();
		}
		if (loopCounter == 125) {
			GPIO_PINToggle(GREEN_LED1);
			loopCounter = 0;
		}
#if (TUNING_PID == ON)
		UART1_sendStr("FR:");
		UART1_sendNum(PID_Pwm.FrontRight);
		UART1_sendStr("\t");
		UART1_sendStr("FL:");
		UART1_sendNum(PID_Pwm.FrontLeft);
		UART1_sendStr("\t");
		UART1_sendStr("BL:");
		UART1_sendNum(PID_Pwm.BackLeft);
		UART1_sendStr("\t");
		UART1_sendStr("BR:");
		UART1_sendNum(PID_Pwm.BackRight);
		UART1_sendStr("\n");
#endif
		CheckTimeOut();
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

FlyingStateType GetFlyingState(void)
{
	FlyingStateType Ret_en;
	if (GPIO_PulseWidth.Aux1 < 1250)
	{
		Ret_en = IDLE;
	}
	else if (GPIO_PulseWidth.Aux1 < 1750)
	{
		Ret_en = TAKE_OFF;
	}
	else
		Ret_en = STAND_BY;
	return Ret_en;
}

void FlyingMode_IDLE(void) {
	GPIO_B6_PWM(1000);
	GPIO_B7_PWM(1000);
	GPIO_B8_PWM(1000);
	GPIO_B9_PWM(1000);
	PID_Reset();
}

void FlyingMode_TAKE_OFF(void) {
	MPU6050_CalculateAngle();
	if (GPIO_PulseWidth.Throttle < 1000)
		GPIO_PulseWidth.Throttle = 1000;
	PID_Calculate(&Pitch, &Roll, &Yaw, &GPIO_PulseWidth);
	if (PID_Pwm.FrontRight > MAX_THROTTLE)
		PID_Pwm.FrontRight = MAX_THROTTLE;
	if (PID_Pwm.FrontLeft  > MAX_THROTTLE)
		PID_Pwm.FrontLeft  = MAX_THROTTLE;
	if (PID_Pwm.BackLeft   > MAX_THROTTLE)
		PID_Pwm.BackLeft   = MAX_THROTTLE;
	if (PID_Pwm.BackRight  > MAX_THROTTLE)
		PID_Pwm.BackRight  = MAX_THROTTLE;
	if (PID_Pwm.FrontRight < MIN_THROTTLE)
		PID_Pwm.FrontRight = MIN_THROTTLE;
	if (PID_Pwm.FrontLeft  < MIN_THROTTLE)
		PID_Pwm.FrontLeft  = MIN_THROTTLE;
	if (PID_Pwm.BackLeft   < MIN_THROTTLE)
		PID_Pwm.BackLeft   = MIN_THROTTLE;
	if (PID_Pwm.BackRight  < MIN_THROTTLE)
		PID_Pwm.BackRight  = MIN_THROTTLE;
	GPIO_B6_PWM(PID_Pwm.FrontRight);
	GPIO_B7_PWM(PID_Pwm.FrontLeft);
	GPIO_B8_PWM(PID_Pwm.BackLeft);
	GPIO_B9_PWM(PID_Pwm.BackRight);
}

void FlyingMode_STAND_BY(void) {

}

void CheckTimeOut(void) {
	if (micros() - loop_timer > 4000)
		ErrorIdx++;
	if (ErrorIdx)
		GPIO_PINLow(WARNING_LED);
}

void LedWarning_NotInIdleMode(void) {
	uint8_t i = 0;
	GPIO_PINHigh(WARNING_LED);
	for (i = 0; i <= 10; i++) {
		GPIO_PINToggle(WARNING_LED);
		delay(100*MS);
	}
	while (i > 0) {
		i--;
		delay(100*MS);
	}
}
