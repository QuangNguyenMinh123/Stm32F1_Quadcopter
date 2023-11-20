#ifndef _PID_H
#define _PID_H
#include "COMMON.h"
#include "MPU6050.h"
#include "GPIO.h"
/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define TIME_US						1000000 /* 1.000.000 us per sec*/
#define TIME_MS						1000	/* 1.000.000 us per sec*/
#define REFRESH_RATE				200		/* Refresh rate */
#define TIME_LOOP_US				TIME_US/REFRESH_RATE	/* Take 5ms to update PWM */
#define TIME_LOOP_MS				TIME_MS/REFRESH_RATE	/* Take 5ms to update PWM */
/*******************************************************************************
 * Prototypes
 ******************************************************************************/
typedef struct{
	uint16_t FrontRight;
	uint16_t FrontLeft;
	uint16_t BackLeft;
	uint16_t BackRight;
}PID_Data_Type;

/*******************************************************************************
 * Global variables
 ******************************************************************************/
extern PID_Data_Type PID_Pwm;
/*******************************************************************************
 * API
 ******************************************************************************/
void PID_Reset(void);
void PID_Calculate(double *PitchVal, double *RollVal, 
	GPIO_PulseWidth_Type* GPIO_Pwm);
PID_Data_Type* PID_GetPwm(void);
#endif   /* _PID_H */
