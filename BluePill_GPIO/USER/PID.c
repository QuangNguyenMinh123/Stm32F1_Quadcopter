#include "PID.h"
#include <math.h>

/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define ui16 					uint16_t
/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*******************************************************************************
 * Variables
 ******************************************************************************/
/**************************** DEFINE PID VARIABLE *****************************/
/* PID Pitch variable */
static const double PID_Pitch_P_Gain = 1.3;
static const double PID_Pitch_I_Gain = 0.0112;
static const double PID_Pitch_D_Gain = 17.5;
static const double PID_Pitch_I_Max  = 400.0;

/* PID Roll variable */
static const double PID_Roll_P_Gain = PID_Pitch_P_Gain;
static const double PID_Roll_I_Gain = PID_Pitch_I_Gain;
static const double PID_Roll_D_Gain = PID_Pitch_D_Gain;
static const double PID_Roll_I_Max  = PID_Pitch_I_Max;


/* PID Yaw variable */
static const double PID_Yaw_P_Gain 	= 4.0;
static const double PID_Yaw_I_Gain 	= 0.015;
static const double PID_Yaw_D_Gain 	= 0.0112;
static const double PID_Yaw_I_Max  	= 400.0;
/* Desired angle */
static double Desired_Pitch;
static double Desired_Roll;
static double Desired_Yaw;
/* Variable declaration */
/* Error */
static double Pitch_Error = 0.0;
static double Roll_Error = 0.0;
static double Yaw_Error = 0.0;
/* P element */
static double P_Pitch = 0.0;
static double P_Roll = 0.0;
static double P_Yaw = 0.0;
/* I element */
 double I_Pitch = 0.0;			/* I of Pitch element accumulation */
 double I_Roll = 0.0;				/* I of Roll element accumulation */
 double I_Yaw = 0.0;				/* I of Yaw element accumulation */
/* D element */
static double D_Pitch = 0.0;
static double D_Roll = 0.0;
static double D_Yaw = 0.0;
static double Pre_Pitch_Error = 0.0;
static double Pre_Roll_Error = 0.0;
static double Pre_Yaw_Error = 0.0;
/********************** END OF DEFINE PID VARIABLE ****************************/
PID_Data_Type PID_Pwm;
/*******************************************************************************
 * Code
 ******************************************************************************/
void PID_Reset(void) {
	I_Pitch = 0.0;
	I_Roll 	= 0.0;
	I_Yaw	= 0.0;
}

void PID_Calculate(double *PitchVal, double *RollVal, double *YawVal,
	GPIO_PulseWidth_Type* GPIO_Pwm) {
	Desired_Roll 	= map(GPIO_PulseWidth.Roll, 1000.0, 2000.0, -30.0, 30.0);
	Desired_Pitch 	= map(GPIO_PulseWidth.Pitch, 1000.0, 2000.0, -30.0, 30.0);
	Desired_Yaw     = map(GPIO_PulseWidth.Yaw, 1000.0, 2000.0, -90.0, 90.0);
	/* Calculating e(t) */
	Pitch_Error = Desired_Pitch - *PitchVal;
	Roll_Error  = Desired_Roll  - *RollVal;
	Yaw_Error   = Desired_Yaw   - *YawVal;
	/* Calculate P element */
	P_Pitch = Pitch_Error * PID_Pitch_P_Gain;
	P_Roll 	= Roll_Error  * PID_Roll_P_Gain;
	P_Yaw 	= Yaw_Error   * PID_Yaw_P_Gain;
	/* Calculate D element */
	D_Pitch = (Pitch_Error - Pre_Pitch_Error) * PID_Pitch_D_Gain;
	D_Roll  = (Roll_Error  - Pre_Roll_Error)  * PID_Roll_D_Gain;
	D_Yaw   = (Yaw_Error   - Pre_Yaw_Error)   * PID_Yaw_D_Gain;
	/* Calculate I element */
	I_Pitch = I_Pitch + Pitch_Error * PID_Pitch_I_Gain;
	I_Roll	= I_Roll  + Roll_Error  * PID_Roll_I_Gain;
	I_Yaw	= I_Yaw   + Yaw_Error   * PID_Yaw_I_Gain;
	if (I_Pitch > PID_Pitch_I_Max)
		I_Pitch = PID_Pitch_I_Max;
	if (I_Roll  > PID_Roll_I_Max)
		I_Roll  = PID_Roll_I_Max;
	if (I_Yaw   > PID_Yaw_I_Max)
		I_Yaw   = PID_Yaw_I_Max;
	if (I_Pitch < -PID_Pitch_I_Max)
		I_Pitch = -PID_Pitch_I_Max;
	if (I_Roll  < -PID_Roll_I_Max)
		I_Roll  = -PID_Roll_I_Max;
	if (I_Yaw   < -PID_Yaw_I_Max)
		I_Yaw   = -PID_Yaw_I_Max;
	/* Sum up */
	PID_Pwm.FrontRight = GPIO_Pwm->Throttle - (ui16)P_Pitch - (ui16)P_Roll
		- (ui16)D_Pitch + (ui16)D_Roll - (ui16)I_Roll - (ui16)I_Pitch;
	PID_Pwm.FrontLeft  = GPIO_Pwm->Throttle - (ui16)P_Pitch + (ui16)P_Roll
		- (ui16)D_Pitch - (ui16)D_Roll + (ui16)I_Roll - (ui16)I_Pitch;
	PID_Pwm.BackLeft   = GPIO_Pwm->Throttle + (ui16)P_Pitch + (ui16)P_Roll
		+ (ui16)D_Pitch - (ui16)D_Roll + (ui16)I_Roll + (ui16)I_Pitch;
	PID_Pwm.BackRight  = GPIO_Pwm->Throttle + (ui16)P_Pitch - (ui16)P_Roll
		+ (ui16)D_Pitch + (ui16)D_Roll - (ui16)I_Roll + (ui16)I_Pitch;
	/* Saving error */
	Pre_Pitch_Error = Pitch_Error;
	Pre_Roll_Error  = Roll_Error;
	Pre_Yaw_Error   = Yaw_Error;
}
/*******************************************************************************
 * EOF
 ******************************************************************************/
