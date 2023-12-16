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
 const double PID_Pitch_P_Gain = 1.3;			/* 1.3 */
 const double PID_Pitch_I_Gain = 0.04;			/* 0.0112 */
 const double PID_Pitch_D_Gain = 18;				/* 17.5 */
 const double PID_Max_Pitch  = 400.0;

/* PID Roll variable */
 const double PID_Roll_P_Gain = PID_Pitch_P_Gain;
 const double PID_Roll_I_Gain = PID_Pitch_I_Gain;
 const double PID_Roll_D_Gain = PID_Pitch_D_Gain;
 const double PID_Max_Roll  = PID_Max_Pitch;


/* PID Yaw variable */
 const double PID_Yaw_P_Gain 	= 4.0;		/* 4.0  */
 const double PID_Yaw_I_Gain 	= 0.2;		/* 0.015 */
 const double PID_Yaw_D_Gain 	= 0.0;		/* 0 */
 const double PID_Max_Yaw  	= 400.0;
/* Desired angle */
static double Desired_Pitch;
static double Desired_Roll;
 double Desired_Yaw;
/* Variable declaration */
/* Error */
 double Pitch_Error = 0.0;
 double Roll_Error = 0.0;
 double Yaw_Error = 0.0;
/* P element */
 double P_Pitch = 0.0;
 double P_Roll = 0.0;
 double P_Yaw = 0.0;
/* I element */
 double I_Pitch = 0.0;			/* I of Pitch element accumulation */
 double I_Roll = 0.0;				/* I of Roll element accumulation */
 double I_Yaw = 0.0;				/* I of Yaw element accumulation */
/* D element */
 double D_Pitch = 0.0;
 double D_Roll = 0.0;
 double D_Yaw = 0.0;
 double Pre_Pitch_Error = 0.0;
 double Pre_Roll_Error = 0.0;
 double Pre_Yaw_Error = 0.0;
 double Battery_Compensation = 40.0;
/* Sum up */
 double PID_OutputPitch = 0.0;
 double PID_OutputRoll = 0.0;
 double PID_OutputYaw = 0.0;
/********************** END OF DEFINE PID VARIABLE ****************************/
PID_Data_Type PID_Pwm;
/*******************************************************************************
 * Code
 ******************************************************************************/
void PID_Reset(void) {
	I_Pitch = 0.0;
	I_Roll 	= 0.0;
	I_Yaw	= 0.0;
	Pre_Pitch_Error = 0.0;
	Pre_Roll_Error = 0.0;
	Pre_Yaw_Error = 0.0;
}

void PID_Calculate(double *PitchVal, double *RollVal, double *YawVal,
	GPIO_PulseWidth_Type* GPIO_Pwm, double* BatLevel) {
	Desired_Roll 	= map(GPIO_PulseWidth.Roll, 1000.0, 2000.0, -30.0, 30.0);
	Desired_Pitch 	= map(GPIO_PulseWidth.Pitch, 1000.0, 2000.0, -30.0, 30.0);
	Desired_Yaw     = map(GPIO_PulseWidth.Yaw, 1000.0, 2000.0, -90.0, 90.0);
	if (GPIO_PulseWidth.Yaw >= 1450 && GPIO_PulseWidth.Yaw <= 1550) {
		Desired_Yaw = 0;
		I_Yaw = 0;
	}
	/* Calculating e(t) */
	Pitch_Error = *PitchVal - Desired_Pitch;
	Roll_Error  = *RollVal 	- Desired_Roll;
	Yaw_Error   = *YawVal	- Desired_Yaw;
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
	if (I_Pitch > PID_Max_Pitch)
		I_Pitch = PID_Max_Pitch;
	if (I_Roll  > PID_Max_Roll)
		I_Roll  = PID_Max_Roll;
	if (I_Yaw   > PID_Max_Yaw)
		I_Yaw   = PID_Max_Yaw;
	if (I_Pitch < -PID_Max_Pitch)
		I_Pitch = -PID_Max_Pitch;
	if (I_Roll  < -PID_Max_Roll)
		I_Roll  = -PID_Max_Roll;
	if (I_Yaw   < -PID_Max_Yaw)
		I_Yaw   = -PID_Max_Yaw;
	/* PID element sum up */
	PID_OutputPitch = P_Pitch + I_Pitch + D_Pitch;
	PID_OutputRoll	= P_Roll + I_Roll + D_Roll;
	PID_OutputYaw 	= P_Yaw + I_Yaw + D_Yaw;
	if (PID_OutputPitch > PID_Max_Pitch)
		PID_OutputPitch = PID_Max_Pitch;
	else if (PID_OutputPitch < PID_Max_Pitch * -1)	
		PID_OutputPitch = PID_Max_Pitch * -1;
	if (PID_OutputRoll > PID_Max_Roll)
		PID_OutputRoll = PID_Max_Roll;
	else if (PID_OutputRoll < PID_Max_Roll * -1)	
		PID_OutputRoll = PID_Max_Roll * -1;
	if (PID_OutputYaw > PID_Max_Yaw)
		PID_OutputYaw = PID_Max_Yaw;
	else if (PID_OutputYaw < PID_Max_Yaw * -1)	
		PID_OutputYaw = PID_Max_Yaw * -1;
	/* Limit throttle*/
//	if (GPIO_Pwm->Throttle > 1800)
//		GPIO_Pwm->Throttle = 1800;
//	/* Sum up */
//	PID_Pwm.FrontRight = GPIO_Pwm->Throttle - (ui16)PID_OutputPitch 
//		+ (ui16)PID_OutputRoll - (ui16)PID_OutputYaw;
//	PID_Pwm.FrontLeft  = GPIO_Pwm->Throttle - (ui16)PID_OutputPitch
//		- (ui16)PID_OutputRoll + (ui16)PID_OutputYaw;
//	PID_Pwm.BackLeft   = GPIO_Pwm->Throttle + (ui16)PID_OutputPitch
//		- (ui16)PID_OutputRoll - (ui16)PID_OutputYaw;
//	PID_Pwm.BackRight  = GPIO_Pwm->Throttle + (ui16)PID_OutputPitch
//		+ (ui16)PID_OutputRoll + (ui16)PID_OutputYaw;
//	/* Battery compensation */
//	PID_Pwm.FrontRight 	+= (ui16) ((12.4 - *BatLevel) * Battery_Compensation);
//	PID_Pwm.FrontLeft 	+= (ui16) ((12.4 - *BatLevel) * Battery_Compensation);
//	PID_Pwm.BackLeft 	+= (ui16) ((12.4 - *BatLevel) * Battery_Compensation);
//	PID_Pwm.BackRight 	+= (ui16) ((12.4 - *BatLevel) * Battery_Compensation);
	
	/* Saving error */
	Pre_Pitch_Error = Pitch_Error;
	Pre_Roll_Error  = Roll_Error;
	Pre_Yaw_Error   = Yaw_Error;
}
/*******************************************************************************
 * EOF
 ******************************************************************************/
