#include "PID.h"
#include <math.h>

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*******************************************************************************
 * Variables
 ******************************************************************************/
/**************************** DEFINE PID VARIABLE *****************************/
/* PID Pitch variable */
static const double PID_Pitch_P_Gain = 0.0;
static const double PID_Pitch_I_Gain = 0.0;
static const double PID_Pitch_D_Gain = 0.0;
static const double PID_Pitch_I_Max  = 0.0;
static double PID_Pitch_I_Stack		 = 0.0;


/* PID Roll variable */
static const double PID_Roll_P_Gain = PID_Pitch_P_Gain;
static const double PID_Roll_I_Gain = PID_Pitch_I_Gain;
static const double PID_Roll_D_Gain = PID_Pitch_D_Gain;
static const double PID_Roll_I_Max  = PID_Pitch_I_Max;
static double PID_Roll_I_Stack 		= 0.0;


/* PID Yaw variable */
static const double PID_Yaw_P_Gain 	= 0.0;
static const double PID_Yaw_I_Gain 	= 0.0;
static const double PID_Yaw_D_Gain 	= 0.0;
static const double PID_Yaw_I_Max  	= 0.0;
static double PID_Yaw_I_Stack 		= 0.0;
/* Desired angle */
static double Desired_Pitch;
static double Desired_Roll;
/********************** END OF DEFINE PID VARIABLE ****************************/
PID_Data_Type PID_Pwm;
/*******************************************************************************
 * Code
 ******************************************************************************/
__inline static void PID_GetDesiredAngle(void) {
	Desired_Roll 	= map(GPIO_PulseWidth.CH1, 1000.0, 2000.0, -30.0, 30.0);
	Desired_Pitch 	= map(GPIO_PulseWidth.CH2, 1000.0, 2000.0, -30.0, 30.0);
}

void PID_Calculate(double *PitchVal, double *RollVal, GPIO_PulseWidth_Type* 
	GPIO_Pwm) {
	PID_GetDesiredAngle();
}
/*******************************************************************************
 * EOF
 ******************************************************************************/
