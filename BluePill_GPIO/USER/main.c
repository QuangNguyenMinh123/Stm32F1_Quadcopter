#include "GPIO.h"
#include "CLOCK.h"
#include "I2C.h"
#include "MPU6050.h"
#include "PID.h"
#include "LCD.h"
/*	This lib used for checking clock speed
#include "system_stm32f10x.h"
*/
/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define PINLOW					1
#define TIMEOUT					2
#define TIMEOUT_PINLOW			3
#define ON						1
#define OFF						0
#define UART_ENABLE				OFF
#if (UART_ENABLE == ON)
#include "UART.h"
#endif
#define PWM_FREQ				250
#define WARNING_LED				IO_C13
#define GREEN_LED1				IO_B14
#define GREEN_LED2				IO_B15
#define MAX_THROTTLE			2000
#define MIN_THROTTLE			1100
#define ui16 					uint16_t
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
static uint8_t loopCounter = 0;
static bool ErrorTimeout = FALSE;
static double Battery = 0;
static uint8_t Error = 0;
static uint8_t start = 0;
#if (UART_ENABLE == ON)
char buffer[10];
#endif
double roll_level_adjust, pitch_level_adjust;
double pid_error_temp;
double pid_i_mem_roll, pid_roll_setpoint, gyro_roll_input, pid_output_roll, pid_last_roll_d_error;
double pid_i_mem_pitch, pid_pitch_setpoint, gyro_pitch_input, pid_output_pitch, pid_last_pitch_d_error;
double pid_i_mem_yaw, pid_yaw_setpoint, gyro_yaw_input, pid_output_yaw, pid_last_yaw_d_error;
double pid_p_gain_roll = 1.3;               //Gain setting for the pitch and roll P-controller (default = 1.3).
double pid_i_gain_roll = 0.04;              //Gain setting for the pitch and roll I-controller (default = 0.04).
double pid_d_gain_roll = 18.0;              //Gain setting for the pitch and roll D-controller (default = 18.0).
int pid_max_roll = 400;                    //Maximum output of the PID-controller (+/-).

double pid_p_gain_pitch ;  //Gain setting for the pitch P-controller.
double pid_i_gain_pitch ;  //Gain setting for the pitch I-controller.
double pid_d_gain_pitch ;  //Gain setting for the pitch D-controller.
int pid_max_pitch = 400;          //Maximum output of the PID-controller (+/-).

double pid_p_gain_yaw = 4.0;                //Gain setting for the pitch P-controller (default = 4.0).
double pid_i_gain_yaw = 0.02;               //Gain setting for the pitch I-controller (default = 0.02).
double pid_d_gain_yaw = 0.0;                //Gain setting for the pitch D-controller (default = 0.0).
int pid_max_yaw = 400; 
uint16_t throttle;
uint16_t FrontRight, RearRight, RearLeft, FrontLeft;
/*******************************************************************************
 * API
 ******************************************************************************/
void System_Init(void);
void CheckTimeOut(void);
void LedWarning_NotInIdleMode(void);
bool TX_Unavailable(void);
void ESC_Clibration(void);
/*******************************************************************************
 * Code
 ******************************************************************************/
extern double PID_OutputPitch ;
extern double PID_OutputRoll ;
extern double PID_OutputYaw ;

int main(void) {
	/*SystemCoreClockUpdate();*/
	pid_p_gain_pitch = pid_p_gain_roll;
	pid_i_gain_pitch = pid_i_gain_roll;
	pid_d_gain_pitch = pid_d_gain_roll;
	pid_max_pitch = pid_max_roll;
	
	System_Init();
	if (GPIO_PulseWidth.Throttle >= 1900) {
		ESC_Clibration();
	}
	while (TX_Unavailable()) {
		LedWarning_NotInIdleMode();
	}
	GPIO_PINHigh(WARNING_LED);
	loop_timer = micros();
	while (1) {
		loopCounter++;
		MPU6050_getPara();
		gyro_roll_input = (gyro_roll_input * 0.7) + (((double)Gyro_Y_Raw / 65.5) * 0.3);   
		gyro_pitch_input = (gyro_pitch_input * 0.7) + (((double)Gyro_X_Raw / 65.5) * 0.3);
		gyro_yaw_input = (gyro_yaw_input * 0.7) + (((double)Gyro_Z_Raw / 65.5) * 0.3);  
		
		MPU6050_CalculateAngle();
		
		pitch_level_adjust = Angle_Pitch * 15;
		roll_level_adjust = Angle_Roll * 15;
		
		//For starting the motors: throttle low and yaw left (step 1).
		if (GPIO_PulseWidth.Throttle < 1050 && GPIO_PulseWidth.Yaw < 1050)start = 1;
		//When yaw stick is back in the center position start the motors (step 2).
		if (start == 1 && GPIO_PulseWidth.Throttle < 1050 && GPIO_PulseWidth.Yaw > 1450) {
			start = 2;
			
			Angle_Pitch = angle_pitch_acc;                                                 //Set the gyro pitch angle equal to the accelerometer pitch angle when the quadcopter is started.
			Angle_Roll = angle_roll_acc;                                                   //Set the gyro roll angle equal to the accelerometer roll angle when the quadcopter is started.

			//Reset the PID controllers for a bumpless start.
			pid_i_mem_roll = 0;
			pid_last_roll_d_error = 0;
			pid_i_mem_pitch = 0;
			pid_last_pitch_d_error = 0;
			pid_i_mem_yaw = 0;
			pid_last_yaw_d_error = 0;
		}
		//Stopping the motors: throttle low and yaw right.
		if (start == 2 && GPIO_PulseWidth.Throttle < 1050 && GPIO_PulseWidth.Yaw > 1950) {
			start = 0;                                                             //Turn on the green led.
		}
			
		pid_roll_setpoint = 0;
		//We need a little dead band of 16us for better results.
		if (GPIO_PulseWidth.Roll > 1517)pid_roll_setpoint = GPIO_PulseWidth.Roll - 1517;
		else if (GPIO_PulseWidth.Roll < 1483)pid_roll_setpoint = GPIO_PulseWidth.Roll - 1483;

		pid_roll_setpoint -= roll_level_adjust;                                          //Subtract the angle correction from the standardized receiver roll input value.
		pid_roll_setpoint /= 6.0;                                                        //Divide the setpoint for the PID roll controller by 3 to get angles in degrees.


		//The PID set point in degrees per second is determined by the pitch receiver input.
		//In the case of deviding by 3 the max pitch rate is aprox 164 degrees per second ( (500-8)/3 = 164d/s ).
		pid_pitch_setpoint = 0;
		//We need a little dead band of 16us for better results.
		if (GPIO_PulseWidth.Pitch > 1517)pid_pitch_setpoint = GPIO_PulseWidth.Pitch - 1517;
		else if (GPIO_PulseWidth.Pitch < 1483)pid_pitch_setpoint = GPIO_PulseWidth.Pitch - 1483;

		pid_pitch_setpoint -= pitch_level_adjust;                                        //Subtract the angle correction from the standardized receiver pitch input value.
		pid_pitch_setpoint /= 6.0;                                                       //Divide the setpoint for the PID pitch controller by 3 to get angles in degrees.

		//The PID set point in degrees per second is determined by the yaw receiver input.
		//In the case of deviding by 3 the max yaw rate is aprox 164 degrees per second ( (500-8)/3 = 164d/s ).
		pid_yaw_setpoint = 0;
		/*We need a little dead band of 16us for better results.*/
		if (GPIO_PulseWidth.Throttle > 1050) { //Do not yaw when turning off the motors.
			if (GPIO_PulseWidth.Yaw > 1517)pid_yaw_setpoint = (GPIO_PulseWidth.Yaw - 1517) / 6.0;
			else if (GPIO_PulseWidth.Yaw < 1483)pid_yaw_setpoint = (GPIO_PulseWidth.Yaw - 1483) / 6.0;
		}

		/*Roll calculations*/
		pid_error_temp = gyro_roll_input - pid_roll_setpoint;
		pid_i_mem_roll += pid_i_gain_roll * pid_error_temp;
		if(pid_i_mem_roll > pid_max_roll)pid_i_mem_roll = pid_max_roll;
		else if(pid_i_mem_roll < pid_max_roll * -1)pid_i_mem_roll = pid_max_roll * -1;

		pid_output_roll = pid_p_gain_roll * pid_error_temp + pid_i_mem_roll + pid_d_gain_roll * (pid_error_temp - pid_last_roll_d_error);
		if(pid_output_roll > pid_max_roll)pid_output_roll = pid_max_roll;
		else if(pid_output_roll < pid_max_roll * -1)pid_output_roll = pid_max_roll * -1;

		pid_last_roll_d_error = pid_error_temp;

		/*Pitch calculations*/
		pid_error_temp = gyro_pitch_input - pid_pitch_setpoint;
		pid_i_mem_pitch += pid_i_gain_pitch * pid_error_temp;
		if(pid_i_mem_pitch > pid_max_pitch)pid_i_mem_pitch = pid_max_pitch;
		else if(pid_i_mem_pitch < pid_max_pitch * -1)pid_i_mem_pitch = pid_max_pitch * -1;

		pid_output_pitch = pid_p_gain_pitch * pid_error_temp + pid_i_mem_pitch + pid_d_gain_pitch * (pid_error_temp - pid_last_pitch_d_error);
		if(pid_output_pitch > pid_max_pitch)pid_output_pitch = pid_max_pitch;
		else if(pid_output_pitch < pid_max_pitch * -1)pid_output_pitch = pid_max_pitch * -1;

		pid_last_pitch_d_error = pid_error_temp;

		/*Yaw calculations*/
		pid_error_temp = gyro_yaw_input - pid_yaw_setpoint;
		pid_i_mem_yaw += pid_i_gain_yaw * pid_error_temp;
		if(pid_i_mem_yaw > pid_max_yaw)pid_i_mem_yaw = pid_max_yaw;
		else if(pid_i_mem_yaw < pid_max_yaw * -1)pid_i_mem_yaw = pid_max_yaw * -1;

		pid_output_yaw = pid_p_gain_yaw * pid_error_temp + pid_i_mem_yaw + pid_d_gain_yaw * (pid_error_temp - pid_last_yaw_d_error);
		if(pid_output_yaw > pid_max_yaw)pid_output_yaw = pid_max_yaw;
		else if(pid_output_yaw < pid_max_yaw * -1)pid_output_yaw = pid_max_yaw * -1;

		pid_last_yaw_d_error = pid_error_temp;
		
		if (GPIO_PulseWidth.Aux1 >= 1250)	/* Urgent stop */
			start = 0;
			
		throttle = GPIO_PulseWidth.Throttle;
		Battery = Battery * 0.92 + GPIO_ReadAnalog(ADC1) * 0.08 * 36.3 / 4096.0;
		if (start == 2) {
			if (throttle > 1800) throttle = 1800;                                          //We need some room to keep full control at full throttle.
			RearLeft = throttle - (ui16)pid_output_pitch + (ui16)pid_output_roll - (ui16)pid_output_yaw;        //Calculate the pulse for esc 1 (front-right - CCW).
			RearRight = throttle + (ui16)pid_output_pitch + (ui16)pid_output_roll + (ui16)pid_output_yaw;        //Calculate the pulse for esc 2 (rear-right - CW).
			FrontRight = throttle + (ui16)pid_output_pitch - (ui16)pid_output_roll - (ui16)pid_output_yaw;        //Calculate the pulse for esc 3 (rear-left - CCW).
			FrontLeft = throttle - (ui16)pid_output_pitch - (ui16)pid_output_roll + (ui16)pid_output_yaw;        //Calculate the pulse for esc 4 (front-left - CW).

			if (FrontRight < MIN_THROTTLE) FrontRight = MIN_THROTTLE;                                                //Keep the motors running.
			if (RearRight < MIN_THROTTLE) RearRight = MIN_THROTTLE;                                                //Keep the motors running.
			if (RearLeft < MIN_THROTTLE) RearLeft = MIN_THROTTLE;                                                //Keep the motors running.
			if (FrontLeft < MIN_THROTTLE) FrontLeft = MIN_THROTTLE;                                                //Keep the motors running.

			if (FrontRight > 2000)FrontRight = 2000;                                                 //Limit the esc-1 pulse to 2000us.
			if (RearRight > 2000)RearRight = 2000;                                                 //Limit the esc-2 pulse to 2000us.
			if (RearLeft > 2000)RearLeft = 2000;                                                 //Limit the esc-3 pulse to 2000us.
			if (FrontLeft > 2000)FrontLeft = 2000;                                                 //Limit the esc-4 pulse to 2000us.
		}
		else {
			FrontRight = 1000;                                                                  //If start is not 2 keep a 1000us pulse for ess-1.
			RearRight = 1000;                                                                  //If start is not 2 keep a 1000us pulse for ess-2.
			RearLeft = 1000;                                                                  //If start is not 2 keep a 1000us pulse for ess-3.
			FrontLeft = 1000;                                                                  //If start is not 2 keep a 1000us pulse for ess-4.
		}
		TIM4->CCR1 = FrontRight;
		TIM4->CCR2 = FrontLeft;
		TIM4->CCR3 = RearLeft;
		TIM4->CCR4 = RearRight;
		TIM4->CNT = 5000;
#if (UART_ENABLE == ON)
		UART1_sendStr("Pitch:");
		UART1_sendNum((int)Angle_Pitch);
		UART1_sendStr("\t");
		UART1_sendStr("Roll:");
		UART1_sendNum((int)Angle_Roll);
		UART1_sendStr("\n");
#endif
		CheckTimeOut();
		/* warning */
		if (Battery < 10.5 && ErrorTimeout == FALSE) {
			Error = PINLOW;
		}
		if (Battery >= 10.5 && ErrorTimeout == TRUE) {
			Error = TIMEOUT;
		}
		if (Battery < 10.5 && ErrorTimeout == TRUE) {
			Error = TIMEOUT_PINLOW;
		}
		while (micros() - loop_timer < 4000);
		loop_timer = micros();
	}
}

void System_Init(void) {
	int i=0;
	GPIO_SetPWM(IO_B6);
	GPIO_SetPWM(IO_B7);
	GPIO_SetPWM(IO_B8);
	GPIO_SetPWM(IO_B9);
	GPIO_B6_PWM(1000);
	GPIO_B7_PWM(1000);
	GPIO_B8_PWM(1000);
	GPIO_B9_PWM(1000);
	CLOCK_SystickInit();
	GPIO_SetPWMMeasurement();
	GPIO_SetOutPut(IO_C13, General_Push_Pull);
	GPIO_SetOutPut(IO_B14, General_Push_Pull);
	GPIO_SetOutPut(IO_B15, General_Push_Pull);
	GPIO_PINHigh(IO_C13);
	GPIO_PINHigh(IO_B14);
	GPIO_PINHigh(IO_B15);
	GPIO_SetAnalog(ADC_A0);
	Battery =  GPIO_ReadAnalog(ADC1) * 36.3 / 4096.0;
	for (;i<=10;i++)
	{
		GPIO_PINToggle(WARNING_LED);
		delay(100*MS);
	}
#if (UART_ENABLE == ON)
	UART1_Init(UART_BAUDRATE_115200);
#endif
	I2C2_Init(I2C_SPEED_400);
	MPU6050_Init();
	
}

void CheckTimeOut(void) {
	if (micros() - loop_timer > 4000)
		ErrorTimeout = TRUE;
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

bool TX_Unavailable(void) {
	if ( !GPIO_PulseWidth.Throttle && !GPIO_PulseWidth.Aux1 
		&& !GPIO_PulseWidth.Yaw && !GPIO_PulseWidth.Pitch && !GPIO_PulseWidth.Roll)
		return TRUE;
	else
		return FALSE;
}

void ESC_Clibration(void) {
	while (GPIO_PulseWidth.Throttle > 1000) {
		GPIO_B6_PWM(GPIO_PulseWidth.Throttle);
		GPIO_B7_PWM(GPIO_PulseWidth.Throttle);
		GPIO_B8_PWM(GPIO_PulseWidth.Throttle);
		GPIO_B9_PWM(GPIO_PulseWidth.Throttle);
	}
}
