/*
 * this is the code for M2006 bldc
 * */

#include "motor.h"

#define ENCODER_ANGLE_RATIO    (8192.0f/360.0f)

int max_current_M2006 = 10000;
int min_current_M2006 = -10000;

int max_current_M3508 = 16384;
int min_current_M3508 = -16384;

float gear_ratio_M2006 = 36.0f;
float gear_ratio_M3508 = 3591.0f / 187.0f; // Approx 19.2032f

/* Output RPM is the RPM at the motor shaft */
uint16_t max_output_rpm_M2006_6S = 500;
uint16_t max_output_rpm_M2006_4S = 300;

uint16_t max_output_rpm_M3508 = 400;

/*this function is used to calculate out the rpm of the motor shaft*/
int RPM_GearRatio (motor_measure_t *motors, int i, Motor_Speed_parameters Motor_Speed)
{
	/* Gear ratio = Driven/driving */
	/* so this function is to find the rpm of the driving motor shaft*/
	float shaft_rpm = (float)(motors[i].speed_rpm) / Motor_Speed.GearRatio;

	// Simple rounding for both positive and negative speeds
	if (shaft_rpm > 0.0f) {
		return (int)(shaft_rpm + 0.5f);
	} else {
		return (int)(shaft_rpm - 0.5f);
	}
}

// int move = 60;
// int rotate = 30;
// int lock = 30;
// int bno = 30;

void motor_struct_init (Motor_Speed_parameters *Motor_Speed, int move_per, int rotate_per, int Lock_per, int BNO_per)
{
	Motor_Speed->Max_Wheel_Speed = max_output_rpm_M2006_4S; // the maximum wheel(motor shaft) RPM for BLDC
	Motor_Speed->GearRatio = gear_ratio_M2006;
	Motor_Speed->Max_Current = max_current_M2006; // -10000 to 10000 (-10A to 10A)

	Motor_Speed->Max_motor_Speed = Motor_Speed->Max_Wheel_Speed * Motor_Speed->GearRatio; // maximum RPM depends on the current robot voltage
	Motor_Speed->Move_Speed = Motor_Speed->Max_Wheel_Speed * (move_per/100.0f);
	Motor_Speed->Max_Wheel_Current = Motor_Speed->Max_Current * (move_per/100.0f);
	Motor_Speed->Rotate_Speed = Motor_Speed->Max_Wheel_Speed * (rotate_per/100.0f);
	// Motor_Speed->Max_Locking_Speed = Motor_Speed->Max_Wheel_Speed * (Lock_per/100.0f);
	// Motor_Speed->Max_BNO_Speed = Motor_Speed->Max_Wheel_Speed * (BNO_per/100.0f);
}

// initialization function for Robomaster motor
void RM_motor_struct_init (Motor_Speed_parameters *Motor_Speed, int move_per, int rotate_per, uint8_t index)
{
	if(index == RM_M2006)
	{
		Motor_Speed->Max_Wheel_Speed = max_output_rpm_M2006_4S; // the maximum wheel(motor shaft) RPM for BLDC
		Motor_Speed->GearRatio = gear_ratio_M2006;
		Motor_Speed->Max_Current = max_current_M2006; // -10000 to 10000 (-10A to 10A)

		Motor_Speed->Max_motor_Speed = Motor_Speed->Max_Wheel_Speed * Motor_Speed->GearRatio; // maximum RPM depends on the current robot voltage
		Motor_Speed->Move_Speed = Motor_Speed->Max_Wheel_Speed * (move_per/100.0f);
		Motor_Speed->Max_Wheel_Current = Motor_Speed->Max_Current * (move_per/100.0f);
		Motor_Speed->Rotate_Speed = Motor_Speed->Max_Wheel_Speed * (rotate_per/100.0f);
	}
	else if(index == RM_M3508)
	{
		Motor_Speed->Max_Wheel_Speed = max_output_rpm_M3508; // the maximum wheel(motor shaft) RPM for BLDC
		Motor_Speed->GearRatio = gear_ratio_M3508;
		Motor_Speed->Max_Current = max_current_M3508; // -16384 to 16384 (-20A to 20A)

		Motor_Speed->Max_motor_Speed = Motor_Speed->Max_Wheel_Speed * Motor_Speed->GearRatio; // maximum RPM depends on the current robot voltage
		Motor_Speed->Move_Speed = Motor_Speed->Max_Wheel_Speed * (move_per/100.0f);
		Motor_Speed->Max_Wheel_Current = Motor_Speed->Max_Current * (move_per/100.0f);
		Motor_Speed->Rotate_Speed = Motor_Speed->Max_Wheel_Speed * (rotate_per/100.0f);
	}

}
