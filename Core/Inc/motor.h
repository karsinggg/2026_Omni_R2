
#ifndef INC_MOTOR_H_
#define INC_MOTOR_H_

#include <stdint.h>
#include "fdcan.h"

typedef struct {
    uint16_t last_ecd;
    uint16_t ecd;
    int16_t ecd_change;
    float total_ecd;
    float total_angle;
    int16_t speed_rpm;
    int16_t given_current;
    int32_t round_cnt;
    uint16_t offset_ecd;
} motor_measure_t;

typedef struct {
	float Max_motor_Speed;
    float GearRatio;
    float Max_Current;
    float Max_Wheel_Current;
    float Max_Wheel_Speed;
    float Max_BNO_Speed;
    float Move_Speed;
    float Rotate_Speed;
    float Max_Locking_Speed;
} Motor_Speed_parameters;

enum
{
	RM_M2006,
	RM_M3508
};

//extern motor_measure_t motors[2];

void motor_struct_init (Motor_Speed_parameters *Motor_Speed, int move_per, int rotate_per, int Lock_per, int BNO_per);
void RM_motor_struct_init (Motor_Speed_parameters *Motor_Speed, int move_per, int rotate_per, uint8_t index);

#endif /* INC_MOTOR_H_ */
