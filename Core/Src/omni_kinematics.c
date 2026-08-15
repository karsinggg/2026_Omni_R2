/*
 * omni_kinematics.c
 *
 *  Created on: May 6, 2026
 *      Author: Acer
 */
#include "omni_kinematics.h"
#include <math.h>

#define GEAR_RATIO 						19 	// the inner rpm will be 19 times greater
#define MAX_OUTPUT_SHAFT_SPEED 			350 // the maximum rpm of this motor will be 350 to 400 rpm (motor shaft)
#define MAX_INNER_SPEED 				(MAX_OUTPUT_SHAFT_SPEED * GEAR_RATIO)
#define HALF_INNER_SPEED				(0.5f * MAX_INNER_SPEED)
#define MAX_JOYSTICK_INPUT				128.0f
#define MAX_THROTTLE_INPUT				256.0f

void omni_algorithm(float v_tx, float v_ty, float w, float target_speeds[], bool speed_up)
{
	float inner_speed_selected;
	if(speed_up)
		inner_speed_selected = MAX_INNER_SPEED;
	else
		inner_speed_selected = HALF_INNER_SPEED;

	float max_speed = 0.0f;
	float scale_factor;

	// 1. Normalize inputs to a -1.0 to 1.0 range
	float vx_norm   = v_tx / MAX_JOYSTICK_INPUT;
	float vy_norm   = v_ty / MAX_JOYSTICK_INPUT;
	float rot_norm  = w    / MAX_THROTTLE_INPUT;

	target_speeds[0] = ( vx_norm + vy_norm + rot_norm) * inner_speed_selected;
	target_speeds[1] = ( vx_norm - vy_norm + rot_norm) * inner_speed_selected;
	target_speeds[2] = (-vx_norm + vy_norm + rot_norm) * inner_speed_selected;
	target_speeds[3] = (-vx_norm - vy_norm + rot_norm) * inner_speed_selected;

	for (int i = 0; i < 4; i++)
	{
		if (fabsf(target_speeds[i]) > max_speed) {
			max_speed = fabsf(target_speeds[i]);
		}
	}

	if (max_speed > inner_speed_selected) {
		scale_factor = inner_speed_selected / max_speed;
		for (size_t i = 0; i < 4; i++) {
			target_speeds[i] *= scale_factor;
		}
	}
}
