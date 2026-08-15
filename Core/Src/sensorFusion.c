#include "main.h"
#include <math.h>
#include "sensorFusion.h"
#include <stdbool.h>

#define PI 							3.1415926535f
#define LASER_Y_FIELD_OFFSET		12500 	// MM
#define LASER_Y_OFFSET 				213 	// MM from the centre of the robot
#define LASER_X_OFFSET 				23 		// MM from the centre of the robot
#define ENCODER_Y_OFFSET			210 	// MM
#define SPEARHEAD_RACK_OFFSET		125		// MM

// --- Tuning Parameters ---
// ALPHA: How much we trust the laser per update.
// 0.05 means 95% Encoder, 5% Laser. (Tune this! Lower = smoother, Higher = faster correction)
#define FUSION_ALPHA_X 		0.25f
#define FUSION_ALPHA_Y 		0.25f

// GATING THRESHOLD: The maximum physically possible error between the encoder and laser.
// If the difference is larger than this, assume the laser hit an obstacle and reject it.
#define LASER_GATE_MM 			50.0f // 50mm -> 5cm
#define FUSION_SNAP_MM 			10.0f
#define ENCODER_PPR				1000
#define TICKS_PER_REV 			(ENCODER_PPR * 4.0f) // for Encoder T1 and T2 Mode
#define MM_PER_TICK 			((ROTARY_ENCODER_DIAMETER_MM * PI) / TICKS_PER_REV)

#define ENC_Y_OFFSET_FROM_X_AXIS_MM 	15.25f // Distance from robot center to X-encoder
#define ENC_X_OFFSET_FROM_Y_AXIS_MM 	190.25f // Distance from robot center to Y-encoder

extern volatile uint32_t encoder_x_ticks;
extern volatile uint32_t encoder_y_ticks;
extern int32_t laser_x_distance;
extern int32_t laser_y_distance;
extern volatile bool new_laser_x_available;
extern volatile bool new_laser_y_available;

// Keep track of ticks to calculate the delta
static uint32_t prev_ticks_X = 0;
static uint32_t prev_ticks_Y = 0;

bool is_laser_x_blocked = false;
bool is_laser_y_blocked = false;

void Sensor_Fusion_Init(float* fused_robot_x, float* fused_robot_y)
{
    // Seed the filter with the robot's starting position on the field
    *fused_robot_x = (float)(laser_x_distance + LASER_X_OFFSET + SPEARHEAD_RACK_OFFSET);
    *fused_robot_y = (float)(laser_y_distance + LASER_Y_OFFSET);

    prev_ticks_X = encoder_x_ticks;
    prev_ticks_Y = encoder_y_ticks;
}

/* --------------- Modified (with Rotation-accounted) --------------- */
void Sensor_Fusion_Update(float* fused_robot_x, float* fused_robot_y, float current_yaw_deg)
{
	static float prev_yaw_deg = 0.0f; // Remember the yaw from 10ms ago

    uint32_t current_ticks_X = encoder_x_ticks;
    uint32_t current_ticks_Y = encoder_y_ticks;

    // Omni-robot -> incremental encoder is reverse direction
    int32_t delta_ticks_X = (int32_t)(prev_ticks_X - current_ticks_X);
    int32_t delta_ticks_Y = (int32_t)(prev_ticks_Y - current_ticks_Y);

    prev_ticks_X = current_ticks_X;
    prev_ticks_Y = current_ticks_Y;

    // Calculate Local Distances (Respective to the encoder)
	float raw_dX_local = (float)delta_ticks_X * MM_PER_TICK;
	float raw_dY_local = (float)delta_ticks_Y * MM_PER_TICK;

	// Calculate how much we rotated in the last 10ms
	float delta_yaw_deg = current_yaw_deg - prev_yaw_deg;

	// Handle IMU wrap-around (e.g., spinning past the 180 to -180 boundary)
	while (delta_yaw_deg > 180.0f)  delta_yaw_deg -= 360.0f;
	while (delta_yaw_deg < -180.0f) delta_yaw_deg += 360.0f;

	prev_yaw_deg = current_yaw_deg;
	float delta_yaw_rad = delta_yaw_deg * (M_PI / 180.0f);

	// DECOUPLE THE ROTATION (s = r * theta)
	// NOTE: The +/- signs here depend entirely on which side of the robot the wheels are mounted!
	// You will need to test spinning the robot in place and flip the +/- signs if the numbers drift worse.
	float clean_dX_local = raw_dX_local + (ENC_X_OFFSET_FROM_Y_AXIS_MM * delta_yaw_rad);
	float clean_dY_local = raw_dY_local + (ENC_Y_OFFSET_FROM_X_AXIS_MM * delta_yaw_rad);

	// Convert Yaw to Radians for the math library
	float yaw_rad = current_yaw_deg * (M_PI / 180.0f);

    // Rotate Local deltas to Global deltas using the Rotation Matrix
	float dX_global = (clean_dX_local * cosf(yaw_rad)) - (clean_dY_local * sinf(yaw_rad));
	float dY_global = (clean_dX_local * sinf(yaw_rad)) + (clean_dY_local * cosf(yaw_rad));

	// Add the globally corrected movements to the robot's position
	*fused_robot_x += dX_global;
	*fused_robot_y += dY_global;

    /* ----- LASER Fusion ----- */
    if(fabs(current_yaw_deg) < 10.0f) // Only trust the lasers if the robot is mostly facing the correct direction
    {
		// --- X AXIS FUSION ---
		if (new_laser_x_available)
		{
			float raw_laser_x = (float)(laser_x_distance + LASER_X_OFFSET);
			new_laser_x_available = false; // Clear flag so we don't use it again until fresh

			float current_laser_x = raw_laser_x * cosf(yaw_rad);
			float absolute_error = fabsf(current_laser_x - *fused_robot_x);

			// Sanity Gating: Check the absolute difference
			if (absolute_error < LASER_GATE_MM)
			{
				// The path is clear! Automatically unblock.
				is_laser_x_blocked = false;

				if (absolute_error < FUSION_SNAP_MM) *fused_robot_x = current_laser_x;
				else
					*fused_robot_x = (*fused_robot_x * (1.0f - FUSION_ALPHA_X)) + (current_laser_x * FUSION_ALPHA_X);
			}
			else
			{
				// OBSTACLE DETECTED! Error is too large. Ignore laser data.
				is_laser_x_blocked = true;
			}
		}

		// --- Y AXIS FUSION ---
		if (new_laser_y_available)
		{
			float raw_laser_y = (float)(laser_y_distance + LASER_Y_OFFSET);
			new_laser_y_available = false;

			float current_laser_y = raw_laser_y * cosf(yaw_rad);
			float absolute_error = fabsf(current_laser_y - *fused_robot_y);

			// Fixed: Added the missing Sanity Gate check for the Y-Axis!
			if (absolute_error < LASER_GATE_MM)
			{
				is_laser_y_blocked = false;

				if (absolute_error < FUSION_SNAP_MM) *fused_robot_y = current_laser_y;
				else
					*fused_robot_y = (*fused_robot_y * (1.0f - FUSION_ALPHA_Y)) + (current_laser_y * FUSION_ALPHA_Y);
			}
			else
			{
				is_laser_y_blocked = true;
			}
		}
    }
}

/* --------------- Previous Sensor Fusion Source Code --------------- */
//void Sensor_Fusion_Update(float* fused_robot_x, float* fused_robot_y)
//{
//    uint32_t current_ticks_X = encoder_x_ticks;
//    uint32_t current_ticks_Y = encoder_y_ticks;
//
//    // Omni-robot -> incremental encoder is reverse direction
//    int32_t delta_ticks_X = (int32_t)(prev_ticks_X - current_ticks_X);
//    int32_t delta_ticks_Y = (int32_t)(prev_ticks_Y - current_ticks_Y);
//
//    prev_ticks_X = current_ticks_X;
//    prev_ticks_Y = current_ticks_Y;
//
//    // Add the fast encoder movements to our fused position
//    *fused_robot_x += (float)delta_ticks_X * MM_PER_TICK;
//    *fused_robot_y += (float)delta_ticks_Y * MM_PER_TICK;
//
//    // --- X AXIS FUSION ---
//    if (new_laser_x_available)
//    {
//        float current_laser_x = (float)(laser_x_distance + LASER_X_OFFSET);
//        new_laser_x_available = false; // Clear flag so we don't use it again until fresh
//
//        float absolute_error = fabsf(current_laser_x - *fused_robot_x);
//
//        // Sanity Gating: Check the absolute difference
//        if (absolute_error < LASER_GATE_MM)
//		{
//			// The path is clear! Automatically unblock.
//			is_laser_x_blocked = false;
//
//			if (absolute_error < FUSION_SNAP_MM)
//			{
//				*fused_robot_x = current_laser_x;
//			}
//			else
//			{
//				*fused_robot_x = (*fused_robot_x * (1.0f - FUSION_ALPHA_X)) + (current_laser_x * FUSION_ALPHA_X);
//			}
//		}
//		else
//		{
//			// OBSTACLE DETECTED! Error is too large. Ignore laser data.
//			is_laser_x_blocked = true;
//		}
//    }
//
//    // --- Y AXIS FUSION ---
//    if (new_laser_y_available)
//    {
//    	float current_laser_y = (float)(laser_y_distance + LASER_Y_OFFSET);
//		new_laser_y_available = false;
//
//		float absolute_error = fabsf(current_laser_y - *fused_robot_y);
//
//		// Fixed: Added the missing Sanity Gate check for the Y-Axis!
//		if (absolute_error < LASER_GATE_MM)
//		{
//			is_laser_y_blocked = false;
//
//			if (absolute_error < FUSION_SNAP_MM)
//			{
//				*fused_robot_y = current_laser_y;
//			}
//			else
//			{
//				*fused_robot_y = (*fused_robot_y * (1.0f - FUSION_ALPHA_Y)) + (current_laser_y * FUSION_ALPHA_Y);
//			}
//		}
//		else
//		{
//			is_laser_y_blocked = true;
//		}
//    }
//}
