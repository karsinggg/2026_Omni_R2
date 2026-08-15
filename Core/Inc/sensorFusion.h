/*
 * sensorFusion.h
 *
 *  Created on: Apr 12, 2026
 *      Author: Acer
 */

#ifndef INC_SENSORFUSION_H_
#define INC_SENSORFUSION_H_

#define ROTARY_ENCODER_DIAMETER_MM		50.80f			// 2 inch

void Sensor_Fusion_Init(float* fused_robot_x, float* fused_robot_y);
//void Sensor_Fusion_Update(float* fused_robot_x, float* fused_robot_y);
void Sensor_Fusion_Update(float* fused_robot_x, float* fused_robot_y, float current_yaw_deg);

#endif /* INC_SENSORFUSION_H_ */
