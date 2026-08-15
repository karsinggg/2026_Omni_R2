/*
 * pid_2dof.h
 *
 *  Created on: Mar 5, 2026
 *      Author: Acer
 */

#ifndef INC_PID_2DOF_H_
#define INC_PID_2DOF_H_

#include "main.h"

typedef struct {
    // 1. Tuning Parameters (Map these directly from MATLAB's output)
    float p;
    float i;
    float d;
    float n; // Filter coefficient (N)
    float b; // Setpoint weight for Proportional
    float c; // Setpoint weight for Derivative

    // 2. Limits
    float integral_limit;
    float max_out;

    // 3. State variables
    float iout;
    float derivative_filter_state; // Replaces prev_derivative

    // 4. Outputs (Useful for live debugging via STM-Studio or Serial)
    float pout;
    float dout;
    float out;
} pid_2dof_t;

void PID_2DOF_Init(pid_2dof_t *pid, float p, float i, float d, float n, float b, float c, float integral_limit, float max_out);
void PID_2DOF_calc(pid_2dof_t *pid, float actual, float setpoint, float dt);
void RM_mapping_pid_out(pid_2dof_t *pid, uint8_t *txData, uint8_t CAN_OK);

#endif /* INC_PID_2DOF_H_ */
