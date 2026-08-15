/*
 * pid_2dof.c
 *
 *  Created on: Mar 5, 2026
 *      Author: Acer
 */

#include "pid_2dof.h"


static void limit(float *a, float max_value)
{
    if (*a > max_value)
        *a = max_value;
    if (*a < -max_value)
        *a = -max_value;
}

void PID_2DOF_Init(pid_2dof_t *pid, float p, float i, float d, float n, float b, float c, float integral_limit, float max_out)
{
	// 1. Load Tuning Parameters from MATLAB PID Tuner [cite: 748, 749]
	pid->p = p;	// Proportional (P)
	pid->i = i; // Integral (I)
	pid->d = d; // Derivative (D)
	pid->n = n; // Filter coefficient (N)
	pid->b = b; // Setpoint weight (b)
	pid->c = c; // Setpoint weight (c)

	// 2. Clear State Variables (Prevents undefined behavior on boot)
	pid->iout = 0.0f;
	pid->derivative_filter_state = 0.0f;
	pid->pout = 0.0f;
	pid->dout = 0.0f;
	pid->out = 0.0f;

	// 3. Configure Safety Limits (e.g., max PWM, max voltage, or max current)
	pid->integral_limit = integral_limit;
	pid->max_out = max_out;
}

// Note: You must pass in the loop time 'dt' (which MATLAB calls Ts)
void PID_2DOF_calc(pid_2dof_t *pid, float actual, float setpoint, float dt)
{
    // 1. Calculate the three independent 2DOF errors
    float err_P = (pid->b * setpoint) - actual;
    float err_I = setpoint - actual;              // Integral always uses pure error
    float err_D = (pid->c * setpoint) - actual;

    // 2. Proportional Term
    pid->pout = pid->p * err_P;

    // 3. Integral Term (Rectangular Integration to match the 1/(z-1) MATLAB setting)
    // Formula: I_out += I * dt * pure_error
    pid->iout += pid->i * dt * err_I;

    // Anti-Windup: Clamp the Integral term individually
    limit(&(pid->iout), pid->integral_limit);

    // 4. Derivative term with MATLAB's N-filter
    // Mathematically equivalent to: D_out = N * (D * err_D - Filter_State)
    pid->dout = pid->n * ((pid->d * err_D) - pid->derivative_filter_state);

    // Update the filter state for the next loop (integrating the D output)
    pid->derivative_filter_state += pid->dout * dt;

    // 5. Total Output
    pid->out = pid->pout + pid->iout + pid->dout;

    // 6. Final Output Limit
    limit(&(pid->out), pid->max_out);
}

void RM_mapping_pid_out(pid_2dof_t *pid, uint8_t *txData, uint8_t CAN_OK)
{
	if(CAN_OK == 0) // HAL_OK = 0
	{
		int16_t motor_current_mapped[4];
		for(int i = 0; i < 4; i++)
		{
			motor_current_mapped[i] = (int16_t)pid[i].out;
		}

		txData[0] = (uint8_t)(motor_current_mapped[0] >> 8);
		txData[1] = (uint8_t)(motor_current_mapped[0] & 0xFF);
		txData[2] = (uint8_t)(motor_current_mapped[1] >> 8);
		txData[3] = (uint8_t)(motor_current_mapped[1] & 0xFF);
		txData[4] = (uint8_t)(motor_current_mapped[2] >> 8);
		txData[5] = (uint8_t)(motor_current_mapped[2] & 0xFF);
		txData[6] = (uint8_t)(motor_current_mapped[3] >> 8);
		txData[7] = (uint8_t)(motor_current_mapped[3] & 0xFF);
	}
	else
	{
		for (int i=0; i<8; i++)
		{
			txData[i]=0;
		}
	}
}
