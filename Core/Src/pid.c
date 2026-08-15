#include "pid.h"
#include <math.h>

//pid_1dof_t rpm_pid_M3508[4];

static void limit(float *a, float max_value)
{
    if (*a > max_value)
        *a = max_value;
    if (*a < -max_value)
        *a = -max_value;
}

int mapValue(int x, int in_min, int in_max, int out_min, int out_max) {
    return ((x - in_min) * (out_max - out_min) / (in_max - in_min)) + out_min;
}

static void pid_reset(pid_1dof_t  *pid, float kp, float ki, float kd)
{
  pid->p = kp;
  pid->i = ki;
  pid->d = kd;

  pid->pout = 0;
  pid->iout = 0;
  pid->dout = 0;
  pid->out = 0;

  pid->prev_derivative = 0.0f; // Reset derivative filter state
}

static void pid_param_init(
  pid_1dof_t *pid,
  uint32_t mode,
  float maxout,
  float integral_limit,
  float kp,
  float ki,
  float kd,
  float n,
  float dt,
  float deadband)
{
  // Validate parameters
  if (kp < 0 || ki < 0 || kd < 0)
  {
      return; // Invalid parameters, return without modifying PID struct
  }

  pid->integral_limit = integral_limit;
  pid->max_out = maxout;
  pid->pid_mode = mode;

  pid->p = kp;
  pid->i = ki;
  pid->d = kd;

  /*initialize the output to 0 at first*/
  pid->pout = 0;
  pid->iout = 0;
  pid->dout = 0;
  pid->out = 0;

  // Initialize derivative filter state
  pid->prev_derivative = 0.0f;
  pid->output_deadband = deadband;

  pid->EMA_filter_coefficient = (n * dt) / (1.0f + (n * dt));
}

void PID_1DOF_Init(
	pid_1dof_t * pid,
	float kp,
	float ki,
	float kd,
	float n,
	float integral_limit,
	float maxout,
	uint32_t mode,
	float dt,
	float deadband
	)
{
    pid->f_param_init = pid_param_init;
    pid->f_pid_reset = pid_reset;

    pid->f_param_init(pid, mode, maxout, integral_limit, kp, ki, kd, n, dt, deadband);
    pid->f_pid_reset(pid, kp, ki, kd);
}

// PID controller equation function
void PID_1DOF_calc(pid_1dof_t *pid, float actual, float setpoint, float dt)
{
	pid->actual = actual;
	pid->setpoint = setpoint;

	// 1. calculate the current error
	pid->err[NOW] = setpoint - actual;
	if(fabs(pid->err[NOW]) < pid->output_deadband)
		pid->err[NOW] = 0.00f;

	// 2. Proportional Term
	pid->pout = pid->p * pid->err[NOW];

	// 3. Integral Term (Rectangular Integration)
	// Formula: I_accum += Ki * Error * dt
//	float i_term_inc = pid->i * pid->err[NOW] * DT;

	// Trapezoidal integration (optional, slightly more accurate):
	float i_term_inc = pid->i * (pid->err[NOW] + pid->err[LAST]) * 0.5f * dt;

	pid->iout += i_term_inc;

	// Anti-Windup: Clamp the Integral term individually
	limit(&(pid->iout), pid->integral_limit);

	// 4. Derivative term (with low-pass filtering)
	// Calculate raw slope: (Current Err - Prev Err) / Time
	float derivative_raw = (pid->err[NOW] - pid->err[LAST]) / dt;

	// Apply Low Pass Filter (Exponential Moving Average)
	// Formula: Filtered_D = (Alpha * Raw) + ((1 - Alpha) * Prev_Filtered)
	float derivative_filtered = (pid->EMA_filter_coefficient * derivative_raw) +
								((1.0f - pid->EMA_filter_coefficient) * pid->prev_derivative);

	// Store for next loop
	pid->prev_derivative = derivative_filtered;

	// Calculate final D output
	pid->dout = pid->d * derivative_filtered;

	// 5. Total Output
	pid->out = pid->pout + pid->iout + pid->dout;

	// 6. Final Output Limit
	limit(&(pid->out), pid->max_out);

	// 7. Update the error history
	pid->err[LAST] = pid->err[NOW];
}

void PID_1DOF_Reset(pid_1dof_t *pid)
{
    pid->iout = 0.0f;
    pid->out = 0.0f;
    pid->err[NOW] = 0.0f; 	// Current error
    pid->err[LAST] = 0.0f; 	// Previous error
    pid->prev_derivative = 0.0f;
}
