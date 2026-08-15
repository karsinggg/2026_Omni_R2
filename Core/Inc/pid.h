#ifndef INC_PID_H_
#define INC_PID_H_

#include <stdint.h>
#include "main.h"

enum
{
  LLAST = 0,
  LAST,
  NOW,
  POSITION_PID,
  DELTA_PID,
};

typedef struct pid_1dof_t
{
  float p;
  float i;
  float d;

  float setpoint;
  float actual; // actual value feedback from the sensor
  float err[3];

  float pout;
  float iout;
  float dout;
  float out;

  float input_max_err;    //input max err;
  float output_deadband;  //output deadband;
  float prev_derivative;
  float EMA_filter_coefficient;

  uint32_t pid_mode;
  float max_out;
  float integral_limit;

  void (*f_param_init)(  struct pid_1dof_t *pid,
		  	  	  	  	 uint32_t pid_mode,
						 float max_output,
						 float inte_limit,
						 float p,
						 float i,
						 float d,
						 float n,
						 float dt,
						 float deadband);
  void (*f_pid_reset)(struct pid_1dof_t  *pid, float p, float i, float d);

} pid_1dof_t ;

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
	);

void PID_1DOF_calc(pid_1dof_t *pid, float actual, float setpoint, float dt);
void PID_1DOF_Reset(pid_1dof_t *pid);

//extern pid_1dof_t rpm_pid_M3508[4];

#endif /* INC_PID_H_ */
