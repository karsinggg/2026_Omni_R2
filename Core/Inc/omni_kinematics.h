/*
 * omni_kinematics.h
 *
 *  Created on: May 6, 2026
 *      Author: Acer
 */

#ifndef INC_OMNI_KINEMATICS_H_
#define INC_OMNI_KINEMATICS_H_

#include <stdbool.h>

void omni_algorithm(float v_tx, float v_ty, float w, float target_speeds[], bool speed_up);

#endif /* INC_OMNI_KINEMATICS_H_ */
