/*
    pid_wrapper.h

    This module provides a unified interface for PID algorithm
*/

#ifndef PID_WRAPPER_H_
#define PID_WRAPPER_H_

#include "control.h"
#include "pid.h"

void PID_InitFunct (struct Controller_t *ctrl, void* config_data);
void PID_RunFunct  (Controller_t *ctrl, float dt, float* out_l, float* out_r);
void PID_ResetFunct(struct Controller_t *ctrl);

#endif /* PID_WRAPPER_H_ */