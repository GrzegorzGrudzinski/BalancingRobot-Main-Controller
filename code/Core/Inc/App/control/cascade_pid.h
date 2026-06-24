#ifndef CASCADE_PID_H_
#define CASCADE_PID_H_

#include "main.h"
#include "pid.h"
#include "control.h"
#include "control_state.h"

#include <stddef.h> // for NULL

typedef struct
{
    PID_t pid_position;
    PID_t pid_velocity;
    PID_t pid_pitch_angle;
    
    // todo yaw
    // PID_t pid_yaw_angle;
    
} CASCADE_t;


void CASCADE_PID_InitFunct  (Controller_t *ctrl, void* config_data);
void CASCADE_PID_RunFunct (Controller_t *ctrl, const ControlState_t *state, float* out_l, float* out_r);
void CASCADE_PID_ResetFunct (Controller_t *ctrl, const ControlState_t *state);
 
#endif /* CASCADE_PID_H_ */