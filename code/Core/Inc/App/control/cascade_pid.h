#ifndef CASCADE_PID_H_
#define CASCADE_PID_H_

#include "pid.h"
#include "control.h"

#include <stddef.h> // for NULL

/*
    struct for calculated targets
*/
typedef struct
{
    PID_t pid_position;
    PID_t pid_velocity;
    PID_t pid_pitch_angle;
    // PID_t pid_yaw_angle;
    
} CASCADE_t;


void CASCADE_PID_InitFunct  (Controller_t *ctrl, void* config_data);
void CASCADE_PID_RunFunct (Controller_t *ctrl, float dt, float* out_l, float* out_r);
void CASCADE_PID_ResetFunct (Controller_t *ctrl);
 
#endif /* CASCADE_PID_H_ */