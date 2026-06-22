/*
    control.h
*/

#ifndef CONTROL_H_
#define CONTROL_H_

#include "control_state.h"

#include <stdint.h>

typedef enum
{
    CTRL_TYPE_PID
} ControllerType_e;

typedef struct Controller_t Controller_t;

typedef void (*CtrlInitFunct_t) (Controller_t *ctrl, void* config_data);
typedef void (*CtrlRunFunct_t)  (Controller_t *ctrl, const ControlState_t *state, float* out_l, float* out_r);
typedef void (*CtrlResetFunct_t)(Controller_t *ctrl, const ControlState_t *state);

struct Controller_t
{
    ControllerType_e ctrl_type;
    void* config_data;
    CtrlInitFunct_t init;
    CtrlRunFunct_t run;
    CtrlResetFunct_t reset;

    uint32_t timeSample;

    float curr_pitch;
    float curr_yaw;
    float curr_pos;
    
    float target_pitch;
    float target_yaw;
    float target_pos;
};

void controlInit (Controller_t *ctrl, ControllerType_e ctrl_type, void* config_data);
void controlRun  (Controller_t *ctrl, const ControlState_t *state, float* out_l, float* out_r);
void controlReset(Controller_t *ctrl, const ControlState_t *state);

#endif /* CONTROL_H_ */
