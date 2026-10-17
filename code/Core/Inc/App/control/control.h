/*
    control.h
*/

#ifndef CONTROL_H_
#define CONTROL_H_

#include "control_types.h"

#include <stdint.h>

typedef struct Controller_t Controller_t;

typedef void (*CtrlInitFunct_t) (Controller_t *ctrl, void* config_data);
typedef void (*CtrlRunFunct_t)  (Controller_t *ctrl, float dt, float* out_l, float* out_r);
typedef void (*CtrlResetFunct_t)(Controller_t *ctrl);

struct Controller_t
{
    uint8_t ctrl_type;
    void* config_data;
    CtrlInitFunct_t init;
    CtrlRunFunct_t run;
    CtrlResetFunct_t reset;

    void* control_data; // data from controller (eg. PID values)
    ControlState_t observed_state; // current observed values
    TargetState_t target_state; // target values

    ControlDebug_t debug; // values of the last run
};

void controlInit (Controller_t *ctrl, void* config_data);
void controlRun  (Controller_t *ctrl, float dt, float* out_l, float* out_r);
void controlReset(Controller_t *ctrl);

#endif /* CONTROL_H_ */



