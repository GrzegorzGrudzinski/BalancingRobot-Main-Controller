/*
    control.c

    base controller
*/

#include "control.h"
#include "pid_wrapper.h"

#include <stddef.h> // for NULL 

void  controlInit (Controller_t *ctrl, ControllerType_e ctrl_type, void* config_data) {
    ctrl->ctrl_type = ctrl_type;
    ctrl->config_data = config_data;

    ctrl->target_pitch = 0.0f;
    ctrl->target_yaw = 0.0f;
    ctrl->target_pos = 0.0f;

    switch (ctrl_type)
    {
    case CTRL_TYPE_PID:
        ctrl->init = PID_InitFunct;
        ctrl->run  = PID_RunFunct;
        ctrl->reset= PID_ResetFunct;
        break;
    
    default:
        ctrl->init = NULL;
        ctrl->run  = NULL;
        ctrl->reset= NULL;
        break;
    }

    if (ctrl->init != NULL) {
        ctrl->init(ctrl, config_data);
    }
}


void controlRun  (Controller_t *ctrl, const ControlState_t *state, float* out_l, float* out_r) {
    if (ctrl->run != NULL) {
        ctrl->run(ctrl, state, out_l, out_r);
    } else {
        *out_l = 0.0f;
        *out_r = 0.0f;
    }
}


void controlReset(Controller_t *ctrl, const ControlState_t *state) {
    if (ctrl->run != NULL) {
        ctrl->reset(ctrl, state);
    }
}
