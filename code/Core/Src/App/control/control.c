/*
    control.c

    base controller
*/

#include "control.h"

#include <stddef.h> // for NULL 
#include <string.h> // for memset

void  controlInit (Controller_t *ctrl, void* config_data) {
    ctrl->config_data = config_data;

    ctrl->target_state.target_pitch = 0.0f;
    ctrl->target_state.target_position = 0.0f;
    ctrl->target_state.target_yaw = 0.0f;

    if (ctrl->init != NULL) {
        ctrl->init(ctrl, config_data);
    }
}


void controlRun(Controller_t *ctrl, float dt ,float* out_l, float* out_r) {
    if (ctrl->run != NULL) {
        ctrl->run(ctrl, dt, out_l, out_r);
    } else {
        *out_l = 0.0f;
        *out_r = 0.0f;
    }
}


void controlReset(Controller_t *ctrl) {
    if (ctrl->reset != NULL) {
        ctrl->reset(ctrl);
    }
    
    memset(&ctrl->debug, 0, sizeof(ctrl->debug));
    // TODO: add ctrl->reset_target 
}
