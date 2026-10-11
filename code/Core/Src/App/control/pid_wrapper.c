/*
    pid_wrapper.c
*/

#include "control.h"
#include "pid.h"

#include <stddef.h> // for NULL

void  PID_InitFunct  (Controller_t *ctrl, void* config_data) {
    /*  */
}


void PID_RunFunct (Controller_t *ctrl, float* out_l, float* out_r) {
    PID_t* pid = (PID_t*)ctrl->config_data;
    if (pid == NULL) {
        *out_l = 0.0f;
        *out_r = 0.0f;
        return;
    }

    float output = run_pid_rate(pid, ctrl->observed_state.pitch, ctrl->observed_state.pitch_rate, ctrl->target_state.target_pitch);

    *out_l = output;
    *out_r = output;
}



void PID_ResetFunct (Controller_t *ctrl) {
    PID_t* pid = (PID_t*)ctrl->config_data;
    
    if (pid != NULL) {
        pid_reset(pid, ctrl->observed_state.pitch);
    }
}
