/*
    cascade_pid.c
*/

#include "cascade_pid.h"
#include "pid.h"
// #include "pid_wrapper.h"

void CASCADE_PID_InitFunct  (Controller_t *ctrl, void* config_data) {

}


void CASCADE_PID_RunFunct (Controller_t *ctrl, float* out_l, float* out_r) {
    CASCADE_t* cascade = (CASCADE_t*)ctrl->config_data;

    if (cascade == NULL) {
        *out_l = 0.0f;
        *out_r = 0.0f;
        return;
    }

    // Position loop
    float target_velocity = run_pid_rate(&cascade->pid_position, ctrl->observed_state.position, ctrl->observed_state.velocity, ctrl->target_state.target_position);
    float velocity_final_target = target_velocity; // todo rc control (+ set_velocity)
    
    // Velocity loop
    float target_pitch = run_pid_rate(&cascade->pid_velocity , ctrl->observed_state.velocity, 0.0f, velocity_final_target);
    float pitch_final_target = ctrl->target_state.target_pitch + target_pitch;
    
    // Pitch loop
    float output = run_pid_rate(&cascade->pid_pitch_angle, ctrl->observed_state.pitch, ctrl->observed_state.pitch_rate, pitch_final_target);

    // Yaw loop
    // float yaw_offet = run_pid_rate(&cascade->pid_yaw_angle, ctrl->observed_state.yaw, ctrl->observed_state.yaw_rate, ctrl->target_yaw);

    // Output
    *out_l = output;  // - yaw_offset
    *out_r = output;  // + yaw_offset

    ctrl->control_data = cascade;
}


void CASCADE_PID_ResetFunct (Controller_t *ctrl) {
    CASCADE_t* cascade = (CASCADE_t*)ctrl->config_data;
    
    if (cascade != NULL) {
        pid_reset(&cascade->pid_position,    ctrl->observed_state.position);
        pid_reset(&cascade->pid_velocity,    ctrl->observed_state.velocity);
        pid_reset(&cascade->pid_pitch_angle, ctrl->observed_state.pitch);
        // pid_reset(&cascade->pid_yaw_angle,ctrl->observed_state.yaw);
    }
}
 