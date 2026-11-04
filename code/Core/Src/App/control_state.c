/*
    control_state.c
*/

#include "config.h"
#include "control_state.h"
#include "imu_handler.h"
#include "dualFOC_communication.h"

#define DEG_TO_RAD 0.0174533f
#define PITCH_COUPLING_SIGN (-1.0f) // pitch > 0 leans towards -x, so the encoder sees +r*pitch_rate 

uint8_t updateControlState(ControlState_t *state, const FeedbackPacket_t *fb) {
    uint8_t imu_err = IMU_Update();
    state->pitch      = IMU_GetPitchAngle();
    state->pitch_rate = IMU_GetPitchRate();
    state->yaw_rate   = IMU_GetYawRate();

    state->left_wheel_pos  = fb->motor1_pos;
    state->right_wheel_pos = fb->motor2_pos;

    float left_vel  = fb->motor1_vel;
    float right_vel = fb->motor2_vel;
    
    // Low-pass the pitch rate like the ESP wheel speed (MOT_VEL_FILTER) before subtracting it
    static float pitch_rate_lpf = 0.0f;
    const float a = WHEEL_VEL_LPF_TF / (WHEEL_VEL_LPF_TF + CONTROL_DT);
    pitch_rate_lpf = a * pitch_rate_lpf + (1.0f - a) * state->pitch_rate;
    
    // Velocity, with correction to angle change
    state->velocity = ((left_vel + right_vel)*0.5f + PITCH_COUPLING_SIGN*(pitch_rate_lpf*DEG_TO_RAD)) * WHEEL_RADIUS;

    state->position = (state->left_wheel_pos + state->right_wheel_pos)*WHEEL_RADIUS*0.5f;

    state->yaw = (state->right_wheel_pos - state->left_wheel_pos) * WHEEL_RADIUS / (ROBOT_WIDTH);

    return imu_err;
}

static int constrain(int x, int lower, int upper) {
    if (x < lower) return lower;
    if (x > upper) return upper;
    return x;
}   

void updateTargetState(TargetState_t *state,/*Limits_t limits,*/ const FeedbackPacket_t *extrn_fb /*eventually - pc feedback*/) {
    /*
     every value has to be checked with limits from Limits_t
    */
    // state->target_position = constrain(target, limit, -limit); // updated at the beginning and via feedback
    // 
    state->target_velocity = 0; // updated via pc_feedback
    state->target_pitch = 0;    // updated at the beginning 
    state->target_yaw = 0;      // updated at the beginning and via feedback
}

