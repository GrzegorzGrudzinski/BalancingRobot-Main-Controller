/*
    control_state.c
*/

#include "config.h"
#include "control_state.h"
#include "imu_handler.h"
#include "dualFOC_communication.h"

// extern FeedbackPacket_t motor_feedback_data;

void updateControlState(ControlState_t *state, const FeedbackPacket_t *fb) {
    IMU_Update();
    state->pitch      = IMU_GetPitchAngle();
    state->pitch_rate = IMU_GetPitchRate();
    state->yaw_rate   = IMU_GetYawRate();

    // todo: disable irq ??

    state->left_wheel_pos  = fb->motor1_pos;
    state->right_wheel_pos = fb->motor2_pos;

    float left_vel  = fb->motor1_vel;
    float right_vel = fb->motor2_vel;
    
    state->velocity = (left_vel + right_vel)* WHEEL_RADIUS*0.5f ;
    state->position = (state->left_wheel_pos + state->right_wheel_pos)*WHEEL_RADIUS*0.5f;

    state->yaw = (state->right_wheel_pos - state->left_wheel_pos) * WHEEL_RADIUS / (ROBOT_WIDTH);
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