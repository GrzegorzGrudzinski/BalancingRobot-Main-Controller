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