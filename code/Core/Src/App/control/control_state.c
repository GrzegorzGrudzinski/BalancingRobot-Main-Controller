/*
    control_state.c
*/

#include "config.h"
#include "control_state.h"
#include "imu_handler.h"
#include "dualFOC_communication.h"

extern FeedbackPacket_t motor_feedback_data;

void updateControlState(ControlState_t *state) {
    IMU_Update();
    state->pitch      = IMU_GetPitchAngle();
    state->pitch_rate = IMU_GetPitchRate();
    state->yaw_rate   = IMU_GetYawRate();

    state->left_wheel_pos  = motor_feedback_data.motor1_pos * WHEEL_RADIUS;
    state->right_wheel_pos = motor_feedback_data.motor2_pos * WHEEL_RADIUS;
    
    float left_vel  = motor_feedback_data.motor1_vel * WHEEL_RADIUS;
    float right_vel = motor_feedback_data.motor2_vel * WHEEL_RADIUS; 
    
    state->velocity = (left_vel + right_vel)*0.5f ;
    state->position = (state->left_wheel_pos + state->right_wheel_pos) * 0.5f;

    state->yaw = (state->right_wheel_pos - state->left_wheel_pos) / (ROBOT_WIDTH);
}