/*
    control_state.c
*/

#include "control_state.h" //
#include "config.h" // 
#include "imu_handler.h" // IMU_*
#include "dualFOC_communication.h" // FeedbackPacket_t
#include "pc_command.h"// PcCommand_t
#include <math.h> // fabsf

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

// static int constrain(int x, int lower, int upper) {
//     if (x < lower) return lower;
//     if (x > upper) return upper;
//     return x;
// }   

static void pcSetTargetState(TargetState_t *t_state, const ControlState_t *c_state, const PcCommand_t *cmd) {
    /*
      TODO: every value has to be checked with limits from Limits_t
     // t_state->target_position = constrain(target, limit, -limit); // updated at the beginning and via feedback
    */
    switch (cmd->param_id) {
        case PC_PARAM_TARGET_POS:
           if (!(fabsf(cmd->value) <= PC_POS_STEP_MAX)) return; // also rejects NaN and inf
           t_state->target_position = c_state->position + cmd->value; 
           break;
    case PC_PARAM_TARGET_PITCH: 
    //    t_state->target_pitch = cmd->value; break;
    case PC_PARAM_TARGET_YAW:
    //    t_state->target_yaw = cmd->value; break;
    case PC_PARAM_TARGET_VEL:
    //    t_state->target_velocity = cmd->value; break;
    default:
        break;
    }

}

void pcProcessCMD (const PcCommand_t *cmd, TargetState_t *t_state, const ControlState_t *c_state) {

    switch (cmd->type) {
        case PC_CMD_SET_TARGET: pcSetTargetState(t_state, c_state, cmd);   break;
        case PC_CMD_STOP:       targetReset(t_state, c_state);             break;
        case PC_CMD_HEARTBEAT:  break; // refreshes the timeout (robot_app.c)
        // case PC_CMD_SET_PARAM:  pcSetCtrlParam(cmd->param_id, cmd->value); break;
        default: break; // SET_PARAM handled in robot_app.c
    }
}

void targetReset(TargetState_t *t_state, const ControlState_t *c_state) {
    // hold current position as target so robot does not go to random point after reset
    t_state->target_position = c_state->position;
    t_state->target_velocity = 0.0f;
    t_state->target_pitch    = TARGET_ANGLE;
    t_state->target_yaw      = c_state->yaw; //0.0f;
}