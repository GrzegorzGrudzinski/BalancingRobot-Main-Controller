/*
    controllers.c
*/

#include "controllers_config.h"
#include "control.h"
#include "config.h"

#if CONTROLLER == CTRL_TYPE_PID
    #include "pid.h"
    #include "pid_wrapper.h"

    
    static PID_t pid;
#elif CONTROLLER == CTRL_TYPE_CASCADE_PID
    #include "cascade_pid.h"
    
    static CASCADE_t cascade;
#endif
    
void controlSystemInit (Controller_t* ctrl) {
#if CONTROLLER == CTRL_TYPE_PID
    PID_Config_t pid_cfg = {
      .Kp = PID_KP,
      .Ki = PID_KI,
      .Kd = PID_KD,
      .max_output = PID_MAX_OUT,
      .max_i = PID_MAX_I,
    //   .timeSample = LOOP_TIME
    };
    ctrl->ctrl_type = CTRL_TYPE_PID;

    ctrl->init = PID_InitFunct;
    ctrl->run  = PID_RunFunct;
    ctrl->reset= PID_ResetFunct;

    pidInit(&pid, &pid_cfg); 
    controlInit(ctrl, (void*)&pid);

#elif CONTROLLER == CTRL_TYPE_CASCADE_PID
    PID_Config_t position_pid_cfg = {
      .Kp = CASCADE_POS_KP,
      .Ki = CASCADE_POS_KI,
      .Kd = CASCADE_POS_KD,
      .max_output = CASCADE_POS_MAX_OUT,
      .max_i = 0.3f * CASCADE_POS_MAX_OUT,
    //   .timeSample = LOOP_TIME
    };
    
    PID_Config_t velocity_pid_cfg = {
        .Kp = CASCADE_VEL_KP,
        .Ki = CASCADE_VEL_KI,
        .Kd = CASCADE_VEL_KD,
        .max_output = CASCADE_VEL_MAX_OUT,
        .max_i = CASCADE_VEL_MAX_I, // was 0.3f * CASCADE_VEL_MAX_OUT,
        // .timeSample = LOOP_TIME
    };
    cascade.vel_i_gate = CASCADE_VEL_I_GATE;

    PID_Config_t pitch_pid_cfg = {
      .Kp = CASCADE_PITCH_KP,
      .Ki = CASCADE_PITCH_KI,
      .Kd = CASCADE_PITCH_KD,
      .max_output = CASCADE_PITCH_MAX_OUT,
      .max_i = 0.3f * CASCADE_PITCH_MAX_OUT,
    //   .timeSample = LOOP_TIME
    };    

    // PID_Config_t yaw_pid_cfg = {
    //   .Kp = CASCADE_YAW_KP,
    //   .Ki = CASCADE_YAW_KI,
    //   .Kd = CASCADE_YAW_KD,
    //   .max_output = CASCADE_YAW_MAX_OUT,
    //   .max_i = CASCADE_YAW_MAX_OUT,
    //   .timeSample = LOOP_TIME
    // };
    
    ctrl->ctrl_type = CTRL_TYPE_CASCADE_PID;

    ctrl->init = CASCADE_PID_InitFunct;
    ctrl->run  = CASCADE_PID_RunFunct;
    ctrl->reset= CASCADE_PID_ResetFunct;
    
    pidInit(&cascade.pid_position, &position_pid_cfg); 
    pidInit(&cascade.pid_velocity, &velocity_pid_cfg); 
    pidInit(&cascade.pid_pitch_angle, &pitch_pid_cfg); 
    // pidInit(&cascade.pid_yaw_angle, &yaw_pid_cfg); 
    controlInit(ctrl, (void*)&cascade);
    
#endif

}


void controlSetParam(Controller_t* ctrl, uint8_t id, float value) {
#if CONTROLLER == CTRL_TYPE_CASCADE_PID
    CASCADE_t *c = (CASCADE_t*)ctrl->config_data; // = &cascade, set in controlSystemInit()
    if (c == NULL || !(value >= 0.0f && value <= 100.0f)) return; // also rejects NaN and inf

    switch (id) {
      case CTRL_PARAM_POS_KP:    c->pid_position.Kp    = value; break;
      case CTRL_PARAM_VEL_KP:    c->pid_velocity.Kp    = value; break;
      case CTRL_PARAM_VEL_MAX_I: c->pid_velocity.max_i = value; break;
      case CTRL_PARAM_PITCH_KP:  c->pid_pitch_angle.Kp = value; break;
      case CTRL_PARAM_PITCH_KD:  c->pid_pitch_angle.Kd = value; break;
      case CTRL_PARAM_VEL_KI: {  // keep I continuous: I = Ki * error_sum (Ki = 0 drops I to 0)
        PID_t *pid = &c->pid_velocity;
        pid->Ki = value;
        pid->error_sum = (value != 0.0f) ? pid->I / value : 0.0f;
        break;
      }
      default: break;
    }
#else
    (void)ctrl; (void)id; (void)value; // no runtime gains for this controller type yet
#endif
}
