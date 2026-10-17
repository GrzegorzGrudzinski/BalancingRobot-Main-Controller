/*
    PID.c

    1. check position
    2. calculate error
    3. correct yourself
*/
# include "pid.h"

void pidInit (PID_t *pid, const PID_Config_t* config)
{
    pid->Kp = config->Kp;
    pid->Ki = config->Ki;
    pid->Kd = config->Kd;

    pid->max_i = config->max_i;
    pid->max_output = config->max_output;
    
    // Initialize state variables
    pid_reset(pid, 0.0f);
}

static float pid_step(PID_t *pid, float error,float d, float dt) {
    pid->error = error;
    pid->P = pid->error * pid->Kp;
        
    pid->error_sum += pid->error*dt;
    pid->I = pid->Ki * pid->error_sum;
    pid->I = pid->error_sum* pid->Ki;
    if (pid->I > pid->max_i) {
        pid->I = pid->max_i;
        if (pid->Ki != 0.0f) pid->error_sum = pid->I / pid->Ki;
    }
    else if (pid->I < -pid->max_i) {
        pid->I = -pid->max_i;
        if (pid->Ki != 0.0f) pid->error_sum = pid->I / pid->Ki;
    }
        
    pid->D = d; 

    float out = pid->P + pid->I + pid->D;
    if (out > pid->max_output)  out = pid->max_output;
    if (out < -pid->max_output) out = -pid->max_output;
    
    pid->output = out;
    return out;
}


// D term provided externally
float run_pid_rate(PID_t *pid, float currVal, float rateVal, float setPoint, float dt) {
    if(dt<=0.0f) return pid->output;

    // Update state for next iteration
    pid->lastPos = currVal;
    return pid_step(pid, setPoint-currVal, -pid->Kd * rateVal, dt);
}

// D term calculated locally
float run_pid_dt(PID_t *pid, float currVal, float setPoint, float dt) {
   if(dt<=0.0f) return pid->output;

    float d =  -pid->Kd * (currVal - pid->lastPos) / dt;
    pid->lastPos = currVal;
    return pid_step(pid, setPoint-currVal, d, dt);
}

void pid_reset(PID_t *pid, float currnet_pos) {
    pid->error = 0.0f;
    pid->error_sum = 0.0f;
    pid->output = 0.0f;
    pid->lastPos = currnet_pos;

    pid->P = 0.0f;
    pid->I = 0.0f;
    pid->D = 0.0f;
}
