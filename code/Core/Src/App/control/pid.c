/*
    PID.c

    1. check position
    2. calculate error
    3. correct yourself
*/
# include "main.h"
# include "pid.h"

static uint32_t _getTime() {
    return HAL_GetTick(); // +(uint32_t)(uwTickFreq);
}

void pidInit (PID_t *pid, const PID_Config_t* config)
{
    pid->Kp = config->Kp;
    pid->Ki = config->Ki;
    pid->Kd = config->Kd;
    pid->timeSample = config->timeSample; // 10 -> 100Hz, 5->200Hz  // + (uint32_t)(uwTickFreq)
    pid->prevTime = _getTime();

    pid->max_i = config->max_i;
    pid->max_output = config->max_output;
    
    // Initialize state variables
    pid->error = 0.0f;
    pid->error_sum = 0.0f;
    pid->lastPos = 0.0f;
    pid->output = 0.0f;
}

// run_pid
float run_pid(PID_t *pid, float currVal, float setPoint) {
    uint32_t currentTime = _getTime();
    uint32_t timeChange = (currentTime - pid->prevTime);

    if ( timeChange >= pid->timeSample ) {
        pid->error = setPoint-currVal;

        float dt = (float)timeChange/1000.0f; // convert to seconds
        float P,I,D;

        P = pid->error * pid->Kp;
        
        pid->error_sum += pid->error*dt;
        // if (pid->error_sum > pid->max_sum) 
        //     pid->error_sum = pid->max_sum;
        // else if (pid->error_sum < -pid->max_sum) 
        //     pid->error_sum = -pid->max_sum;
        I = pid->error_sum* pid->Ki;
        if (I > pid->max_i) {
            I = pid->max_i;
            if (pid->Ki != 0.0f) pid->error_sum = I / pid->Ki;
        }
        else if (I < -pid->max_i) {
            I = -pid->max_i;
            if (pid->Ki != 0.0f) pid->error_sum = I / pid->Ki;
        }
        
        float dposition = currVal-pid->lastPos;
        D = -pid->Kd * (dposition/dt); // position change
        
        
        pid->output = P+I+D;
        
        if (pid->output > pid->max_output) pid->output = pid->max_output;
        else if (pid->output < -pid->max_output) pid->output = -pid->max_output;

        // Update state for next iteration
        pid->lastPos = currVal;
        pid->prevTime = currentTime;
    }
    return pid->output;
}

void pid_reset(PID_t *pid, float currnet_pos) {
    pid->error = 0.0f;
    pid->error_sum = 0.0f;
    pid->lastPos = currnet_pos;
    pid->output = 0.0f;
    pid->prevTime = _getTime();
}
