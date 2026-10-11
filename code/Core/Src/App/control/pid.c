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

    pid->P = 0.0f;
    pid->I = 0.0f;
    pid->D = 0.0f;
}

// run_pid
float run_pid_rate(PID_t *pid, float currVal, float rateVal, float setPoint) {
    uint32_t currentTime = _getTime();
    uint32_t timeChange = (currentTime - pid->prevTime);

    if ( timeChange >= pid->timeSample ) {
        pid->error = setPoint-currVal;

        float dt = (float)pid->timeSample / 1000.0f; // convert to seconds

        pid->P = pid->error * pid->Kp;
        
        pid->error_sum += pid->error*dt;

        pid->I = pid->error_sum* pid->Ki;
        if (pid->I > pid->max_i) {
            pid->I = pid->max_i;
            if (pid->Ki != 0.0f) pid->error_sum = pid->I / pid->Ki;
        }
        else if (pid->I < -pid->max_i) {
            pid->I = -pid->max_i;
            if (pid->Ki != 0.0f) pid->error_sum = pid->I / pid->Ki;
        }
        
        pid->D = -pid->Kd * rateVal; 
        
        pid->output = pid->P+pid->I+pid->D;
        
        if (pid->output > pid->max_output) pid->output = pid->max_output;
        else if (pid->output < -pid->max_output) pid->output = -pid->max_output;

        // Update state for next iteration
        pid->lastPos = currVal;
        pid->prevTime += pid->timeSample;
    }
    return pid->output;
}


float run_pid_dt(PID_t *pid, float currVal, float setPoint) {
    uint32_t currentTime = _getTime();
    uint32_t timeChange = (currentTime - pid->prevTime);

    if ( timeChange >= pid->timeSample ) {
        pid->error = setPoint-currVal;

        float dt = (float)pid->timeSample / 1000.0f; // convert to seconds

        pid->P = pid->error * pid->Kp;
        
        pid->error_sum += pid->error*dt;
        // if (pid->error_sum > pid->max_sum) 
        //     pid->error_sum = pid->max_sum;
        // else if (pid->error_sum < -pid->max_sum) 
        //     pid->error_sum = -pid->max_sum;
        pid->I = pid->error_sum* pid->Ki;
        if (pid->I > pid->max_i) {
            pid->I = pid->max_i;
            if (pid->Ki != 0.0f) pid->error_sum = pid->I / pid->Ki;
        }
        else if (pid->I < -pid->max_i) {
            pid->I = -pid->max_i;
            if (pid->Ki != 0.0f) pid->error_sum = pid->I / pid->Ki;
        }
        
        float dposition = currVal-pid->lastPos;
        pid->D = -pid->Kd * (dposition/dt); // position change
        // D = -pid->Kd * rateVal; // position change
        
        pid->output = pid->P+pid->I+pid->D;
        
        if (pid->output > pid->max_output) pid->output = pid->max_output;
        else if (pid->output < -pid->max_output) pid->output = -pid->max_output;

        // Update state for next iteration
        pid->lastPos = currVal;
        pid->prevTime += pid->timeSample;
    }
    return pid->output;
}



void pid_reset(PID_t *pid, float currnet_pos) {
    pid->error = 0.0f;
    pid->error_sum = 0.0f;
    pid->lastPos = currnet_pos;
    pid->output = 0.0f;
    pid->prevTime = _getTime();

    pid->P = 0.0f;
    pid->I = 0.0f;
    pid->D = 0.0f;
}
