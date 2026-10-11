#ifndef PID_H_
#define PID_H_

#include <stdint.h>


typedef struct 
{
    float Kp;
    float Ki;
    float Kd;

    float max_output;
    float max_i;

    uint32_t timeSample; // 
} PID_Config_t;

/*
    struct for live PID calculations
*/
typedef struct
{
    
    float Kp;
    float Ki;
    float Kd;
    
    float max_output;
    float max_i;
    
    uint32_t timeSample;
    uint32_t prevTime;
    
    float error;
    float error_sum;
    float lastPos;
    float output;
    
    float P;
    float I;
    float D;
} PID_t;

void pidInit (PID_t *pid, const PID_Config_t* config);
float run_pid_rate(PID_t *pid, float currVal, float rateVal, float setPoint);
float run_pid_dt(PID_t *pid, float currVal, float setPoint);
void pid_reset(PID_t *pid, float current_pos);

#endif /* PID_H_ */
