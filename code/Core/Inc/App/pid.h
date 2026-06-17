#ifndef PID_H_
#define PID_H_

typedef struct
{
    float Kp;
    float Ki;
    float Kd;

    float max_output;
    float max_i;
    float max_sum;

    float error;
    float last_error;
    float error_sum;

    float P;
    float I;
    float D;

    float output;

    uint32_t timeSample; // 
    uint32_t prevTime;
    
    float lastPos;
} PID_t;

void pidInit (PID_t *pid, float Kp, float Ki,float Kd, float max_output, float max_i, uint32_t timeSample);
float run_pid(PID_t *pid, float position, float setPoint);
void pid_reset(PID_t *pid, float current_pos);


#endif /* PID_H_ */
