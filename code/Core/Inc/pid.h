#ifndef POSITION_CHECKER_H_
#define POSITION_CHECKER_H_

typedef struct
{
    float Kp;
    float Ki;
    float Kd;

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

void pidInit (PID_t *pid, float Kp, float Ki,float Kd, uint32_t timeSample);
float run_pid(PID_t *pid, float position, float setPoint);



#endif /* POSITION_CHECKER_H_ */