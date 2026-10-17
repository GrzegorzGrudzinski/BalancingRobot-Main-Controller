#ifndef PID_H_
#define PID_H_

typedef struct 
{
    float Kp;
    float Ki;
    float Kd;

    float max_output;
    float max_i;
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
    
    float error;
    float error_sum; // for i term calculation
    float lastPos;   // for d term calculation
    float output;
    
    float P;
    float I;
    float D;
} PID_t;

void pidInit (PID_t *pid, const PID_Config_t* config);

/* dt <= 0 returns last output, not changing the state */
float run_pid_rate(PID_t *pid, float currVal, float rateVal, float setPoint, float dt);
float run_pid_dt(PID_t *pid, float currVal, float setPoint, float dt);


void pid_reset(PID_t *pid, float current_pos);

#endif /* PID_H_ */
