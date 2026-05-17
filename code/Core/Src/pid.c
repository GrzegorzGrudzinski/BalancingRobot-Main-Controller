/*
    1. check position
    2. calculate error
    3. correct yourself
*/
# include "main.h"
# include "pid.h"

uint32_t _getTime() {
    return HAL_GetTick(); // +(uint32_t)(uwTickFreq);
}

void pidInit (PID_t *pid, float Kp, float Ki,float Kd, float max_output, float max_i, uint32_t timeSample)
{
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
    pid->timeSample = timeSample; // 10 -> 100Hz, 5->200Hz  // + (uint32_t)(uwTickFreq)
    pid->prevTime = _getTime();

    pid->max_i = max_i;
    if (pid->Ki > 0) {
        pid->max_sum = pid->max_i / pid->Ki;
    } else {
        pid->max_sum = 0;
    } 
    pid->max_output = max_output;
    
    // Initialize state variables
    pid->error = 0.0f;
    pid->last_error = 0.0f;
    pid->error_sum = 0.0f;
    pid->lastPos = 0.0f;

    pid->P = 0.0f;
    pid->I = 0.0f;
    pid->D = 0.0f;
    pid->output = 0.0f;
    
}

// run_pid
float run_pid(PID_t *pid, float position, float setPoint) {
    uint32_t currentTime = _getTime();
    uint32_t timeChange = (currentTime - pid->prevTime);

    if ( timeChange >= pid->timeSample ) {
        pid->error = setPoint-position;

        float dt = (float)timeChange/1000.0f; // convert to seconds
        
        pid->P = pid->error * pid->Kp;
        
        pid->error_sum += pid->error*dt;
        if (pid->error_sum > pid->max_sum) 
            pid->error_sum = pid->max_sum;
        else if (pid->error_sum < -pid->max_sum) 
            pid->error_sum = -pid->max_sum;
        pid->I = pid->error_sum* pid->Ki;
        // if (pid->I > pid->max_i) pid->I = pid->max_i;
        // else if (pid->I < -pid->max_i) pid->I = -pid->max_i;
        
        float dposition = position-pid->lastPos;
        pid->D = -pid->Kd * (dposition/dt); // - because we use position change
        
        
        pid->output = pid->P+pid->I+pid->D;
        
        if (pid->output > pid->max_output) pid->output = pid->max_output;
        else if (pid->output < -pid->max_output) pid->output = -pid->max_output;

        // Update state for next iteration
        pid->lastPos = position;
        pid->last_error = pid->error;
        pid->prevTime = currentTime;
    }
    return pid->output;
}

void pid_reset(PID_t *pid) {
    // 1. Resetujemy historię uchybów i sumę całkowania (Integral Windup)
    pid->error = 0.0f;
    pid->last_error = 0.0f;
    pid->error_sum = 0.0f;
    
    // 2. Resetujemy historię pozycji (zapobiega strzałom z członu D)
    pid->lastPos = 0.0f;

    // 3. Resetujemy składowe wyjściowe dla pewności
    pid->P = 0.0f;
    pid->I = 0.0f;
    pid->D = 0.0f;
    pid->output = 0.0f;

    // 4. BARDZO WAŻNE: Aktualizujemy czas, żeby `dt` w następnym cyklu było małe
    pid->prevTime = _getTime();
}