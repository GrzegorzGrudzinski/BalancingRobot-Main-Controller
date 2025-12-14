// /*
//     1. check position
//     2. calculate error
//     3. correct yourself
// */
// # include "main.h"

// typedef struct
// {
//     float Kp;
//     float Ki;
//     float Kd;

//     float error;
//     float last_error;
//     float error_sum;

//     float P;
//     float I;
//     float D;

//     float output;

//     uint32_t timeSample; // 
//     uint32_t prevTime;
//     float lastPos;
// } PID_t;


// void pidInit (PID_t *pid, float Kp, float Ki,float Kd, uint32_t timeSample)
// {
//     pid->Kp = Kp;
//     pid->Ki = Ki;
//     pid->Kd = Kd;
//     pid->timeSample = timeSample; // + (uint32_t)(uwTickFreq)
//     pid->prevTime = getTime();

//     // Initialize state variables
//     pid->error = 0.0f;
//     pid->last_error = 0.0f;
//     pid->error_sum = 0.0f;
//     pid->lastPos = 0.0f;

//     pid->P = 0.0f;
//     pid->I = 0.0f;
//     pid->D = 0.0f;
//     pid->output = 0.0f;
// }


// uint32_t getTime() {
//     return HAL_GetTick(); // +(uint32_t)(uwTickFreq);
// }

// // run_pid
// float run_pid(PID_t *pid, float position, float setPoint) {
//     uint32_t currentTime = getTime();
//     uint32_t timeChange = (currentTime - pid->prevTime);

//     if ( timeChange >= pid->timeSample ) {
//         pid->error = setPoint-position;

//         float dt = (float)timeChange/1000.0f; // convert to seconds
        
//         pid->P = pid->error * pid->Kp;
        
//         pid->error_sum += pid->error*dt;
//         pid->I = pid->error_sum* pid->Ki; // TODO I max and min values
//         const float I_MAX = 180.0f;
//         const float I_MIN = -180.0f;
//         if (pid->I > I_MAX) pid->I = I_MAX;
//         else if (pid->I < I_MIN) pid->I = I_MIN;
        
//         float dposition = position-pid->lastPos;
//         pid->D = -pid->Kd * (dposition/dt); // - because we use position change

        
//         pid->output = pid->P+pid->I+pid->D;
//         const float OUT_MAX = 180.0f;
//         const float OUT_MIN = -180.0f;
//         if (pid->output > OUT_MAX) pid->output = OUT_MAX;
//         else if (pid->output < OUT_MIN) pid->output = OUT_MIN;

//         // Update state for next iteration
//         pid->lastPos = position;
//         pid->last_error = pid->error;
//         pid->prevTime = currentTime;
//     }
//     return pid->output;
// }


// // send_position

// void convertToMotors () {
//     // angle to torque commands



// }