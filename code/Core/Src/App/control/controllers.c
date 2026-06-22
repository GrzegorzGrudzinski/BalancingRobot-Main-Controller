
#include "controllers.h"
#include "control.h"
#include "config.h"

#if CONTROLLER == CTRL_TYPE_PID
    #include "pid.h"
    
    static PID_t pid;
// #elif CONTROLLER ==
    
#endif
    
    void controlSystemInit (Controller_t* ctrl) {
        #if CONTROLLER == CTRL_TYPE_PID
    PID_Config_t pid_cfg = {
      .Kp = PID_KP,
      .Ki = PID_KI,
      .Kd = PID_KD,
      .max_output = PID_MAX_OUT,
      .max_i = PID_MAX_I,
      .timeSample = LOOP_TIME
    };

    pidInit(&pid, &pid_cfg); 
    controlInit(ctrl, CTRL_TYPE_PID, (void*)&pid);

// #elif CONTROLLER == 

#endif

}
