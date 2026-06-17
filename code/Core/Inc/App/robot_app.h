#ifndef ROBOT_APP_H
#define ROBOT_APP_H

#include "main.h"

//======================
//  State machine
//======================

typedef enum {
    STATE_STANDBY,
    STATE_INIT,
    STATE_IDLE,
    STATE_BALANCING,
    STATE_FALLEN,
    STATE_ERROR
} RobotState_t;

//======================
//  
//======================

void RobotInit(void);
void RobotUpdate(void);
void RobotLog(void);

#endif