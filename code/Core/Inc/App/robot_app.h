#ifndef ROBOT_APP_H
#define ROBOT_APP_H

#include "main.h"
#include "pc_telemetry.h"
//======================
//  State machine
//======================

typedef enum {
    STATE_STANDBY,
    STATE_CONNECT,
    STATE_INIT,
    STATE_BALANCING,
    STATE_FALLEN,
    STATE_ERROR,
    STATE_STOP
} RobotState_t;

//======================
//  
//======================

void RobotInit(void);
TelemetryPacket_t* RobotUpdate(void);
// void RobotLog(void);

#endif