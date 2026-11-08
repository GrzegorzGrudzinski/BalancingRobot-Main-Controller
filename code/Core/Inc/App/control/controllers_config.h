/*
    controllers.h

    This module provides a unified interface for various control algorithms

*/

#ifndef CONTROLLERS_H_
#define CONTROLLERS_H_

#include "control.h"

typedef enum {
    CTRL_PARAM_POS_KP    = 1,
    CTRL_PARAM_VEL_KP    = 2,
    CTRL_PARAM_VEL_KI    = 3,
    CTRL_PARAM_VEL_MAX_I = 4,
    CTRL_PARAM_PITCH_KP  = 5,
    CTRL_PARAM_PITCH_KD  = 6,
} CtrlParamId_t;


void controlSystemInit (Controller_t* ctrl);
void controlSetParam   (Controller_t* ctrl, uint8_t id, float value); // PC SET_PARAM: one gain, 0..100, unknown ids ignored

#endif /* CONTROLLERS_H_ */