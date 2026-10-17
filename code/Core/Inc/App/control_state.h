/*
  control_state.h
*/

#ifndef CONTROL_STATE_H_
#define CONTROL_STATE_H_

#include "control_types.h"
#include "dualFOC_communication.h" // for feedback packet

void updateControlState(ControlState_t *state, const FeedbackPacket_t *fb);

#endif /* CONTROL_STATE_H_ */
