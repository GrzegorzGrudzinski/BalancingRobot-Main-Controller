/*
  control_state.h
*/

#ifndef CONTROL_STATE_H_
#define CONTROL_STATE_H_

#include "control_types.h"
#include "dualFOC_communication.h" // for feedback packet
#include "pc_command.h" // for PcCommand_t

uint8_t updateControlState(ControlState_t *state, const FeedbackPacket_t *fb); // 0 = ok, 1 = imu err
void targetReset(TargetState_t *t_state, const ControlState_t *c_state);
void pcProcessCMD(const PcCommand_t *cmd, TargetState_t *t_state, const ControlState_t *c_state); // HEARTBEAT, STOP, SET_TARGET

#endif /* CONTROL_STATE_H_ */
