#ifndef MB_CONTROLLER_H
#define MB_CONTROLLER_H

#include "mb_structs.h"
#define CFG_PATH "lqr.cfg"
#define PID_PATH "pid.cfg"
#define TARGET_PATH "targets.cfg"

double maxPhiControlStep;

int mb_controller_init();
int mb_controller_load_config();
int mb_controller_update(mb_state_t* mb_state, Setpoint* sp);
int mb_controller_cleanup();
void reset_phi_controller();

#endif
