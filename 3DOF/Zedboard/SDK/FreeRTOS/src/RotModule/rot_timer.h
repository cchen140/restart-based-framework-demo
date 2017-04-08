/*
 * rot_timer.h
 *
 *  Created on: Mar 11, 2017
 */

#ifndef SRC_ROTMODULE_ROT_TIMER_H_
#define SRC_ROTMODULE_ROT_TIMER_H_

#include "xil_types.h"

int RotTimerInit(void);
int RotTimerSetNextRestartTime(u16 u16Time);

#define ROT_TIMER_CMD_SET_RESTART_TIME  0xFE
#define ROT_TIMER_CMD_END_SECURE_INTERVAL  0xFD

#endif /* SRC_ROTMODULE_ROT_TIMER_H_ */
