/*
 * zedboard_io.h
 *
 *  Created on: Apr 5, 2017
 */

#ifndef SRC_ZEDBOARD_IO_H_
#define SRC_ZEDBOARD_IO_H_

#include "xil_io.h"

#define SWITCH_INPUT_ADDRESS	0x41200000
#define BUTTON_INPUT_ADDRESS	0x41200001
#define	LED_OUTPUT_ADDRESS		0x41210000

#define	getSwitchInput()	Xil_In32(SWITCH_INPUT_ADDRESS)

#endif /* SRC_ZEDBOARD_IO_H_ */
