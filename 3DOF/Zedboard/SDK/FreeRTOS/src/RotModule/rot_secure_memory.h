/*
 * rot_secure_memory.h
 *
 *  Created on: Mar 12, 2017
 */

#ifndef SRC_ROTMODULE_ROT_SECURE_MEMORY_H_
#define SRC_ROTMODULE_ROT_SECURE_MEMORY_H_

int RotSecureMemoryInit(void);
u8 RotSecureMemoryReadTestByte();
int RotSecureMemoryUpdateTestByte(u8 inVal);
void RotSecureMemoryClose(void);

#endif /* SRC_ROTMODULE_ROT_SECURE_MEMORY_H_ */
