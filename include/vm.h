/*
 * Copyright (C) 2019, Ksenia Balistreri
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the BSD license.  See the LICENSE file for details.
 */

#ifndef _VM_H_
#define _VM_H_

#include <stdint.h>
#include "structs.h"

#define INTERRUPT_FLAGS 	0x0f
#define TIMA 				0x05
#define TMA 				0x06
#define TAC			 		0x07

#define VB_INT_FLAG_MASK			0x01
#define LCD_STAT_INT_FLAG_MASK		0x02
#define TIMER_INT_FLAG_MASK			0x04
#define SERIAL_INT_FLAG_MASK		0x08
#define JOYPAD_INT_FLAG_MASK		0x10

#define SCREEN_WIDTH	160
#define SCREEN_HEIGHT	144

void pb_io_regs_init(pb_vm_t* vm);
int pb_vm_run(pb_vm_t *vm);


#endif
