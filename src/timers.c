/*
 * Copyright (C) 2019, Ksenia Balistreri
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the BSD license.  See the LICENSE file for details.
 */

#include <stdlib.h>
#include <stdint.h>
#include "timers.h"

#define FREQ_4096Hz 	0x00
#define FREQ_262144Hz 	0x01
#define FREQ_65536Hz 	0x10
#define FREQ_16386Hz 	0x11

#define DIV_BIT4_MASK	0x0008
#define DIV_BIT6_MASK	0x0020
#define DIV_BIT8_MASK	0x0080
#define DIV_BIT10_MASK	0x0200

uint8_t timer_inc(pb_vm_t* vm) {
	uint8_t div_out;
	uint8_t tac_enable = (vm->io_registers[TAC] & 0x04) >> 2;

	switch (vm->io_registers[TAC] & 0x03) {
		case FREQ_4096Hz:
			div_out = ((vm->divider_register & DIV_BIT10_MASK) >> 9) & tac_enable;
			break;
		case FREQ_262144Hz:
			div_out = ((vm->divider_register & DIV_BIT4_MASK) >> 3) & tac_enable;
			break;
		case FREQ_65536Hz:
			div_out = ((vm->divider_register & DIV_BIT6_MASK) >> 5) & tac_enable;
			break;
		case FREQ_16386Hz:
			div_out = ((vm->divider_register & DIV_BIT8_MASK) >> 7) & tac_enable;
			break;
		}

	uint8_t t_inc = (~div_out) & vm->div_delay;
	vm->div_delay = div_out;

	return t_inc;
}

int pb_timers_update(pb_vm_t* vm) {
	uint8_t timer_if = 0;
	uint8_t prev_timer = vm->io_registers[TIMA];

	vm->divider_register++;

	vm->io_registers[TIMA] += timer_inc(vm);

	if (vm->io_registers[TIMA] < prev_timer) {
		timer_if = 1;
		vm->io_registers[TIMA] = vm->io_registers[TMA];
	}

	if (timer_if) {
		vm->io_registers[INTERRUPT_FLAGS] |= TIMER_INT_FLAG_MASK;
	}

	return 0;
}
