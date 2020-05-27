/*
 * Copyright (C) 2019, Ksenia Balistreri
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the BSD license.  See the LICENSE file for details.
 */

#include <string.h>
#include "vm.h"
#include "audio.h"
#include "timers.h"
#include "display.h"
#include "keypad.h"
#include "cpu.h"
#include "cart.h"
#include "opcodes.h"

void pb_io_regs_init(pb_vm_t* vm) {
	memset(vm->io_registers, 0xff, sizeof(IO_REGS_SIZE));

	vm->io_registers[INTERRUPT_FLAGS] = 0xe1;
	vm->ie_register = 0;
	vm->io_registers[TIMA] = 0;
	vm->io_registers[TMA] = 0;
	vm->io_registers[TAC]	= 0xf8;
	vm->divider_register = 0xabcb;
	vm->div_delay = 0;
}

int pb_vm_run(pb_vm_t* vm) {
	pb_cpu_init(&vm->cpu);
	pb_cart_init(&vm->cart);
	pb_io_regs_init(vm);
	pb_display_init();
	pb_audio_init();

	vm->pb_run = 1;

	while (vm->pb_run) {
		pb_timers_update(vm);
		pb_cpu_cycle(vm);
		pb_keypad_scan(vm);
		pb_display_draw(vm);
		pb_audio_play(vm);
	}

	return 0;
}
