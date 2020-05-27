/*
 * Copyright (C) 2019, Ksenia Balistreri
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the BSD license.  See the LICENSE file for details.
 */

#include <stdint.h>
#include <stdlib.h>
#include "cpu.h"
#include "opcodes.h"
#include "mem.h"



#define HRAM_BASE_ADDR 0xff80

#define VERTICAL_BLANK_INTERRUPT	0x0040
#define LCD_STAT_INTERRUPT 			0x0048
#define TIMER_INTERRUPT 			0x0050
#define SERIAL_INTERRUPT 			0x0058
#define JOYPAD_INTERRUPT 			0x0060

#define INTERRUPT_WAIT_CYCLE 20

#define PREFIX_CB 0xcb

#define INTERRUPT_ENABLED(flag, ie, mask) ((flag & mask) != 0) && ((ie & mask) != 0)

#define INTERRUPT_MASK 0x1f


void pb_cpu_init(pb_cpu_t* cpu) {
	cpu->registers[REG_A] = 0x01;
	cpu->registers[REG_F] = 0xb0;
	cpu->registers[REG_B] = 0x00;
	cpu->registers[REG_C] = 0x13;
	cpu->registers[REG_D] = 0x00;
	cpu->registers[REG_E] = 0xd8;
	cpu->registers[REG_H] = 0x01;
	cpu->registers[REG_L] = 0x4d;
	cpu->programm_counter = 0x0100;
	cpu->registers[REG_SP] = 0xff;
	cpu->registers[REG_SP + 1] = 0xfe;
	cpu->ime = 0;
	cpu->cpu_state = INST_FETCH_DECODE;
	cpu->cpu_mode = NORMAL;
	cpu->wait_cycles = 0;
}

uint16_t pb_get_uint_16_reg(pb_cpu_t* cpu, int i) {
	return cpu->registers[i] << 8 | cpu->registers[i + 1];
};

void pb_set_uint16_reg(pb_cpu_t* cpu, int i, uint16_t data) {
	cpu->registers[i] = data >> 8;
	cpu->registers[i + 1] = data;
};

void pb_cpu_stack_push(pb_vm_t* vm, uint16_t data) {
	int sp = pb_get_uint_16_reg(&vm->cpu, REG_SP);
	pb_mem_write(vm, (sp - 1), (data >> 8));
	pb_mem_write(vm, (sp - 2), data);
	sp -= 2;
	pb_set_uint16_reg(&vm->cpu, REG_SP, sp);
}

uint16_t pb_cpu_stack_pop(pb_vm_t* vm) {
	int sp = pb_get_uint_16_reg(&vm->cpu, REG_SP);
	uint16_t data = (pb_mem_read(vm, sp + 1) << 8) | pb_mem_read(vm, sp);
	sp += 2;
	pb_set_uint16_reg(&vm->cpu, REG_SP, sp);
	return data;
}

void pb_cpu_interrupt_check(pb_vm_t* vm) {
	if (((vm->io_registers[INTERRUPT_FLAGS] & vm->ie_register & INTERRUPT_MASK) != 0)) {
		vm->cpu.cpu_mode = NORMAL;

		if (vm->cpu.ime == 1) {
			pb_cpu_stack_push(vm, vm->cpu.programm_counter);

			if (INTERRUPT_ENABLED(vm->io_registers[INTERRUPT_FLAGS], vm->ie_register, VB_INT_FLAG_MASK)) {
				vm->cpu.programm_counter = VERTICAL_BLANK_INTERRUPT;
				vm->io_registers[INTERRUPT_FLAGS] = vm->io_registers[INTERRUPT_FLAGS] & ~VB_INT_FLAG_MASK;
			} else if (INTERRUPT_ENABLED(vm->io_registers[INTERRUPT_FLAGS], vm->ie_register, LCD_STAT_INT_FLAG_MASK)) {
				vm->cpu.programm_counter = LCD_STAT_INTERRUPT;
				vm->io_registers[INTERRUPT_FLAGS] = vm->io_registers[INTERRUPT_FLAGS] & ~LCD_STAT_INT_FLAG_MASK;
			} else if (INTERRUPT_ENABLED(vm->io_registers[INTERRUPT_FLAGS], vm->ie_register, TIMER_INT_FLAG_MASK)) {
				vm->cpu.programm_counter = TIMER_INTERRUPT;
				vm->io_registers[INTERRUPT_FLAGS] = vm->io_registers[INTERRUPT_FLAGS] & ~TIMER_INT_FLAG_MASK;
			} else if (INTERRUPT_ENABLED(vm->io_registers[INTERRUPT_FLAGS], vm->ie_register, SERIAL_INT_FLAG_MASK)) {
				vm->cpu.programm_counter = SERIAL_INTERRUPT;
				vm->io_registers[INTERRUPT_FLAGS] = vm->io_registers[INTERRUPT_FLAGS] & ~SERIAL_INT_FLAG_MASK;
			} else if (INTERRUPT_ENABLED(vm->io_registers[INTERRUPT_FLAGS], vm->ie_register, JOYPAD_INT_FLAG_MASK)) {
				vm->cpu.programm_counter = JOYPAD_INTERRUPT;
				vm->io_registers[INTERRUPT_FLAGS] = vm->io_registers[INTERRUPT_FLAGS] & ~JOYPAD_INT_FLAG_MASK;
			}

			vm->cpu.ime = 0;
			vm->cpu.wait_cycles = INTERRUPT_WAIT_CYCLE;
		}

	}
}

void pb_cpu_inst_fetch_decode(pb_vm_t* vm) {
	uint8_t opcode = pb_mem_read(vm, vm->cpu.programm_counter++);

	if (opcode == PREFIX_CB) {
		opcode = pb_mem_read(vm, vm->cpu.programm_counter++);
		vm->cpu.instr = &pb_pref_cb_opcodes[opcode];
		vm->cpu.wait_cycles += 4;
	} else {
		vm->cpu.instr = &pb_cpu_opcodes[opcode];
	}

	if (vm->cpu.instr->imm_length == 1) {
		vm->cpu.imm = pb_mem_read(vm, vm->cpu.programm_counter++);
	} else if (vm->cpu.instr->imm_length == 2) {
		vm->cpu.imm = pb_mem_read(vm, vm->cpu.programm_counter + 1) << 8 | pb_mem_read(vm, vm->cpu.programm_counter);
		vm->cpu.programm_counter += 2;
	}

	vm->cpu.wait_cycles += vm->cpu.instr->cycles - 2;
}

void pb_cpu_inst_execute(pb_vm_t* vm) {
	if (vm->cpu.instr->func != NULL) {
		//printf("%04x: %s - %04x .....stack_pointer: %02x%02x\n", vm->cpu.programm_counter, vm->cpu.instr->command, vm->cpu.imm, vm->cpu.registers[REG_SP], vm->cpu.registers[REG_SP + 1]);
		(*vm->cpu.instr->func)(vm, vm->cpu.instr);
	} else {
		exit(-1);
	}
}

int pb_cpu_cycle(pb_vm_t* vm) {
	switch(vm->cpu.cpu_state) {
		case INST_FETCH_DECODE:
			pb_cpu_interrupt_check(vm);
			if (vm->cpu.cpu_mode == NORMAL) {
				pb_cpu_inst_fetch_decode(vm);
				vm->cpu.cpu_state = CPU_WAIT;
			}
			break;
		case CPU_WAIT:
			if (--vm->cpu.wait_cycles == 0) {
				vm->cpu.cpu_state = INST_EXECUTE;
			}
			break;
		case INST_EXECUTE:
			pb_cpu_inst_execute(vm);
			vm->cpu.cpu_state = INST_FETCH_DECODE;
			break;
		default:
			break;
	}

	return 0;
}
