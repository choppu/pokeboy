/*
 * Copyright (C) 2019, Ksenia Balistreri
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the BSD license.  See the LICENSE file for details.
 */

#include <mem.h>
#include <stdlib.h>
#include "structs.h"
#include "vm.h"
#include "cpu.h"

#define TABLE_SIZE 256
#define LD_BASIC_ADDR 0xff00

#define CARRY_FLAG_MASK 		0x10
#define HALF_CARRY_FLAG_MASK 	0x20
#define SUBTRACT_FLAG_MASK 		0x40
#define ZERO_FLAG_MASK 			0x80

#define HC_FLAG_F 0xf
#define HC_FLAG_FFF 0xfff
#define HC_FLAG_CHECK_8 0x10
#define HC_FLAG_CHECK_16 0x1000
#define C_FLAG_CHECK_8 0x100
#define C_FLAG_CHECK_16 0x10000
#define C_FLAG_FF 0xff
#define C_FLAG_FFFF 0xffff

#define SET_FLAG(a, cond, flag) cond ? (a | flag) : (a & (~flag))

#define BIT0_MASK	0x1
#define BIT1_MASK	0x2
#define BIT2_MASK	0x4
#define BIT3_MASK	0x8
#define BIT4_MASK	0x10
#define BIT5_MASK	0x20
#define BIT6_MASK	0x40
#define BIT7_MASK	0x80

void pb_uint8_set_flags(pb_vm_t* vm, uint8_t a, uint8_t b, uint16_t sum, uint8_t carry) {
	vm->cpu.registers[REG_F] = SET_FLAG(vm->cpu.registers[REG_F], ((sum & 0xff) == 0), ZERO_FLAG_MASK);
	vm->cpu.registers[REG_F] &= (~SUBTRACT_FLAG_MASK);
	vm->cpu.registers[REG_F] = SET_FLAG(vm->cpu.registers[REG_F], ((((a & HC_FLAG_F) + (b & HC_FLAG_F) + carry) & HC_FLAG_CHECK_8) == HC_FLAG_CHECK_8), HALF_CARRY_FLAG_MASK);
	vm->cpu.registers[REG_F] = SET_FLAG(vm->cpu.registers[REG_F], ((sum & C_FLAG_CHECK_8) == C_FLAG_CHECK_8), CARRY_FLAG_MASK);
}

void pb_uint16_set_flags(pb_vm_t* vm, uint16_t a, uint16_t b, uint32_t sum) {
	vm->cpu.registers[REG_F] &= (~SUBTRACT_FLAG_MASK);
	vm->cpu.registers[REG_F] = SET_FLAG(vm->cpu.registers[REG_F], ((((a & HC_FLAG_FFF) + (b & HC_FLAG_FFF)) & HC_FLAG_CHECK_16) == HC_FLAG_CHECK_16), HALF_CARRY_FLAG_MASK);
	vm->cpu.registers[REG_F] = SET_FLAG(vm->cpu.registers[REG_F], ((sum & C_FLAG_CHECK_16) == C_FLAG_CHECK_16), CARRY_FLAG_MASK);
}

void pb_set_hc_flag(pb_vm_t* vm, uint8_t res) {
	vm->cpu.registers[REG_F] |= HALF_CARRY_FLAG_MASK;
	vm->cpu.registers[REG_F] &= (~CARRY_FLAG_MASK);
	vm->cpu.registers[REG_F] &= (~SUBTRACT_FLAG_MASK);
	vm->cpu.registers[REG_F] = SET_FLAG(vm->cpu.registers[REG_F], (res == 0), ZERO_FLAG_MASK);
}

void pb_set_rot_flags(pb_vm_t* vm, uint8_t data) {
	vm->cpu.registers[REG_F] = (vm->cpu.registers[REG_F] & (~CARRY_FLAG_MASK)) | (data << 4);
	vm->cpu.registers[REG_F] &= (~SUBTRACT_FLAG_MASK);
	vm->cpu.registers[REG_F] &= (~HALF_CARRY_FLAG_MASK);
}

void pb_cpu_sub_set_flags(pb_vm_t* vm, uint8_t a, uint8_t b, int16_t res, uint8_t carry) {
	int16_t half_res = (a & HC_FLAG_F) - (b & HC_FLAG_F) - carry;
	vm->cpu.registers[REG_F] = SET_FLAG(vm->cpu.registers[REG_F], ((res & 0xff) == 0), ZERO_FLAG_MASK);
	vm->cpu.registers[REG_F] |= SUBTRACT_FLAG_MASK;
	vm->cpu.registers[REG_F] = SET_FLAG(vm->cpu.registers[REG_F], (half_res < 0), HALF_CARRY_FLAG_MASK);
	vm->cpu.registers[REG_F] = SET_FLAG(vm->cpu.registers[REG_F], (res < 0), CARRY_FLAG_MASK);
}

void pb_cpu_sa_flags(pb_vm_t* vm, uint8_t data, uint8_t cf) {
	vm->cpu.registers[REG_F] = SET_FLAG(vm->cpu.registers[REG_F], data == 0, ZERO_FLAG_MASK);
	vm->cpu.registers[REG_F] &= (~SUBTRACT_FLAG_MASK);
	vm->cpu.registers[REG_F] &= (~HALF_CARRY_FLAG_MASK);
	vm->cpu.registers[REG_F] = (vm->cpu.registers[REG_F] & (~CARRY_FLAG_MASK)) | (cf << 4);
}

void pb_cpu_load_hl_sp_flags(pb_vm_t* vm, uint16_t a, uint16_t b, uint16_t res) {
	vm->cpu.registers[REG_F] &= (~ZERO_FLAG_MASK);
	vm->cpu.registers[REG_F] &= (~SUBTRACT_FLAG_MASK);
	vm->cpu.registers[REG_F] = SET_FLAG(vm->cpu.registers[REG_F], ((((a & HC_FLAG_F) + (b & HC_FLAG_F)) & HC_FLAG_CHECK_8) == HC_FLAG_CHECK_8), HALF_CARRY_FLAG_MASK);
	vm->cpu.registers[REG_F] = SET_FLAG(vm->cpu.registers[REG_F], ((((a & C_FLAG_FF) + (b & C_FLAG_FF)) & C_FLAG_CHECK_8) == C_FLAG_CHECK_8), CARRY_FLAG_MASK);
}

uint8_t pb_cpu_flag_inc(uint8_t f_reg, uint8_t a) {
	uint8_t res = a + 1;

	f_reg = SET_FLAG(f_reg, (res == 0), ZERO_FLAG_MASK);
	f_reg &= ~SUBTRACT_FLAG_MASK;
	f_reg = SET_FLAG(f_reg, ((((a & HC_FLAG_F) + (1 & HC_FLAG_F)) & HC_FLAG_CHECK_8) == HC_FLAG_CHECK_8), HALF_CARRY_FLAG_MASK);

	return f_reg;
}

uint8_t pb_cpu_dec_flags(uint8_t f_reg, uint8_t a, uint8_t b) {
	uint8_t res = a - b;

	f_reg = SET_FLAG(f_reg, (res == 0), ZERO_FLAG_MASK);
	f_reg |= SUBTRACT_FLAG_MASK;
	f_reg = SET_FLAG(f_reg, ((b & HC_FLAG_F) > (a & HC_FLAG_F)), HALF_CARRY_FLAG_MASK);

	return f_reg;
}

void pb_cpu_add_sp_flags(pb_vm_t* vm, uint8_t a, uint8_t b, uint16_t sum) {
	vm->cpu.registers[REG_F] &= (~ZERO_FLAG_MASK);
	vm->cpu.registers[REG_F] &= (~SUBTRACT_FLAG_MASK);
	vm->cpu.registers[REG_F] = SET_FLAG(vm->cpu.registers[REG_F], ((((a & HC_FLAG_F) + (b & HC_FLAG_F)) & HC_FLAG_CHECK_8) == HC_FLAG_CHECK_8), HALF_CARRY_FLAG_MASK);
	vm->cpu.registers[REG_F] = SET_FLAG(vm->cpu.registers[REG_F], ((sum & C_FLAG_CHECK_8) == C_FLAG_CHECK_8), CARRY_FLAG_MASK);
}

void pb_cpu_bit_u3_flags(pb_vm_t* vm, uint8_t res) {
	vm->cpu.registers[REG_F] = SET_FLAG(vm->cpu.registers[REG_F], (res == 0), ZERO_FLAG_MASK);
	vm->cpu.registers[REG_F] |= HALF_CARRY_FLAG_MASK;
	vm->cpu.registers[REG_F] &= (~SUBTRACT_FLAG_MASK);
}

void pb_cpu_or_a_flags(pb_vm_t* vm, uint8_t a, uint8_t b) {
	uint8_t res = a | b;

	vm->cpu.registers[REG_F] = SET_FLAG(vm->cpu.registers[REG_F], (res == 0), ZERO_FLAG_MASK);
	vm->cpu.registers[REG_F] &= (~SUBTRACT_FLAG_MASK);
	vm->cpu.registers[REG_F] &= (~HALF_CARRY_FLAG_MASK);
	vm->cpu.registers[REG_F] &= (~CARRY_FLAG_MASK);
	vm->cpu.registers[REG_A] = res;
}

uint8_t pb_cpu_swap_flags(pb_vm_t* vm, uint8_t data) {
	uint8_t upper = data >> 4;
	data = (data << 4) | upper;

	vm->cpu.registers[REG_F] = SET_FLAG(vm->cpu.registers[REG_F], data == 0, ZERO_FLAG_MASK);
	vm->cpu.registers[REG_F] &= (~SUBTRACT_FLAG_MASK);
	vm->cpu.registers[REG_F] &= (~HALF_CARRY_FLAG_MASK);
	vm->cpu.registers[REG_F] &= (~CARRY_FLAG_MASK);

	return data;
}

void pb_cpu_xor_flags(pb_vm_t* vm, uint8_t data) {
	uint8_t res = vm->cpu.registers[REG_A] ^ data;

	vm->cpu.registers[REG_F] = SET_FLAG(vm->cpu.registers[REG_F], (res == 0), ZERO_FLAG_MASK);
	vm->cpu.registers[REG_F] &= (~SUBTRACT_FLAG_MASK);
	vm->cpu.registers[REG_F] &= (~HALF_CARRY_FLAG_MASK);
	vm->cpu.registers[REG_F] &= (~CARRY_FLAG_MASK);

	vm->cpu.registers[REG_A] = res;
}

void pb_cpu_nop(pb_vm_t* vm, pb_inst_t* inst) {
	;
}

void pb_cpu_load_r8_r8(pb_vm_t* vm, pb_inst_t* inst) {
	vm->cpu.registers[inst->op1] = vm->cpu.registers[inst->op2];
}

void pb_cpu_load_r8_n8(pb_vm_t* vm, pb_inst_t* inst) {
	vm->cpu.registers[inst->op1] = vm->cpu.imm;
}

void pb_cpu_load_r16_n16(pb_vm_t* vm, pb_inst_t* inst) {
	pb_set_uint16_reg(&vm->cpu, inst->op1, vm->cpu.imm);
}

void pb_cpu_load_hl_r8(pb_vm_t* vm, pb_inst_t* inst) {
	uint16_t addr = pb_get_uint_16_reg(&vm->cpu, REG_H);
	pb_mem_write(vm, addr, vm->cpu.registers[inst->op2]);
}

void pb_cpu_load_hl_n8(pb_vm_t* vm, pb_inst_t* inst) {
	uint16_t addr = pb_get_uint_16_reg(&vm->cpu, REG_H);
	pb_mem_write(vm, addr, vm->cpu.imm);
}

void pb_cpu_load_r8_hl(pb_vm_t* vm, pb_inst_t* inst) {
	uint16_t addr = pb_get_uint_16_reg(&vm->cpu, REG_H);
	vm->cpu.registers[inst->op1] = pb_mem_read(vm, addr);
}

void pb_cpu_load_r16_a(pb_vm_t* vm, pb_inst_t* inst) {
	uint16_t addr = pb_get_uint_16_reg(&vm->cpu, inst->op1);
	pb_mem_write(vm, addr, vm->cpu.registers[REG_A]);
}

void pb_cpu_load_n16_a(pb_vm_t* vm, pb_inst_t* inst) {
	pb_mem_write(vm, vm->cpu.imm, vm->cpu.registers[REG_A]);
}

void pb_cpu_load_c_a(pb_vm_t* vm, pb_inst_t* inst) {
	uint16_t addr = LD_BASIC_ADDR | vm->cpu.registers[REG_C];
	pb_mem_write(vm, addr, vm->cpu.registers[REG_A]);
}

void pb_cpu_load_a_r16(pb_vm_t* vm, pb_inst_t* inst) {
	uint16_t addr = pb_get_uint_16_reg(&vm->cpu, inst->op2);
	vm->cpu.registers[REG_A] = pb_mem_read(vm, addr);
}

void pb_cpu_load_a_n16(pb_vm_t* vm, pb_inst_t* inst) {
	vm->cpu.registers[REG_A] = pb_mem_read(vm, vm->cpu.imm);
}

void pb_cpu_load_a_n8(pb_vm_t* vm, pb_inst_t* inst) {
	uint16_t addr = LD_BASIC_ADDR | vm->cpu.imm;
	vm->cpu.registers[REG_A] = pb_mem_read(vm, addr);
}

void pb_cpu_load_a_c(pb_vm_t* vm, pb_inst_t* inst) {
	uint16_t addr = LD_BASIC_ADDR | vm->cpu.registers[REG_C];
	vm->cpu.registers[REG_A] = pb_mem_read(vm, addr);
}

void pb_cpu_load_hl_inc_a(pb_vm_t* vm, pb_inst_t* inst) {
	uint16_t addr = pb_get_uint_16_reg(&vm->cpu, REG_H);
	pb_mem_write(vm, addr, vm->cpu.registers[REG_A]);
	pb_set_uint16_reg(&vm->cpu, REG_H, addr + 1);
}

void pb_cpu_load_hl_dec_a(pb_vm_t* vm, pb_inst_t* inst) {
	uint16_t addr = pb_get_uint_16_reg(&vm->cpu, REG_H);
	pb_mem_write(vm, addr, vm->cpu.registers[REG_A]);
	pb_set_uint16_reg(&vm->cpu, REG_H, addr - 1);
}

void pb_cpu_load_a_hl_inc(pb_vm_t* vm, pb_inst_t* inst) {
	uint16_t addr = pb_get_uint_16_reg(&vm->cpu, REG_H);
	vm->cpu.registers[REG_A] = pb_mem_read(vm, addr);
	pb_set_uint16_reg(&vm->cpu, REG_H, addr + 1);
}

void pb_cpu_load_a_hl_dec(pb_vm_t* vm, pb_inst_t* inst) {
	uint16_t addr = pb_get_uint_16_reg(&vm->cpu, REG_H);
	vm->cpu.registers[REG_A] = pb_mem_read(vm, addr);
	pb_set_uint16_reg(&vm->cpu, REG_H, addr - 1);
}

void pb_cpu_load_sp_n16(pb_vm_t* vm, pb_inst_t* inst) {
	pb_set_uint16_reg(&vm->cpu, REG_SP, vm->cpu.imm);
}

void pb_cpu_load_n16_sp(pb_vm_t* vm, pb_inst_t* inst) {
	pb_mem_write(vm, vm->cpu.imm, vm->cpu.registers[REG_SP + 1]);
	pb_mem_write(vm, vm->cpu.imm + 1, vm->cpu.registers[REG_SP]);
}

void pb_cpu_load_hl_sp(pb_vm_t* vm, pb_inst_t* inst) {
	uint16_t data = pb_get_uint_16_reg(&vm->cpu, REG_SP) + (int8_t)vm->cpu.imm;
	pb_cpu_load_hl_sp_flags(vm, pb_get_uint_16_reg(&vm->cpu, REG_SP), (int8_t)vm->cpu.imm, data);
	pb_set_uint16_reg(&vm->cpu, REG_H, data);
}

void pb_cpu_load_sp_hl(pb_vm_t* vm, pb_inst_t* inst) {
	vm->cpu.registers[REG_SP] = vm->cpu.registers[REG_H];
	vm->cpu.registers[REG_SP +1] = vm->cpu.registers[REG_L];
}

void pb_cpu_load_n8_a(pb_vm_t* vm, pb_inst_t* inst) {
	uint16_t addr = LD_BASIC_ADDR | vm->cpu.imm;
	pb_mem_write(vm, addr, vm->cpu.registers[REG_A]);
}

void pb_cpu_adc(pb_vm_t* vm, pb_inst_t* inst, uint8_t b) {
	uint16_t carry = (vm->cpu.registers[REG_F] & CARRY_FLAG_MASK) >> 4;
	uint16_t sum = vm->cpu.registers[REG_A] + b + carry;
	pb_uint8_set_flags(vm, vm->cpu.registers[REG_A], b, sum, carry);
	vm->cpu.registers[REG_A] = sum;
}

void pb_cpu_adc_a_r8(pb_vm_t* vm, pb_inst_t* inst) {
	pb_cpu_adc(vm, inst, vm->cpu.registers[inst->op2]);
}

void pb_cpu_adc_a_hl(pb_vm_t* vm, pb_inst_t* inst) {
	uint16_t addr = pb_get_uint_16_reg(&vm->cpu, REG_H);
	pb_cpu_adc(vm, inst, pb_mem_read(vm, addr));
}

void pb_cpu_adc_a_n8(pb_vm_t* vm, pb_inst_t* inst) {
	pb_cpu_adc(vm, inst, vm->cpu.imm);
}

void pb_cpu_add(pb_vm_t* vm, pb_inst_t* inst, uint8_t b) {
	uint16_t sum = vm->cpu.registers[REG_A] + b;
	pb_uint8_set_flags(vm, vm->cpu.registers[REG_A], b, sum, 0);
	vm->cpu.registers[REG_A] = sum;
}

void pb_cpu_add_a_r8(pb_vm_t* vm, pb_inst_t* inst) {
	pb_cpu_add(vm, inst, vm->cpu.registers[inst->op2]);
}

void pb_cpu_add_a_hl(pb_vm_t* vm, pb_inst_t* inst) {
	uint16_t addr = pb_get_uint_16_reg(&vm->cpu, REG_H);
	pb_cpu_add(vm, inst, pb_mem_read(vm, addr));
}

void pb_cpu_add_a_n8(pb_vm_t* vm, pb_inst_t* inst) {
	pb_cpu_add(vm, inst, vm->cpu.imm);
}

void pb_cpu_add_hl_r16(pb_vm_t* vm, pb_inst_t* inst) {
	uint16_t a = pb_get_uint_16_reg(&vm->cpu, REG_H);
	uint16_t b = pb_get_uint_16_reg(&vm->cpu, inst->op2);
	uint32_t sum = a + b;
	pb_uint16_set_flags(vm, a, b, sum);
	pb_set_uint16_reg(&vm->cpu, REG_H, sum);
}

void pb_cpu_add_sp_e8(pb_vm_t* vm, pb_inst_t* inst) {
	uint16_t a = pb_get_uint_16_reg(&vm->cpu, REG_SP);
	uint16_t sum = a + (int8_t) vm->cpu.imm;
	pb_cpu_add_sp_flags(vm, (a & 0xff), vm->cpu.imm, (a & 0xff) + vm->cpu.imm);
	pb_set_uint16_reg(&vm->cpu, REG_SP, sum);
}

void pb_cpu_and_a_r8(pb_vm_t* vm, pb_inst_t* inst) {
	uint8_t res = vm->cpu.registers[REG_A] & vm->cpu.registers[inst->op2];
	pb_set_hc_flag(vm, res);
	vm->cpu.registers[REG_A] = res;
}

void pb_cpu_and_a_hl(pb_vm_t* vm, pb_inst_t* inst) {
	uint16_t addr = pb_get_uint_16_reg(&vm->cpu, REG_H);
	uint8_t res = vm->cpu.registers[REG_A] & pb_mem_read(vm, addr);
	pb_set_hc_flag(vm, res);
	vm->cpu.registers[REG_A] = res;
}

void pb_cpu_and_a_n8(pb_vm_t* vm, pb_inst_t* inst) {
	uint8_t res = vm->cpu.registers[REG_A] & vm->cpu.imm;
	pb_set_hc_flag(vm, res);
	vm->cpu.registers[REG_A] = res;
}

void pb_cpu_bit_u3_r8(pb_vm_t* vm, pb_inst_t* inst) {
	pb_cpu_bit_u3_flags(vm, (vm->cpu.registers[inst->op1] & inst->op2));
}

void pb_cpu_bit_u3_hl(pb_vm_t* vm, pb_inst_t* inst) {
	uint16_t addr = pb_get_uint_16_reg(&vm->cpu, REG_H);
	uint8_t data = pb_mem_read(vm, addr);
	pb_cpu_bit_u3_flags(vm, (data & inst->op2));
}

void pb_cpu_call_n16(pb_vm_t* vm, pb_inst_t* inst) {
	pb_cpu_stack_push(vm, vm->cpu.programm_counter);
	vm->cpu.programm_counter = vm->cpu.imm;
}

void pb_cpu_call_cc_n16(pb_vm_t* vm, pb_inst_t* inst) {
	if ((vm->cpu.registers[REG_F] & inst->op2) == inst->op2) {
		vm->cpu.instr->cycles += 12;
		pb_cpu_call_n16(vm, inst);
	}
}

void pb_cpu_call_ncc_n16(pb_vm_t* vm, pb_inst_t* inst) {
	if ((vm->cpu.registers[REG_F] & inst->op2) != inst->op2) {
		vm->cpu.instr->cycles += 12;
		pb_cpu_call_n16(vm, inst);
	}
}

void pb_cpu_ccf(pb_vm_t* vm, pb_inst_t* inst) {
	uint8_t c_flag_mask = (~vm->cpu.registers[REG_F]) & CARRY_FLAG_MASK;
	uint8_t z_flag_mask = vm->cpu.registers[REG_F] & ZERO_FLAG_MASK;
	vm->cpu.registers[REG_F] = 0x00 | z_flag_mask | c_flag_mask;
}


void pb_cpu_cp_a_r8(pb_vm_t* vm, pb_inst_t* inst) {
	pb_cpu_sub_set_flags(vm, vm->cpu.registers[REG_A], vm->cpu.registers[inst->op2], (vm->cpu.registers[REG_A] - vm->cpu.registers[inst->op2]), 0);
}

void pb_cpu_cp_a_hl(pb_vm_t* vm, pb_inst_t* inst) {
	uint16_t addr = pb_get_uint_16_reg(&vm->cpu, REG_H);
	uint8_t data = pb_mem_read(vm, addr);
	pb_cpu_sub_set_flags(vm, vm->cpu.registers[REG_A], data, (vm->cpu.registers[REG_A] - data), 0);
}

void pb_cpu_cp_a_n8(pb_vm_t* vm, pb_inst_t* inst) {
	pb_cpu_sub_set_flags(vm, vm->cpu.registers[REG_A], vm->cpu.imm, (vm->cpu.registers[REG_A] - vm->cpu.imm), 0);
}

void pb_cpu_cpl(pb_vm_t* vm, pb_inst_t* inst) {
	vm->cpu.registers[REG_A] = ~vm->cpu.registers[REG_A];
	vm->cpu.registers[REG_F] |= HALF_CARRY_FLAG_MASK;
	vm->cpu.registers[REG_F] |= SUBTRACT_FLAG_MASK;
}

void pb_cpu_daa(pb_vm_t* vm, pb_inst_t* inst) {
	uint8_t n_flag = vm->cpu.registers[REG_F] & SUBTRACT_FLAG_MASK;
	uint8_t c_flag = vm->cpu.registers[REG_F] & CARRY_FLAG_MASK;
	uint8_t h_flag = vm->cpu.registers[REG_F] & HALF_CARRY_FLAG_MASK;
	uint8_t a = vm->cpu.registers[REG_A];

	if (!n_flag) {
		if (c_flag || a > 0x99) {
			a += 0x60;
			vm->cpu.registers[REG_F] |= CARRY_FLAG_MASK;
		}

		if (h_flag || (a & 0x0f) > 0x09) {
			a += 0x6;
		}
	} else {
		if (c_flag) {
			a -= 0x60;
		}

		if (h_flag) {
			a -= 0x6;
		}
	}

	vm->cpu.registers[REG_F] = SET_FLAG(vm->cpu.registers[REG_F], (a == 0), ZERO_FLAG_MASK);
	vm->cpu.registers[REG_F] &= ~(HALF_CARRY_FLAG_MASK);
	vm->cpu.registers[REG_A] = a;
}

void pb_cpu_dec_r8(pb_vm_t* vm, pb_inst_t* inst) {
	vm->cpu.registers[REG_F] = pb_cpu_dec_flags(vm->cpu.registers[REG_F], vm->cpu.registers[inst->op1], 1);
	vm->cpu.registers[inst->op1]--;
}

void pb_cpu_dec_hl(pb_vm_t* vm, pb_inst_t* inst) {
	uint16_t addr = pb_get_uint_16_reg(&vm->cpu, REG_H);
	uint8_t data = pb_mem_read(vm, addr);
	vm->cpu.registers[REG_F] = pb_cpu_dec_flags(vm->cpu.registers[REG_F], data, 1);
	pb_mem_write(vm, addr, data - 1);
}

void pb_cpu_dec_r16(pb_vm_t* vm, pb_inst_t* inst) {
	uint16_t r16 = pb_get_uint_16_reg(&vm->cpu, inst->op1);
	pb_set_uint16_reg(&vm->cpu, inst->op1, r16 - 1);
}

void pb_cpu_di(pb_vm_t* vm, pb_inst_t* inst) {
	vm->cpu.ime = 0;
}

void pb_cpu_ei(pb_vm_t* vm, pb_inst_t* inst) {
	vm->cpu.ime = 1;
}

void pb_cpu_halt(pb_vm_t* vm, pb_inst_t* inst) {
	vm->cpu.cpu_mode = HALT;
	/*TODO: HALT bug */
}

void pb_cpu_inc_r8(pb_vm_t* vm, pb_inst_t* inst) {
	vm->cpu.registers[REG_F] = pb_cpu_flag_inc(vm->cpu.registers[REG_F], vm->cpu.registers[inst->op1]);
	vm->cpu.registers[inst->op1]++;
}

void pb_cpu_inc_hl(pb_vm_t* vm, pb_inst_t* inst) {
	uint16_t addr = pb_get_uint_16_reg(&vm->cpu, REG_H);
	uint8_t data = pb_mem_read(vm, addr);
	vm->cpu.registers[REG_F] = pb_cpu_flag_inc(vm->cpu.registers[REG_F], data);
	pb_mem_write(vm, addr, data + 1);
}

void pb_cpu_inc_r16(pb_vm_t* vm, pb_inst_t* inst) {
	uint16_t r16 = pb_get_uint_16_reg(&vm->cpu, inst->op1);
	pb_set_uint16_reg(&vm->cpu, inst->op1, r16 + 1);
}

void pb_cpu_jp_n16(pb_vm_t* vm, pb_inst_t* inst) {
	vm->cpu.programm_counter = vm->cpu.imm;
}

void pb_cpu_jp_cc_n16(pb_vm_t* vm, pb_inst_t* inst) {
	if ((vm->cpu.registers[REG_F] & inst->op2) == inst->op2) {
		vm->cpu.programm_counter = vm->cpu.imm;
		vm->cpu.instr->cycles += 4;
	}
}

void pb_cpu_jp_ncc_n16(pb_vm_t* vm, pb_inst_t* inst) {
	if ((vm->cpu.registers[REG_F] & inst->op2) != inst->op2) {
		vm->cpu.programm_counter = vm->cpu.imm;
		vm->cpu.instr->cycles += 4;
	}
}

void pb_cpu_jp_hl(pb_vm_t* vm, pb_inst_t* inst) {
	uint16_t hl = pb_get_uint_16_reg(&vm->cpu, REG_H);
	vm->cpu.programm_counter = hl;
}

void pb_cpu_jr_e8(pb_vm_t* vm, pb_inst_t* inst) {
	vm->cpu.programm_counter += (int8_t) vm->cpu.imm;
}

void pb_cpu_jr_cc_e8(pb_vm_t* vm, pb_inst_t* inst) {
	if ((vm->cpu.registers[REG_F] & inst->op2) == inst->op2) {
		vm->cpu.programm_counter += (int8_t) vm->cpu.imm;
		vm->cpu.instr->cycles += 4;
	}
}

void pb_cpu_jr_ncc_e8(pb_vm_t* vm, pb_inst_t* inst) {
	if ((vm->cpu.registers[REG_F] & inst->op2) != inst->op2) {
		vm->cpu.programm_counter += (int8_t) vm->cpu.imm;
		vm->cpu.instr->cycles += 4;
	}
}

void pb_cpu_or_a_r8(pb_vm_t* vm, pb_inst_t* inst) {
	pb_cpu_or_a_flags(vm, vm->cpu.registers[REG_A], vm->cpu.registers[inst->op2]);
}

void pb_cpu_or_a_hl(pb_vm_t* vm, pb_inst_t* inst) {
	uint16_t addr = pb_get_uint_16_reg(&vm->cpu, REG_H);
	pb_cpu_or_a_flags(vm, vm->cpu.registers[REG_A], pb_mem_read(vm, addr));
}

void pb_cpu_or_a_n8(pb_vm_t* vm, pb_inst_t* inst) {
	pb_cpu_or_a_flags(vm, vm->cpu.registers[REG_A], vm->cpu.imm);
}

void pb_cpu_pop_r16(pb_vm_t* vm, pb_inst_t* inst) {
	uint16_t data = pb_cpu_stack_pop(vm);
	pb_set_uint16_reg(&vm->cpu, inst->op1, data);
}

void pb_cpu_pop_af(pb_vm_t* vm, pb_inst_t* inst) {
	uint16_t data = pb_cpu_stack_pop(vm);
	pb_set_uint16_reg(&vm->cpu, inst->op1, (data & 0xfff0));
}

void pb_cpu_push_r16(pb_vm_t* vm, pb_inst_t* inst) {
	uint16_t data = pb_get_uint_16_reg(&vm->cpu, inst->op1);
	pb_cpu_stack_push(vm, data);
}

void pb_cpu_res_u3_r8(pb_vm_t* vm, pb_inst_t* inst) {
	vm->cpu.registers[inst->op1] &= ~(inst->op2);
}

void pb_cpu_res_u3_hl(pb_vm_t* vm, pb_inst_t* inst) {
	uint16_t addr = pb_get_uint_16_reg(&vm->cpu, REG_H);
	uint16_t data = pb_mem_read(vm, addr);
	data &= ~(inst->op2);
	pb_mem_write(vm, addr, data);
}

void pb_cpu_ret(pb_vm_t* vm, pb_inst_t* inst) {
	uint16_t data = pb_cpu_stack_pop(vm);
	vm->cpu.programm_counter = data;
}

void pb_cpu_ret_cc(pb_vm_t* vm, pb_inst_t* inst) {
	if ((vm->cpu.registers[REG_F] & inst->op2) == inst->op2) {
		pb_cpu_ret(vm, inst);
		vm->cpu.instr->cycles += 12;
	}
}

void pb_cpu_ret_ncc(pb_vm_t* vm, pb_inst_t* inst) {
	if ((vm->cpu.registers[REG_F] & inst->op2) != inst->op2) {
		pb_cpu_ret(vm, inst);
		vm->cpu.instr->cycles += 12;
	}
}

void pb_cpu_reti(pb_vm_t* vm, pb_inst_t* inst) {
	pb_cpu_ret(vm, inst);
	vm->cpu.ime = 1;
}

uint8_t pb_cpu_rl(pb_vm_t* vm, uint8_t data) {
	uint8_t bit7 = (data & BIT7_MASK) >> 7;
	data = (data << 1) | ((vm->cpu.registers[REG_F] & CARRY_FLAG_MASK) >> 4);
	pb_set_rot_flags(vm, bit7);
	return data;
}

void pb_cpu_rl_r8(pb_vm_t* vm, pb_inst_t* inst) {
	vm->cpu.registers[inst->op1] = pb_cpu_rl(vm, vm->cpu.registers[inst->op1]);
	vm->cpu.registers[REG_F] = SET_FLAG(vm->cpu.registers[REG_F], (vm->cpu.registers[inst->op1] == 0), ZERO_FLAG_MASK);
}

void pb_cpu_rl_hl(pb_vm_t* vm, pb_inst_t* inst) {
	uint16_t addr = pb_get_uint_16_reg(&vm->cpu, REG_H);
	uint16_t data = pb_cpu_rl(vm, pb_mem_read(vm, addr));
	vm->cpu.registers[REG_F] = SET_FLAG(vm->cpu.registers[REG_F], (data == 0), ZERO_FLAG_MASK);
	pb_mem_write(vm, addr, data);

}

void pb_cpu_rla(pb_vm_t* vm, pb_inst_t* inst) {
	vm->cpu.registers[REG_A] = pb_cpu_rl(vm, vm->cpu.registers[REG_A]);
	vm->cpu.registers[REG_F] &= (~ZERO_FLAG_MASK);
}

uint8_t pb_cpu_rlc(pb_vm_t* vm, uint8_t data) {
	uint8_t bit7 = (data & BIT7_MASK) >> 7;
	data = (data << 1) | bit7;
	pb_set_rot_flags(vm, bit7);
	return data;
}

void pb_cpu_rlc_r8(pb_vm_t* vm, pb_inst_t* inst) {
	vm->cpu.registers[inst->op1] = pb_cpu_rlc(vm, vm->cpu.registers[inst->op1]);
	vm->cpu.registers[REG_F] = SET_FLAG(vm->cpu.registers[REG_F], (vm->cpu.registers[inst->op1] == 0), ZERO_FLAG_MASK);
}

void pb_cpu_rlc_hl(pb_vm_t* vm, pb_inst_t* inst) {
	uint16_t addr = pb_get_uint_16_reg(&vm->cpu, REG_H);
	uint16_t data = pb_cpu_rlc(vm, pb_mem_read(vm, addr));
	vm->cpu.registers[REG_F] = SET_FLAG(vm->cpu.registers[REG_F], (data == 0), ZERO_FLAG_MASK);
	pb_mem_write(vm, addr, data);
}

void pb_cpu_rlca(pb_vm_t* vm, pb_inst_t* inst) {
	vm->cpu.registers[REG_A] = pb_cpu_rlc(vm, vm->cpu.registers[REG_A]);
	vm->cpu.registers[REG_F] &= (~ZERO_FLAG_MASK);
}

uint8_t pb_cpu_rr(pb_vm_t* vm, uint8_t data) {
	uint8_t bit0 = data & BIT0_MASK;
	data = (data >> 1) | ((vm->cpu.registers[REG_F] & CARRY_FLAG_MASK) << 3);
	pb_set_rot_flags(vm, bit0);
	return data;
}

void pb_cpu_rr_r8(pb_vm_t* vm, pb_inst_t* inst) {
	vm->cpu.registers[inst->op1] = pb_cpu_rr(vm, vm->cpu.registers[inst->op1]);
	vm->cpu.registers[REG_F] = SET_FLAG(vm->cpu.registers[REG_F], (vm->cpu.registers[inst->op1] == 0), ZERO_FLAG_MASK);
}

void pb_cpu_rr_hl(pb_vm_t* vm, pb_inst_t* inst) {
	uint16_t addr = pb_get_uint_16_reg(&vm->cpu, REG_H);
	uint16_t data = pb_cpu_rr(vm, pb_mem_read(vm, addr));
	vm->cpu.registers[REG_F] = SET_FLAG(vm->cpu.registers[REG_F], (data == 0), ZERO_FLAG_MASK);
	pb_mem_write(vm, addr, data);
}

void pb_cpu_rra(pb_vm_t* vm, pb_inst_t* inst) {
	vm->cpu.registers[REG_A] = pb_cpu_rr(vm, vm->cpu.registers[REG_A]);
	vm->cpu.registers[REG_F] &= (~ZERO_FLAG_MASK);
}

uint8_t pb_cpu_rrc(pb_vm_t* vm, uint8_t data) {
	uint8_t bit0 = data & BIT0_MASK;
	data = (data >> 1) | (bit0 << 7);
	pb_set_rot_flags(vm, bit0);
	return data;
}

void pb_cpu_rrc_r8(pb_vm_t* vm, pb_inst_t* inst) {
	vm->cpu.registers[inst->op1] = pb_cpu_rrc(vm, vm->cpu.registers[inst->op1]);
	vm->cpu.registers[REG_F] = SET_FLAG(vm->cpu.registers[REG_F], (vm->cpu.registers[inst->op1] == 0), ZERO_FLAG_MASK);
}

void pb_cpu_rrc_hl(pb_vm_t* vm, pb_inst_t* inst) {
	uint16_t addr = pb_get_uint_16_reg(&vm->cpu, REG_H);
	uint16_t data = pb_cpu_rrc(vm, pb_mem_read(vm, addr));
	vm->cpu.registers[REG_F] = SET_FLAG(vm->cpu.registers[REG_F], (data == 0), ZERO_FLAG_MASK);
	pb_mem_write(vm, addr, data);
}

void pb_cpu_rrca(pb_vm_t* vm, pb_inst_t* inst) {
	vm->cpu.registers[REG_A] = pb_cpu_rrc(vm, vm->cpu.registers[REG_A]);
	vm->cpu.registers[REG_F] &= (~ZERO_FLAG_MASK);
}

void pb_cpu_rst_vec(pb_vm_t* vm, pb_inst_t* inst) {
	pb_cpu_stack_push(vm, vm->cpu.programm_counter);
	vm->cpu.programm_counter = inst->op2;
}

void pb_cpu_subc_a_r8(pb_vm_t* vm, pb_inst_t* inst) {
	uint8_t carry = (vm->cpu.registers[REG_F] & CARRY_FLAG_MASK) >> 4;
	int16_t res = vm->cpu.registers[REG_A] - vm->cpu.registers[inst->op2] - carry;
	pb_cpu_sub_set_flags(vm, vm->cpu.registers[REG_A], vm->cpu.registers[inst->op2], res, carry);
	vm->cpu.registers[REG_A] = res;
}

void pb_cpu_subc_a_hl(pb_vm_t* vm, pb_inst_t* inst) {
	uint8_t carry = (vm->cpu.registers[REG_F] & CARRY_FLAG_MASK) >> 4;
	uint16_t addr = pb_get_uint_16_reg(&vm->cpu, REG_H);
	uint8_t data = pb_mem_read(vm, addr);
	int16_t res = vm->cpu.registers[REG_A] - data - carry;
	pb_cpu_sub_set_flags(vm, vm->cpu.registers[REG_A], data, res, carry);
	vm->cpu.registers[REG_A] = res;
}

void pb_cpu_subc_a_n8(pb_vm_t* vm, pb_inst_t* inst) {
	uint8_t carry = (vm->cpu.registers[REG_F] & CARRY_FLAG_MASK) >> 4;
	int16_t res = vm->cpu.registers[REG_A] - vm->cpu.imm - carry;
	pb_cpu_sub_set_flags(vm, vm->cpu.registers[REG_A], vm->cpu.imm, res, carry);
	vm->cpu.registers[REG_A] = res;
}

void pb_cpu_sub_a_r8(pb_vm_t* vm, pb_inst_t* inst) {
	int16_t res = vm->cpu.registers[REG_A] - vm->cpu.registers[inst->op2];
	pb_cpu_sub_set_flags(vm, vm->cpu.registers[REG_A], vm->cpu.registers[inst->op2], res, 0);
	vm->cpu.registers[REG_A] = res;
}

void pb_cpu_sub_a_hl(pb_vm_t* vm, pb_inst_t* inst) {
	uint16_t addr = pb_get_uint_16_reg(&vm->cpu, REG_H);
	uint8_t data = pb_mem_read(vm, addr);
	int16_t res = vm->cpu.registers[REG_A] - data;
	pb_cpu_sub_set_flags(vm, vm->cpu.registers[REG_A], data, res, 0);
	vm->cpu.registers[REG_A] = res;
}

void pb_cpu_sub_a_n8(pb_vm_t* vm, pb_inst_t* inst) {
	int16_t res = vm->cpu.registers[REG_A] - vm->cpu.imm;
	pb_cpu_sub_set_flags(vm, vm->cpu.registers[REG_A], vm->cpu.imm, res, 0);
	vm->cpu.registers[REG_A] = res;
}

void pb_cpu_scf(pb_vm_t* vm, pb_inst_t* inst) {
	vm->cpu.registers[REG_F] &= (~SUBTRACT_FLAG_MASK);
	vm->cpu.registers[REG_F] &= (~HALF_CARRY_FLAG_MASK);
	vm->cpu.registers[REG_F] |= CARRY_FLAG_MASK;
}

void pb_cpu_set_u3_r8(pb_vm_t* vm, pb_inst_t* inst) {
	vm->cpu.registers[inst->op1] |= inst->op2;
}

void pb_cpu_set_u3_hl(pb_vm_t* vm, pb_inst_t* inst) {
	uint16_t addr = pb_get_uint_16_reg(&vm->cpu, REG_H);
	uint8_t data = pb_mem_read(vm, addr) | inst->op2;
	pb_mem_write(vm, addr, data);
}

uint8_t pb_cpu_sla(pb_vm_t* vm, uint8_t data) {
	uint8_t bit7 = (data & BIT7_MASK) >> 7;
	data <<= 1;
	pb_cpu_sa_flags(vm, data, bit7);
	return data;
}

void pb_cpu_sla_r8(pb_vm_t* vm, pb_inst_t* inst) {
	vm->cpu.registers[inst->op1] = pb_cpu_sla(vm, vm->cpu.registers[inst->op1]);
}

void pb_cpu_sla_hl(pb_vm_t* vm, pb_inst_t* inst) {
	uint16_t addr = pb_get_uint_16_reg(&vm->cpu, REG_H);
	uint8_t data = pb_cpu_sla(vm, pb_mem_read(vm, addr));
	pb_mem_write(vm, addr, data);
}

uint8_t pb_cpu_sra(pb_vm_t* vm, uint8_t data) {
	uint8_t bit0 = data & BIT0_MASK;
	uint8_t bit7 = data & BIT7_MASK;
	data >>= 1;
	data |= bit7;
	pb_cpu_sa_flags(vm, data, bit0);
	return data;
}

void pb_cpu_sra_r8(pb_vm_t* vm, pb_inst_t* inst) {
	vm->cpu.registers[inst->op1] = pb_cpu_sra(vm, vm->cpu.registers[inst->op1]);
}

void pb_cpu_sra_hl(pb_vm_t* vm, pb_inst_t* inst) {
	uint16_t addr = pb_get_uint_16_reg(&vm->cpu, REG_H);
	uint8_t data = pb_cpu_sra(vm, pb_mem_read(vm, addr));
	pb_mem_write(vm, addr, data);
}

uint8_t pb_cpu_srl(pb_vm_t* vm, uint8_t data) {
	uint8_t bit0 = data & BIT0_MASK;
	data >>= 1;
	pb_cpu_sa_flags(vm, data, bit0);
	return data;
}

void pb_cpu_srl_r8(pb_vm_t* vm, pb_inst_t* inst) {
	vm->cpu.registers[inst->op1] = pb_cpu_srl(vm, vm->cpu.registers[inst->op1]);
}

void pb_cpu_srl_hl(pb_vm_t* vm, pb_inst_t* inst) {
	uint16_t addr = pb_get_uint_16_reg(&vm->cpu, REG_H);
	uint8_t data = pb_cpu_srl(vm, pb_mem_read(vm, addr));
	pb_mem_write(vm, addr, data);
}

void pb_cpu_stop(pb_vm_t* vm, pb_inst_t* inst) {
	vm->cpu.cpu_mode = STOP;
}

void pb_cpu_swap_r8(pb_vm_t* vm, pb_inst_t* inst) {
	vm->cpu.registers[inst->op1] = pb_cpu_swap_flags(vm, vm->cpu.registers[inst->op1]);
}

void pb_cpu_swap_hl(pb_vm_t* vm, pb_inst_t* inst) {
	uint16_t addr = pb_get_uint_16_reg(&vm->cpu, REG_H);
	uint8_t data = pb_cpu_swap_flags(vm, pb_mem_read(vm, addr));
	pb_mem_write(vm, addr, data);
}

void pb_cpu_xor_a_r8(pb_vm_t* vm, pb_inst_t* inst) {
	pb_cpu_xor_flags(vm, vm->cpu.registers[inst->op2]);
}

void pb_cpu_xor_a_hl(pb_vm_t* vm, pb_inst_t* inst) {
	uint16_t addr = pb_get_uint_16_reg(&vm->cpu, REG_H);
	pb_cpu_xor_flags(vm, pb_mem_read(vm, addr));
}

void pb_cpu_xor_a_n8(pb_vm_t* vm, pb_inst_t* inst) {
	pb_cpu_xor_flags(vm, vm->cpu.imm);
}


pb_inst_t pb_cpu_opcodes[TABLE_SIZE] = {
		{pb_cpu_nop, "NOP", 4, 0, 0, 0},
		{pb_cpu_load_r16_n16, "LD BC,d16", 12, REG_B, 0, 2},
		{pb_cpu_load_r16_a, "LD (BC),A", 8, REG_B, REG_A, 0},
		{pb_cpu_inc_r16, "INC BC", 8, REG_B, 0, 0},
		{pb_cpu_inc_r8, "INC B", 4, REG_B, 0, 0},
		{pb_cpu_dec_r8, "DEC B", 4, REG_B, 0, 0},
		{pb_cpu_load_r8_n8, "LD B,d8", 8, REG_B, 0, 1},
		{pb_cpu_rlca, "RLCA", 4, 0, 0, 0},
		{pb_cpu_load_n16_sp, "LD (a16),SP", 20, 0, REG_SP, 2},
		{pb_cpu_add_hl_r16, "ADD HL,BC", 8, REG_H, REG_B, 0},
		{pb_cpu_load_a_r16, "LD A,(BC)", 8, REG_A, REG_B, 0},
		{pb_cpu_dec_r16, "DEC BC", 8, REG_B, 0, 0},
		{pb_cpu_inc_r8, "INC C", 4, REG_C, 0, 0},
		{pb_cpu_dec_r8, "DEC C", 4, REG_C, 0, 0},
		{pb_cpu_load_r8_n8, "LD C,d8", 8, REG_C, 0, 1},
		{pb_cpu_rrca, "RRCA", 4, 0, 0, 0},
		{pb_cpu_stop, "STOP 0", 4, 0, 0, 1},
		{pb_cpu_load_r16_n16, "LD DE,d16", 12, REG_D, 0, 2},
		{pb_cpu_load_r16_a, "LD (DE),A", 8, REG_D, REG_A, 0},
		{pb_cpu_inc_r16, "INC DE", 8, REG_D, 0, 0},
		{pb_cpu_inc_r8, "INC D", 4, REG_D, 0, 0},
		{pb_cpu_dec_r8, "DEC D", 4, REG_D, 0, 0},
		{pb_cpu_load_r8_n8, "LD D,d8", 8, REG_D, 0, 1},
		{pb_cpu_rla, "RLA", 4, 0, 0, 0},
		{pb_cpu_jr_e8, "JR r8", 12, 0, 0, 1},
		{pb_cpu_add_hl_r16, "ADD HL,DE", 8, REG_H, REG_D, 0},
		{pb_cpu_load_a_r16, "LD A,(DE)", 8, REG_A, REG_D, 0},
		{pb_cpu_dec_r16, "DEC DE", 8, REG_D, 0, 0},
		{pb_cpu_inc_r8, "INC E", 4, REG_E, 0, 0},
		{pb_cpu_dec_r8, "DEC E", 4, REG_E, 0, 0},
		{pb_cpu_load_r8_n8, "LD E,d8", 8, REG_E, 0, 1},
		{pb_cpu_rra, "RRA", 4, 0, 0, 0},
		{pb_cpu_jr_ncc_e8, "JR NZ,r8", 8, 0, ZERO_FLAG_MASK, 1},
		{pb_cpu_load_r16_n16, "LD HL,d16", 12, REG_H, 0, 2},
		{pb_cpu_load_hl_inc_a, "LD (HL+),A", 8, REG_H, REG_A, 0},
		{pb_cpu_inc_r16, "INC HL", 8, REG_H, 0, 0},
		{pb_cpu_inc_r8, "INC H", 4, REG_H, 0, 0},
		{pb_cpu_dec_r8, "DEC H", 4, REG_H, 0, 0},
		{pb_cpu_load_r8_n8, "LD H,d8", 8, REG_H, 0, 1},
		{pb_cpu_daa, "DAA", 4, 0, 0, 0},
		{pb_cpu_jr_cc_e8, "JR Z,r8", 8, 0, ZERO_FLAG_MASK, 1},
		{pb_cpu_add_hl_r16, "ADD HL,HL", 8, REG_H, REG_H, 0},
		{pb_cpu_load_a_hl_inc, "LD A,(HL+)", 8, REG_A, REG_H, 0},
		{pb_cpu_dec_r16, "DEC HL", 8, REG_H, 0, 0},
		{pb_cpu_inc_r8, "INC L", 4, REG_L, 0, 0},
		{pb_cpu_dec_r8, "DEC L", 4, REG_L, 0, 0},
		{pb_cpu_load_r8_n8, "LD L,d8", 8, REG_L, 0, 1},
		{pb_cpu_cpl, "CPL", 4, 0, 0, 0},
		{pb_cpu_jr_ncc_e8, "JR NC,r8", 8, 0, CARRY_FLAG_MASK, 1},
		{pb_cpu_load_sp_n16, "LD SP,d16", 12, REG_SP, 0, 2},
		{pb_cpu_load_hl_dec_a, "LD (HL-),A", 8, REG_H, REG_A, 0},
		{pb_cpu_inc_r16, "INC SP", 8, REG_SP, 0, 0},
		{pb_cpu_inc_hl, "INC (HL)", 12, REG_H, 0, 0},
		{pb_cpu_dec_hl, "DEC (HL)", 12, REG_H, 0, 0},
		{pb_cpu_load_hl_n8, "LD (HL),d8", 12, REG_H, 0, 1},
		{pb_cpu_scf, "SCF", 4, 0, 0, 0},
		{pb_cpu_jr_cc_e8, "JR C,r8", 8, 0, CARRY_FLAG_MASK, 1},
		{pb_cpu_add_hl_r16, "ADD HL,SP", 8, REG_H, REG_SP, 0},
		{pb_cpu_load_a_hl_dec, "LD A,(HL-)", 8, REG_A, REG_H, 0},
		{pb_cpu_dec_r16, "DEC SP", 8, REG_SP, 0, 0},
		{pb_cpu_inc_r8, "INC A", 4, REG_A, 0, 0},
		{pb_cpu_dec_r8, "DEC A", 4, REG_A, 0, 0},
		{pb_cpu_load_r8_n8, "LD A,d8", 8, REG_A, 0, 1},
		{pb_cpu_ccf, "CCF", 4, 0, 0, 0},
		{pb_cpu_load_r8_r8, "LD B,B", 4, REG_B, REG_B, 0},
		{pb_cpu_load_r8_r8, "LD B,C", 4, REG_B, REG_C, 0},
		{pb_cpu_load_r8_r8, "LD B,D", 4, REG_B, REG_D, 0},
		{pb_cpu_load_r8_r8, "LD B,E", 4, REG_B, REG_E, 0},
		{pb_cpu_load_r8_r8, "LD B,H", 4, REG_B, REG_H, 0},
		{pb_cpu_load_r8_r8, "LD B,L", 4, REG_B, REG_L, 0},
		{pb_cpu_load_r8_hl, "LD B,(HL)", 8, REG_B, REG_H, 0},
		{pb_cpu_load_r8_r8, "LD B,A", 4, REG_B, REG_A, 0},
		{pb_cpu_load_r8_r8, "LD C,B", 4, REG_C, REG_B, 0},
		{pb_cpu_load_r8_r8, "LD C,C", 4, REG_C, REG_C, 0},
		{pb_cpu_load_r8_r8, "LD C,D", 4, REG_C, REG_D, 0},
		{pb_cpu_load_r8_r8, "LD C,E", 4, REG_C, REG_E, 0},
		{pb_cpu_load_r8_r8, "LD C,H", 4, REG_C, REG_H, 0},
		{pb_cpu_load_r8_r8, "LD C,L", 4, REG_C, REG_L, 0},
		{pb_cpu_load_r8_hl, "LD C,(HL)", 8, REG_C, REG_H, 0},
		{pb_cpu_load_r8_r8, "LD C,A", 4, REG_C, REG_A, 0},
		{pb_cpu_load_r8_r8, "LD D,B", 4, REG_D, REG_B, 0},
		{pb_cpu_load_r8_r8, "LD D,C", 4, REG_D, REG_C, 0},
		{pb_cpu_load_r8_r8, "LD D,D", 4, REG_D, REG_D, 0},
		{pb_cpu_load_r8_r8, "LD D,E", 4, REG_D, REG_E, 0},
		{pb_cpu_load_r8_r8, "LD D,H", 4, REG_D, REG_H, 0},
		{pb_cpu_load_r8_r8, "LD D,L", 4, REG_D, REG_L, 0},
		{pb_cpu_load_r8_hl, "LD D,(HL)", 8, REG_D, REG_H, 0},
		{pb_cpu_load_r8_r8, "LD D,A", 4, REG_D, REG_A, 0},
		{pb_cpu_load_r8_r8, "LD E,B", 4, REG_E, REG_B, 0},
		{pb_cpu_load_r8_r8, "LD E,C", 4, REG_E, REG_C, 0},
		{pb_cpu_load_r8_r8, "LD E,D", 4, REG_E, REG_D, 0},
		{pb_cpu_load_r8_r8, "LD E,E", 4, REG_E, REG_E, 0},
		{pb_cpu_load_r8_r8, "LD E,H", 4, REG_E, REG_H, 0},
		{pb_cpu_load_r8_r8, "LD E,L", 4, REG_E, REG_L, 0},
		{pb_cpu_load_r8_hl, "LD E,(HL)", 8, REG_E, REG_H, 0},
		{pb_cpu_load_r8_r8, "LD E,A", 4, REG_E, REG_A, 0},
		{pb_cpu_load_r8_r8, "LD H,B", 4, REG_H, REG_B, 0},
		{pb_cpu_load_r8_r8, "LD H,C", 4, REG_H, REG_C, 0},
		{pb_cpu_load_r8_r8, "LD H,D", 4, REG_H, REG_D, 0},
		{pb_cpu_load_r8_r8, "LD H,E", 4, REG_H, REG_E, 0},
		{pb_cpu_load_r8_r8, "LD H,H", 4, REG_H, REG_H, 0},
		{pb_cpu_load_r8_r8, "LD H,L", 4, REG_H, REG_L, 0},
		{pb_cpu_load_r8_hl, "LD H,(HL)", 8, REG_H, REG_H, 0},
		{pb_cpu_load_r8_r8, "LD H,A", 4, REG_H, REG_A, 0},
		{pb_cpu_load_r8_r8, "LD L,B", 4, REG_L, REG_B, 0},
		{pb_cpu_load_r8_r8, "LD L,C", 4, REG_L, REG_C, 0},
		{pb_cpu_load_r8_r8, "LD L,D", 4, REG_L, REG_D, 0},
		{pb_cpu_load_r8_r8, "LD L,E", 4, REG_L, REG_E, 0},
		{pb_cpu_load_r8_r8, "LD L,H", 4, REG_L, REG_H, 0},
		{pb_cpu_load_r8_r8, "LD L,L", 4, REG_L, REG_L, 0},
		{pb_cpu_load_r8_hl, "LD L,(HL)", 8, REG_L, REG_H, 0},
		{pb_cpu_load_r8_r8, "LD L,A", 4, REG_L, REG_A, 0},
		{pb_cpu_load_hl_r8, "LD (HL),B", 8, REG_H, REG_B, 0},
		{pb_cpu_load_hl_r8, "LD (HL),C", 8, REG_H, REG_C, 0},
		{pb_cpu_load_hl_r8, "LD (HL),D", 8, REG_H, REG_D, 0},
		{pb_cpu_load_hl_r8, "LD (HL),E", 8, REG_H, REG_E, 0},
		{pb_cpu_load_hl_r8, "LD (HL),H", 8, REG_H, REG_H, 0},
		{pb_cpu_load_hl_r8, "LD (HL),L", 8, REG_H, REG_L, 0},
		{pb_cpu_halt, "HALT", 4, 0, 0, 0},
		{pb_cpu_load_hl_r8, "LD (HL),A", 8, REG_H, REG_A, 0},
		{pb_cpu_load_r8_r8, "LD A,B", 4, REG_A, REG_B, 0},
		{pb_cpu_load_r8_r8, "LD A,C", 4, REG_A, REG_C, 0},
		{pb_cpu_load_r8_r8, "LD A,D", 4, REG_A, REG_D, 0},
		{pb_cpu_load_r8_r8, "LD A,E", 4, REG_A, REG_E, 0},
		{pb_cpu_load_r8_r8, "LD A,H", 4, REG_A, REG_H, 0},
		{pb_cpu_load_r8_r8, "LD A,L", 4, REG_A, REG_L, 0},
		{pb_cpu_load_r8_hl, "LD A,(HL)", 8, REG_A, REG_H, 0},
		{pb_cpu_load_r8_r8, "LD A,A", 4, REG_A, REG_A, 0},
		{pb_cpu_add_a_r8, "ADD A,B", 4, REG_A, REG_B, 0},
		{pb_cpu_add_a_r8, "ADD A,C", 4, REG_A, REG_C, 0},
		{pb_cpu_add_a_r8, "ADD A,D", 4, REG_A, REG_D, 0},
		{pb_cpu_add_a_r8, "ADD A,E", 4, REG_A, REG_E, 0},
		{pb_cpu_add_a_r8, "ADD A,H", 4, REG_A, REG_H, 0},
		{pb_cpu_add_a_r8, "ADD A,L", 4, REG_A, REG_L, 0},
		{pb_cpu_add_a_hl, "ADD A,(HL)", 8, REG_A, REG_H, 0},
		{pb_cpu_add_a_r8, "ADD A,A", 4, REG_A, REG_A, 0},
		{pb_cpu_adc_a_r8, "ADC A,B", 4, REG_A, REG_B, 0},
		{pb_cpu_adc_a_r8, "ADC A,C", 4, REG_A, REG_C, 0},
		{pb_cpu_adc_a_r8, "ADC A,D", 4, REG_A, REG_D, 0},
		{pb_cpu_adc_a_r8, "ADC A,E", 4, REG_A, REG_E, 0},
		{pb_cpu_adc_a_r8, "ADC A,H", 4, REG_A, REG_H, 0},
		{pb_cpu_adc_a_r8, "ADC A,L", 4, REG_A, REG_L, 0},
		{pb_cpu_adc_a_hl, "ADC A,(HL)", 8, REG_A, REG_H, 0},
		{pb_cpu_adc_a_r8, "ADC A,A", 4, REG_A, REG_A, 0},
		{pb_cpu_sub_a_r8, "SUB B", 4, REG_A, REG_B, 0},
		{pb_cpu_sub_a_r8, "SUB C", 4, REG_A, REG_C, 0},
		{pb_cpu_sub_a_r8, "SUB D", 4, REG_A, REG_D, 0},
		{pb_cpu_sub_a_r8, "SUB E", 4, REG_A, REG_E, 0},
		{pb_cpu_sub_a_r8, "SUB H", 4, REG_A, REG_H, 0},
		{pb_cpu_sub_a_r8, "SUB L", 4, REG_A, REG_L, 0},
		{pb_cpu_sub_a_hl, "SUB (HL)", 8, REG_A, REG_H, 0},
		{pb_cpu_sub_a_r8, "SUB A", 4, REG_A, REG_A, 0},
		{pb_cpu_subc_a_r8, "SUBC B", 4, REG_A, REG_B, 0},
		{pb_cpu_subc_a_r8, "SUBC C", 4, REG_A, REG_C, 0},
		{pb_cpu_subc_a_r8, "SUBC D", 4, REG_A, REG_D, 0},
		{pb_cpu_subc_a_r8, "SUBC E", 4, REG_A, REG_E, 0},
		{pb_cpu_subc_a_r8, "SUBC H", 4, REG_A, REG_H, 0},
		{pb_cpu_subc_a_r8, "SUBC L", 4, REG_A, REG_L, 0},
		{pb_cpu_subc_a_hl, "SUBC (HL)", 8, REG_A, REG_H, 0},
		{pb_cpu_subc_a_r8, "SUBC A", 4, REG_A, REG_A, 0},
		{pb_cpu_and_a_r8, "AND B", 4, REG_A, REG_B, 0},
		{pb_cpu_and_a_r8, "AND C", 4, REG_A, REG_C, 0},
		{pb_cpu_and_a_r8, "AND D", 4, REG_A, REG_D, 0},
		{pb_cpu_and_a_r8, "AND E", 4, REG_A, REG_E, 0},
		{pb_cpu_and_a_r8, "AND H", 4, REG_A, REG_H, 0},
		{pb_cpu_and_a_r8, "AND L", 4, REG_A, REG_L, 0},
		{pb_cpu_and_a_hl, "AND (HL)", 8, REG_A, REG_H, 0},
		{pb_cpu_and_a_r8, "AND A", 4, REG_A, REG_A, 0},
		{pb_cpu_xor_a_r8, "XOR B", 4, REG_A, REG_B, 0},
		{pb_cpu_xor_a_r8, "XOR C", 4, REG_A, REG_C, 0},
		{pb_cpu_xor_a_r8, "XOR D", 4, REG_A, REG_D, 0},
		{pb_cpu_xor_a_r8, "XOR E", 4, REG_A, REG_E, 0},
		{pb_cpu_xor_a_r8, "XOR H", 4, REG_A, REG_H, 0},
		{pb_cpu_xor_a_r8, "XOR L", 4, REG_A, REG_L, 0},
		{pb_cpu_xor_a_hl, "XOR (HL)", 8, REG_A, REG_H, 0},
		{pb_cpu_xor_a_r8, "XOR A", 4, REG_A, REG_A, 0},
		{pb_cpu_or_a_r8, "OR B", 4, REG_A, REG_B, 0},
		{pb_cpu_or_a_r8, "OR C", 4, REG_A, REG_C, 0},
		{pb_cpu_or_a_r8, "OR D", 4, REG_A, REG_D, 0},
		{pb_cpu_or_a_r8, "OR E", 4, REG_A, REG_E, 0},
		{pb_cpu_or_a_r8, "OR H", 4, REG_A, REG_H, 0},
		{pb_cpu_or_a_r8, "OR L", 4, REG_A, REG_L, 0},
		{pb_cpu_or_a_hl, "OR (HL)", 8, REG_A, REG_H, 0},
		{pb_cpu_or_a_r8, "OR A", 4, REG_A, REG_A, 0},
		{pb_cpu_cp_a_r8, "CP B", 4, REG_A, REG_B, 0},
		{pb_cpu_cp_a_r8, "CP C", 4, REG_A, REG_C, 0},
		{pb_cpu_cp_a_r8, "CP D", 4, REG_A, REG_D, 0},
		{pb_cpu_cp_a_r8, "CP E", 4, REG_A, REG_E, 0},
		{pb_cpu_cp_a_r8, "CP H", 4, REG_A, REG_H, 0},
		{pb_cpu_cp_a_r8, "CP L", 4, REG_A, REG_L, 0},
		{pb_cpu_cp_a_hl, "CP (HL)", 8, REG_A, REG_H, 0},
		{pb_cpu_cp_a_r8, "CP A", 4, REG_A, REG_A, 0},
		{pb_cpu_ret_ncc, "RET NZ", 8, 0, ZERO_FLAG_MASK, 0},
		{pb_cpu_pop_r16, "POP BC", 12, REG_B, 0, 0},
		{pb_cpu_jp_ncc_n16, "JP NZ,a16", 12, REG_F, ZERO_FLAG_MASK, 2},
		{pb_cpu_jp_n16, "JP a16", 16, 0, 0, 2},
		{pb_cpu_call_ncc_n16, "CALL NZ,a16", 12, REG_F, ZERO_FLAG_MASK, 2},
		{pb_cpu_push_r16, "PUSH BC", 16, REG_B, 0, 0},
		{pb_cpu_add_a_n8, "ADD A,d8", 8, REG_A, 0, 1},
		{pb_cpu_rst_vec, "RST 00H", 16, 0, 0x00, 0},
		{pb_cpu_ret_cc, "RET Z", 8, 0, ZERO_FLAG_MASK, 0},
		{pb_cpu_ret, "RET", 16, 0, 0, 0},
		{pb_cpu_jp_cc_n16, "JP Z,a16", 12, REG_F, ZERO_FLAG_MASK, 2},
		{NULL, "PREFIX CB", 4, 0, 0, 0},
		{pb_cpu_call_cc_n16, "CALL Z,a16", 12, REG_F, ZERO_FLAG_MASK, 2},
		{pb_cpu_call_n16, "CALL a16", 24, 0, 0, 2},
		{pb_cpu_adc_a_n8, "ADC A,d8", 8, REG_A, 0, 1},
		{pb_cpu_rst_vec, "RST 08H", 16, 0, 0x08, 0},
		{pb_cpu_ret_ncc, "RET NC", 8, 0, CARRY_FLAG_MASK, 0},
		{pb_cpu_pop_r16, "POP DE", 12, REG_D, 0, 0},
		{pb_cpu_jp_ncc_n16, "JP NC,a16", 12, REG_F, CARRY_FLAG_MASK, 2},
		{NULL, "", 0, 0, 0, 0},
		{pb_cpu_call_ncc_n16, "CALL NC,a16", 12, REG_F, CARRY_FLAG_MASK, 2},
		{pb_cpu_push_r16, "PUSH DE", 16, REG_D, 0, 0},
		{pb_cpu_sub_a_n8, "SUB d8", 8, REG_A, 0, 1},
		{pb_cpu_rst_vec, "RST 10H", 16, 0, 0x10, 0},
		{pb_cpu_ret_cc, "RET C", 8, 0, CARRY_FLAG_MASK, 0},
		{pb_cpu_reti, "RETI", 16, 0, 0, 0},
		{pb_cpu_jp_cc_n16, "JP C,a16", 12, REG_F, CARRY_FLAG_MASK, 2},
		{NULL, "", 0, 0, 0, 0},
		{pb_cpu_call_cc_n16, "CALL C,a16", 12, REG_F, CARRY_FLAG_MASK, 2},
		{NULL, "", 0, 0, 0, 0},
		{pb_cpu_subc_a_n8, "SBC A,d8", 8, REG_A, 0, 1},
		{pb_cpu_rst_vec, "RST 18H", 16, 0, 0x18, 0},
		{pb_cpu_load_n8_a, "LDH (a8),A", 12, 0, REG_A, 1},
		{pb_cpu_pop_r16, "POP HL", 12, REG_H, 0, 0},
		{pb_cpu_load_c_a, "LD (C),A", 8, 0, REG_A, 0},
		{NULL, "", 0, 0, 0, 0},
		{NULL, "", 0, 0, 0, 0},
		{pb_cpu_push_r16, "PUSH HL", 16, REG_H, 0, 0},
		{pb_cpu_and_a_n8, "AND d8", 8, REG_A, 0, 1},
		{pb_cpu_rst_vec, "RST 20H", 16, 0, 0x20, 0},
		{pb_cpu_add_sp_e8, "ADD SP,r8", 16, REG_SP, 0, 1},
		{pb_cpu_jp_hl, "JP (HL)", 4, 0, 0, 0},
		{pb_cpu_load_n16_a, "LD (a16),A", 16, REG_A, 0, 2},
		{NULL, "", 0, 0, 0, 0},
		{NULL, "", 0, 0, 0, 0},
		{NULL, "", 0, 0, 0, 0},
		{pb_cpu_xor_a_n8, "XOR d8", 8, REG_A, 0, 1},
		{pb_cpu_rst_vec, "RST 28H", 16, 0, 0x28, 0},
		{pb_cpu_load_a_n8, "LDH A,(a8)", 12, REG_A, 0, 1},
		{pb_cpu_pop_af, "POP AF", 12, REG_A, 0, 0},
		{pb_cpu_load_a_c, "LD A,(C)", 8, REG_A, 0, 0},
		{pb_cpu_di, "DI", 4, 0, 0, 0},
		{NULL, "", 0, 0, 0, 0},
		{pb_cpu_push_r16, "PUSH AF", 16, REG_A, 0, 0},
		{pb_cpu_or_a_n8, "OR d8", 8, REG_A, 0, 1},
		{pb_cpu_rst_vec, "RST 30H", 16, 0, 0x30, 0},
		{pb_cpu_load_hl_sp, "LD HL,SP+r8", 12, REG_H, REG_SP, 1},
		{pb_cpu_load_sp_hl, "LD SP,HL", 8, REG_SP, REG_H, 0},
		{pb_cpu_load_a_n16, "LD A,(a16)", 16, REG_A, 0, 2},
		{pb_cpu_ei, "EI", 4, 0, 0, 0},
		{NULL, "", 0, 0, 0, 0},
		{NULL, "", 0, 0, 0, 0},
		{pb_cpu_cp_a_n8, "CP d8", 8, REG_A, 0, 1},
		{pb_cpu_rst_vec, "RST 38H", 16, 0, 0x38, 0}
};

pb_inst_t pb_pref_cb_opcodes[TABLE_SIZE] = {
		{pb_cpu_rlc_r8, "RLC B", 4, REG_B, 0, 0},
		{pb_cpu_rlc_r8, "RLC C", 4, REG_C, 0, 0},
		{pb_cpu_rlc_r8, "RLC D", 4, REG_D, 0, 0},
		{pb_cpu_rlc_r8, "RLC E", 4, REG_E, 0, 0},
		{pb_cpu_rlc_r8, "RLC H", 4, REG_H, 0, 0},
		{pb_cpu_rlc_r8, "RLC L", 4, REG_L, 0, 0},
		{pb_cpu_rlc_hl, "RLC (HL)", 12, REG_H, 0, 0},
		{pb_cpu_rlc_r8, "RLC A", 4, REG_A, 0, 0},
		{pb_cpu_rrc_r8, "RRC B", 4, REG_B, 0, 0},
		{pb_cpu_rrc_r8, "RRC C", 4, REG_C, 0, 0},
		{pb_cpu_rrc_r8, "RRC D", 4, REG_D, 0, 0},
		{pb_cpu_rrc_r8, "RRC E", 4, REG_E, 0, 0},
		{pb_cpu_rrc_r8, "RRC H", 4, REG_H, 0, 0},
		{pb_cpu_rrc_r8, "RRC L", 4, REG_L, 0, 0},
		{pb_cpu_rrc_hl, "RRC (HL)", 12, REG_H, 0, 0},
		{pb_cpu_rrc_r8, "RRC A", 4, REG_A, 0, 0},
		{pb_cpu_rl_r8, "RL B", 4, REG_B, 0, 0},
		{pb_cpu_rl_r8, "RL C", 4, REG_C, 0, 0},
		{pb_cpu_rl_r8, "RL D", 4, REG_D, 0, 0},
		{pb_cpu_rl_r8, "RL E", 4, REG_E, 0, 0},
		{pb_cpu_rl_r8, "RL H", 4, REG_H, 0, 0},
		{pb_cpu_rl_r8, "RL L", 4, REG_L, 0, 0},
		{pb_cpu_rl_hl, "RL (HL)", 12, REG_H, 0, 0},
		{pb_cpu_rl_r8, "RL A", 4, REG_A, 0, 0},
		{pb_cpu_rr_r8, "RR B", 4, REG_B, 0, 0},
		{pb_cpu_rr_r8, "RR C", 4, REG_C, 0, 0},
		{pb_cpu_rr_r8, "RR D", 4, REG_D, 0, 0},
		{pb_cpu_rr_r8, "RR E", 4, REG_E, 0, 0},
		{pb_cpu_rr_r8, "RR H", 4, REG_H, 0, 0},
		{pb_cpu_rr_r8, "RR L", 4, REG_L, 0, 0},
		{pb_cpu_rr_hl, "RR (HL)", 12, REG_H, 0, 0},
		{pb_cpu_rr_r8, "RR A", 4, REG_A, 0, 0},
		{pb_cpu_sla_r8, "SLA B", 4, REG_B, 0, 0},
		{pb_cpu_sla_r8, "SLA C", 4, REG_C, 0, 0},
		{pb_cpu_sla_r8, "SLA D", 4, REG_D, 0, 0},
		{pb_cpu_sla_r8, "SLA E", 4, REG_E, 0, 0},
		{pb_cpu_sla_r8, "SLA H", 4, REG_H, 0, 0},
		{pb_cpu_sla_r8, "SLA L", 4, REG_L, 0, 0},
		{pb_cpu_sla_hl, "SLA (HL)", 12, REG_H, 0, 0},
		{pb_cpu_sla_r8, "SLA A", 4, REG_A, 0, 0},
		{pb_cpu_sra_r8, "SRA B", 4, REG_B, 0, 0},
		{pb_cpu_sra_r8, "SRA C", 4, REG_C, 0, 0},
		{pb_cpu_sra_r8, "SRA D", 4, REG_D, 0, 0},
		{pb_cpu_sra_r8, "SRA E", 4, REG_E, 0, 0},
		{pb_cpu_sra_r8, "SRA H", 4, REG_H, 0, 0},
		{pb_cpu_sra_r8, "SRA L", 4, REG_L, 0, 0},
		{pb_cpu_sra_hl, "SRA (HL)", 12, REG_H, 0, 0},
		{pb_cpu_sra_r8, "SRA A", 4, REG_A, 0, 0},
		{pb_cpu_swap_r8, "SWAP B", 4, REG_B, 0, 0},
		{pb_cpu_swap_r8, "SWAP C", 4, REG_C, 0, 0},
		{pb_cpu_swap_r8, "SWAP D", 4, REG_D, 0, 0},
		{pb_cpu_swap_r8, "SWAP E", 4, REG_E, 0, 0},
		{pb_cpu_swap_r8, "SWAP H", 4, REG_H, 0, 0},
		{pb_cpu_swap_r8, "SWAP L", 4, REG_L, 0, 0},
		{pb_cpu_swap_hl, "SWAP (HL)", 12, REG_H, 0, 0},
		{pb_cpu_swap_r8, "SRL A", 4, REG_A, 0, 0},
		{pb_cpu_srl_r8, "SRL B", 4, REG_B, 0, 0},
		{pb_cpu_srl_r8, "SRL C", 4, REG_C, 0, 0},
		{pb_cpu_srl_r8, "SRL D", 4, REG_D, 0, 0},
		{pb_cpu_srl_r8, "SRL E", 4, REG_E, 0, 0},
		{pb_cpu_srl_r8, "SRL H", 4, REG_H, 0, 0},
		{pb_cpu_srl_r8, "SRL L", 4, REG_L, 0, 0},
		{pb_cpu_srl_hl, "SRL (HL)", 12, REG_H, 0, 0},
		{pb_cpu_srl_r8, "SRL A", 4, REG_A, 0, 0},
		{pb_cpu_bit_u3_r8, "BIT 0,B", 4, REG_B, BIT0_MASK, 0},
		{pb_cpu_bit_u3_r8, "BIT 0,C", 4, REG_C, BIT0_MASK, 0},
		{pb_cpu_bit_u3_r8, "BIT 0,D", 4, REG_D, BIT0_MASK, 0},
		{pb_cpu_bit_u3_r8, "BIT 0,E", 4, REG_E, BIT0_MASK, 0},
		{pb_cpu_bit_u3_r8, "BIT 0,H", 4, REG_H, BIT0_MASK, 0},
		{pb_cpu_bit_u3_r8, "BIT 0,L", 4, REG_L, BIT0_MASK, 0},
		{pb_cpu_bit_u3_hl, "BIT 0,(HL)", 12, REG_H, BIT0_MASK, 0},
		{pb_cpu_bit_u3_r8, "BIT 0,A", 4, REG_A, BIT0_MASK, 0},
		{pb_cpu_bit_u3_r8, "BIT 1,B", 4, REG_B, BIT1_MASK, 0},
		{pb_cpu_bit_u3_r8, "BIT 1,C", 4, REG_C, BIT1_MASK, 0},
		{pb_cpu_bit_u3_r8, "BIT 1,D", 4, REG_D, BIT1_MASK, 0},
		{pb_cpu_bit_u3_r8, "BIT 1,E", 4, REG_E, BIT1_MASK, 0},
		{pb_cpu_bit_u3_r8, "BIT 1,H", 4, REG_H, BIT1_MASK, 0},
		{pb_cpu_bit_u3_r8, "BIT 1,L", 4, REG_L, BIT1_MASK, 0},
		{pb_cpu_bit_u3_hl, "BIT 1,(HL)", 12, REG_H, BIT1_MASK, 0},
		{pb_cpu_bit_u3_r8, "BIT 1,A", 4, REG_A, BIT1_MASK, 0},
		{pb_cpu_bit_u3_r8, "BIT 2,B", 4, REG_B, BIT2_MASK, 0},
		{pb_cpu_bit_u3_r8, "BIT 2,C", 4, REG_C, BIT2_MASK, 0},
		{pb_cpu_bit_u3_r8, "BIT 2,D", 4, REG_D, BIT2_MASK, 0},
		{pb_cpu_bit_u3_r8, "BIT 2,E", 4, REG_E, BIT2_MASK, 0},
		{pb_cpu_bit_u3_r8, "BIT 2,H", 4, REG_H, BIT2_MASK, 0},
		{pb_cpu_bit_u3_r8, "BIT 2,L", 4, REG_L, BIT2_MASK, 0},
		{pb_cpu_bit_u3_hl, "BIT 2,(HL)", 12, REG_H, BIT2_MASK, 0},
		{pb_cpu_bit_u3_r8, "BIT 2,A", 4, REG_A, BIT2_MASK, 0},
		{pb_cpu_bit_u3_r8, "BIT 3,B", 4, REG_B, BIT3_MASK, 0},
		{pb_cpu_bit_u3_r8, "BIT 3,C", 4, REG_C, BIT3_MASK, 0},
		{pb_cpu_bit_u3_r8, "BIT 3,D", 4, REG_D, BIT3_MASK, 0},
		{pb_cpu_bit_u3_r8, "BIT 3,E", 4, REG_E, BIT3_MASK, 0},
		{pb_cpu_bit_u3_r8, "BIT 3,H", 4, REG_H, BIT3_MASK, 0},
		{pb_cpu_bit_u3_r8, "BIT 3,L", 4, REG_L, BIT3_MASK, 0},
		{pb_cpu_bit_u3_hl, "BIT 3,(HL)", 12, REG_H, BIT3_MASK, 0},
		{pb_cpu_bit_u3_r8, "BIT 3,A", 4, REG_A, BIT3_MASK, 0},
		{pb_cpu_bit_u3_r8, "BIT 4,B", 4, REG_B, BIT4_MASK, 0},
		{pb_cpu_bit_u3_r8, "BIT 4,C", 4, REG_C, BIT4_MASK, 0},
		{pb_cpu_bit_u3_r8, "BIT 4,D", 4, REG_D, BIT4_MASK, 0},
		{pb_cpu_bit_u3_r8, "BIT 4,E", 4, REG_E, BIT4_MASK, 0},
		{pb_cpu_bit_u3_r8, "BIT 4,H", 4, REG_H, BIT4_MASK, 0},
		{pb_cpu_bit_u3_r8, "BIT 4,L", 4, REG_L, BIT4_MASK, 0},
		{pb_cpu_bit_u3_hl, "BIT 4,(HL)", 12, REG_H, BIT4_MASK, 0},
		{pb_cpu_bit_u3_r8, "BIT 4,A", 4, REG_A, BIT4_MASK, 0},
		{pb_cpu_bit_u3_r8, "BIT 5,B", 4, REG_B, BIT5_MASK, 0},
		{pb_cpu_bit_u3_r8, "BIT 5,C", 4, REG_C, BIT5_MASK, 0},
		{pb_cpu_bit_u3_r8, "BIT 5,D", 4, REG_D, BIT5_MASK, 0},
		{pb_cpu_bit_u3_r8, "BIT 5,E", 4, REG_E, BIT5_MASK, 0},
		{pb_cpu_bit_u3_r8, "BIT 5,H", 4, REG_H, BIT5_MASK, 0},
		{pb_cpu_bit_u3_r8, "BIT 5,L", 4, REG_L, BIT5_MASK, 0},
		{pb_cpu_bit_u3_hl, "BIT 5,(HL)", 12, REG_H, BIT5_MASK, 0},
		{pb_cpu_bit_u3_r8, "BIT 5,A", 4, REG_A, BIT5_MASK, 0},
		{pb_cpu_bit_u3_r8, "BIT 6,B", 4, REG_B, BIT6_MASK, 0},
		{pb_cpu_bit_u3_r8, "BIT 6,C", 4, REG_C, BIT6_MASK, 0},
		{pb_cpu_bit_u3_r8, "BIT 6,D", 4, REG_D, BIT6_MASK, 0},
		{pb_cpu_bit_u3_r8, "BIT 6,E", 4, REG_E, BIT6_MASK, 0},
		{pb_cpu_bit_u3_r8, "BIT 6,H", 4, REG_H, BIT6_MASK, 0},
		{pb_cpu_bit_u3_r8, "BIT 6,L", 4, REG_L, BIT6_MASK, 0},
		{pb_cpu_bit_u3_hl, "BIT 6,(HL)", 12, REG_H, BIT6_MASK, 0},
		{pb_cpu_bit_u3_r8, "BIT 6,A", 4, REG_A, BIT6_MASK, 0},
		{pb_cpu_bit_u3_r8, "BIT 7,B", 4, REG_B, BIT7_MASK, 0},
		{pb_cpu_bit_u3_r8, "BIT 7,C", 4, REG_C, BIT7_MASK, 0},
		{pb_cpu_bit_u3_r8, "BIT 7,D", 4, REG_D, BIT7_MASK, 0},
		{pb_cpu_bit_u3_r8, "BIT 7,E", 4, REG_E, BIT7_MASK, 0},
		{pb_cpu_bit_u3_r8, "BIT 7,H", 4, REG_H, BIT7_MASK, 0},
		{pb_cpu_bit_u3_r8, "BIT 7,L", 4, REG_L, BIT7_MASK, 0},
		{pb_cpu_bit_u3_hl, "BIT 7,(HL)", 12, REG_H, BIT7_MASK, 0},
		{pb_cpu_bit_u3_r8, "BIT 7,A", 4, REG_A, BIT7_MASK, 0},
		{pb_cpu_res_u3_r8, "RES 0,B", 4, REG_B, BIT0_MASK, 0},
		{pb_cpu_res_u3_r8, "RES 0,C", 4, REG_C, BIT0_MASK, 0},
		{pb_cpu_res_u3_r8, "RES 0,D", 4, REG_D, BIT0_MASK, 0},
		{pb_cpu_res_u3_r8, "RES 0,E", 4, REG_E, BIT0_MASK, 0},
		{pb_cpu_res_u3_r8, "RES 0,H", 4, REG_H, BIT0_MASK, 0},
		{pb_cpu_res_u3_r8, "RES 0,L", 4, REG_L, BIT0_MASK, 0},
		{pb_cpu_res_u3_hl, "RES 0,(HL)", 12, REG_H, BIT0_MASK, 0},
		{pb_cpu_res_u3_r8, "RES 0,A", 4, REG_A, BIT0_MASK, 0},
		{pb_cpu_res_u3_r8, "RES 1,B", 4, REG_B, BIT1_MASK, 0},
		{pb_cpu_res_u3_r8, "RES 1,C", 4, REG_C, BIT1_MASK, 0},
		{pb_cpu_res_u3_r8, "RES 1,D", 4, REG_D, BIT1_MASK, 0},
		{pb_cpu_res_u3_r8, "RES 1,E", 4, REG_E, BIT1_MASK, 0},
		{pb_cpu_res_u3_r8, "RES 1,H", 4, REG_H, BIT1_MASK, 0},
		{pb_cpu_res_u3_r8, "RES 1,L", 4, REG_L, BIT1_MASK, 0},
		{pb_cpu_res_u3_hl, "RES 1,(HL)", 12, REG_H, BIT1_MASK, 0},
		{pb_cpu_res_u3_r8, "RES 1,A", 4, REG_A, BIT1_MASK, 0},
		{pb_cpu_res_u3_r8, "RES 2,B", 4, REG_B, BIT2_MASK, 0},
		{pb_cpu_res_u3_r8, "RES 2,C", 4, REG_C, BIT2_MASK, 0},
		{pb_cpu_res_u3_r8, "RES 2,D", 4, REG_D, BIT2_MASK, 0},
		{pb_cpu_res_u3_r8, "RES 2,E", 4, REG_E, BIT2_MASK, 0},
		{pb_cpu_res_u3_r8, "RES 2,H", 4, REG_H, BIT2_MASK, 0},
		{pb_cpu_res_u3_r8, "RES 2,L", 4, REG_L, BIT2_MASK, 0},
		{pb_cpu_res_u3_hl, "RES 2,(HL)", 12, REG_H, BIT2_MASK, 0},
		{pb_cpu_res_u3_r8, "RES 2,A", 4, REG_A, BIT2_MASK, 0},
		{pb_cpu_res_u3_r8, "RES 3,B", 4, REG_B, BIT3_MASK, 0},
		{pb_cpu_res_u3_r8, "RES 3,C", 4, REG_C, BIT3_MASK, 0},
		{pb_cpu_res_u3_r8, "RES 3,D", 4, REG_D, BIT3_MASK, 0},
		{pb_cpu_res_u3_r8, "RES 3,E", 4, REG_E, BIT3_MASK, 0},
		{pb_cpu_res_u3_r8, "RES 3,H", 4, REG_H, BIT3_MASK, 0},
		{pb_cpu_res_u3_r8, "RES 3,L", 4, REG_L, BIT3_MASK, 0},
		{pb_cpu_res_u3_hl, "RES 3,(HL)", 12, REG_H, BIT3_MASK, 0},
		{pb_cpu_res_u3_r8, "RES 3,A", 4, REG_A, BIT3_MASK, 0},
		{pb_cpu_res_u3_r8, "RES 4,B", 4, REG_B, BIT4_MASK, 0},
		{pb_cpu_res_u3_r8, "RES 4,C", 4, REG_C, BIT4_MASK, 0},
		{pb_cpu_res_u3_r8, "RES 4,D", 4, REG_D, BIT4_MASK, 0},
		{pb_cpu_res_u3_r8, "RES 4,E", 4, REG_E, BIT4_MASK, 0},
		{pb_cpu_res_u3_r8, "RES 4,H", 4, REG_H, BIT4_MASK, 0},
		{pb_cpu_res_u3_r8, "RES 4,L", 4, REG_L, BIT4_MASK, 0},
		{pb_cpu_res_u3_hl, "RES 4,(HL)", 12, REG_H, BIT4_MASK, 0},
		{pb_cpu_res_u3_r8, "RES 4,A", 4, REG_A, BIT4_MASK, 0},
		{pb_cpu_res_u3_r8, "RES 5,B", 4, REG_B, BIT5_MASK, 0},
		{pb_cpu_res_u3_r8, "RES 5,C", 4, REG_C, BIT5_MASK, 0},
		{pb_cpu_res_u3_r8, "RES 5,D", 4, REG_D, BIT5_MASK, 0},
		{pb_cpu_res_u3_r8, "RES 5,E", 4, REG_E, BIT5_MASK, 0},
		{pb_cpu_res_u3_r8, "RES 5,H", 4, REG_H, BIT5_MASK, 0},
		{pb_cpu_res_u3_r8, "RES 5,L", 4, REG_L, BIT5_MASK, 0},
		{pb_cpu_res_u3_hl, "RES 5,(HL)", 12, REG_H, BIT5_MASK, 0},
		{pb_cpu_res_u3_r8, "RES 5,A", 4, REG_A, BIT5_MASK, 0},
		{pb_cpu_res_u3_r8, "RES 6,B", 4, REG_B, BIT6_MASK, 0},
		{pb_cpu_res_u3_r8, "RES 6,C", 4, REG_C, BIT6_MASK, 0},
		{pb_cpu_res_u3_r8, "RES 6,D", 4, REG_D, BIT6_MASK, 0},
		{pb_cpu_res_u3_r8, "RES 6,E", 4, REG_E, BIT6_MASK, 0},
		{pb_cpu_res_u3_r8, "RES 6,H", 4, REG_H, BIT6_MASK, 0},
		{pb_cpu_res_u3_r8, "RES 6,L", 4, REG_L, BIT6_MASK, 0},
		{pb_cpu_res_u3_hl, "RES 6,(HL)", 12, REG_H, BIT6_MASK, 0},
		{pb_cpu_res_u3_r8, "RES 6,A", 4, REG_A, BIT6_MASK, 0},
		{pb_cpu_res_u3_r8, "RES 7,B", 4, REG_B, BIT7_MASK, 0},
		{pb_cpu_res_u3_r8, "RES 7,C", 4, REG_C, BIT7_MASK, 0},
		{pb_cpu_res_u3_r8, "RES 7,D", 4, REG_D, BIT7_MASK, 0},
		{pb_cpu_res_u3_r8, "RES 7,E", 4, REG_E, BIT7_MASK, 0},
		{pb_cpu_res_u3_r8, "RES 7,H", 4, REG_H, BIT7_MASK, 0},
		{pb_cpu_res_u3_r8, "RES 7,L", 4, REG_L, BIT7_MASK, 0},
		{pb_cpu_res_u3_hl, "RES 7,(HL)", 12, REG_H, BIT7_MASK, 0},
		{pb_cpu_res_u3_r8, "RES 7,A", 4, REG_A, BIT7_MASK, 0},
		{pb_cpu_set_u3_r8, "SET 0,B", 4, REG_B, BIT0_MASK, 0},
		{pb_cpu_set_u3_r8, "SET 0,C", 4, REG_C, BIT0_MASK, 0},
		{pb_cpu_set_u3_r8, "SET 0,D", 4, REG_D, BIT0_MASK, 0},
		{pb_cpu_set_u3_r8, "SET 0,E", 4, REG_E, BIT0_MASK, 0},
		{pb_cpu_set_u3_r8, "SET 0,H", 4, REG_H, BIT0_MASK, 0},
		{pb_cpu_set_u3_r8, "SET 0,L", 4, REG_L, BIT0_MASK, 0},
		{pb_cpu_set_u3_hl, "SET 0,(HL)", 12, REG_H, BIT0_MASK, 0},
		{pb_cpu_set_u3_r8, "SET 0,A", 4, REG_A, BIT0_MASK, 0},
		{pb_cpu_set_u3_r8, "SET 1,B", 4, REG_B, BIT1_MASK, 0},
		{pb_cpu_set_u3_r8, "SET 1,C", 4, REG_C, BIT1_MASK, 0},
		{pb_cpu_set_u3_r8, "SET 1,D", 4, REG_D, BIT1_MASK, 0},
		{pb_cpu_set_u3_r8, "SET 1,E", 4, REG_E, BIT1_MASK, 0},
		{pb_cpu_set_u3_r8, "SET 1,H", 4, REG_H, BIT1_MASK, 0},
		{pb_cpu_set_u3_r8, "SET 1,L", 4, REG_L, BIT1_MASK, 0},
		{pb_cpu_set_u3_hl, "SET 1,(HL)", 12, REG_H, BIT1_MASK, 0},
		{pb_cpu_set_u3_r8, "SET 1,A", 4, REG_A, BIT1_MASK, 0},
		{pb_cpu_set_u3_r8, "SET 2,B", 4, REG_B, BIT2_MASK, 0},
		{pb_cpu_set_u3_r8, "SET 2,C", 4, REG_C, BIT2_MASK, 0},
		{pb_cpu_set_u3_r8, "SET 2,D", 4, REG_D, BIT2_MASK, 0},
		{pb_cpu_set_u3_r8, "SET 2,E", 4, REG_E, BIT2_MASK, 0},
		{pb_cpu_set_u3_r8, "SET 2,H", 4, REG_H, BIT2_MASK, 0},
		{pb_cpu_set_u3_r8, "SET 2,L", 4, REG_L, BIT2_MASK, 0},
		{pb_cpu_set_u3_hl, "SET 2,(HL)", 12, REG_H, BIT2_MASK, 0},
		{pb_cpu_set_u3_r8, "SET 2,A", 4, REG_A, BIT2_MASK, 0},
		{pb_cpu_set_u3_r8, "SET 3,B", 4, REG_B, BIT3_MASK, 0},
		{pb_cpu_set_u3_r8, "SET 3,C", 4, REG_C, BIT3_MASK, 0},
		{pb_cpu_set_u3_r8, "SET 3,D", 4, REG_D, BIT3_MASK, 0},
		{pb_cpu_set_u3_r8, "SET 3,E", 4, REG_E, BIT3_MASK, 0},
		{pb_cpu_set_u3_r8, "SET 3,H", 4, REG_H, BIT3_MASK, 0},
		{pb_cpu_set_u3_r8, "SET 3,L", 4, REG_L, BIT3_MASK, 0},
		{pb_cpu_set_u3_hl, "SET 3,(HL)", 12, REG_H, BIT3_MASK, 0},
		{pb_cpu_set_u3_r8, "SET 3,A", 4, REG_A, BIT3_MASK, 0},
		{pb_cpu_set_u3_r8, "SET 4,B", 4, REG_B, BIT4_MASK, 0},
		{pb_cpu_set_u3_r8, "SET 4,C", 4, REG_C, BIT4_MASK, 0},
		{pb_cpu_set_u3_r8, "SET 4,D", 4, REG_D, BIT4_MASK, 0},
		{pb_cpu_set_u3_r8, "SET 4,E", 4, REG_E, BIT4_MASK, 0},
		{pb_cpu_set_u3_r8, "SET 4,H", 4, REG_H, BIT4_MASK, 0},
		{pb_cpu_set_u3_r8, "SET 4,L", 4, REG_L, BIT4_MASK, 0},
		{pb_cpu_set_u3_hl, "SET 4,(HL)", 12, REG_H, BIT4_MASK, 0},
		{pb_cpu_set_u3_r8, "SET 4,A", 4, REG_A, BIT4_MASK, 0},
		{pb_cpu_set_u3_r8, "SET 5,B", 4, REG_B, BIT5_MASK, 0},
		{pb_cpu_set_u3_r8, "SET 5,C", 4, REG_C, BIT5_MASK, 0},
		{pb_cpu_set_u3_r8, "SET 5,D", 4, REG_D, BIT5_MASK, 0},
		{pb_cpu_set_u3_r8, "SET 5,E", 4, REG_E, BIT5_MASK, 0},
		{pb_cpu_set_u3_r8, "SET 5,H", 4, REG_H, BIT5_MASK, 0},
		{pb_cpu_set_u3_r8, "SET 5,L", 4, REG_L, BIT5_MASK, 0},
		{pb_cpu_set_u3_hl, "SET 5,(HL)", 12, REG_H, BIT5_MASK, 0},
		{pb_cpu_set_u3_r8, "SET 5,A", 4, REG_A, BIT5_MASK, 0},
		{pb_cpu_set_u3_r8, "SET 6,B", 4, REG_B, BIT6_MASK, 0},
		{pb_cpu_set_u3_r8, "SET 6,C", 4, REG_C, BIT6_MASK, 0},
		{pb_cpu_set_u3_r8, "SET 6,D", 4, REG_D, BIT6_MASK, 0},
		{pb_cpu_set_u3_r8, "SET 6,E", 4, REG_E, BIT6_MASK, 0},
		{pb_cpu_set_u3_r8, "SET 6,H", 4, REG_H, BIT6_MASK, 0},
		{pb_cpu_set_u3_r8, "SET 6,L", 4, REG_L, BIT6_MASK, 0},
		{pb_cpu_set_u3_hl, "SET 6,(HL)", 12, REG_H, BIT6_MASK, 0},
		{pb_cpu_set_u3_r8, "SET 6,A", 4, REG_A, BIT6_MASK, 0},
		{pb_cpu_set_u3_r8, "SET 7,B", 4, REG_B, BIT7_MASK, 0},
		{pb_cpu_set_u3_r8, "SET 7,C", 4, REG_C, BIT7_MASK, 0},
		{pb_cpu_set_u3_r8, "SET 7,D", 4, REG_D, BIT7_MASK, 0},
		{pb_cpu_set_u3_r8, "SET 7,E", 4, REG_E, BIT7_MASK, 0},
		{pb_cpu_set_u3_r8, "SET 7,H", 4, REG_H, BIT7_MASK, 0},
		{pb_cpu_set_u3_r8, "SET 7,L", 4, REG_L, BIT7_MASK, 0},
		{pb_cpu_set_u3_hl, "SET 7,(HL)", 12, REG_H, BIT7_MASK, 0},
		{pb_cpu_set_u3_r8, "SET 7,A", 4, REG_A, BIT7_MASK, 0}
};
