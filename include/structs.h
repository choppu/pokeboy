/*
 * Copyright (C) 2019, Ksenia Balistreri
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the BSD license.  See the LICENSE file for details.
 */

#ifndef _STRUCTS_H_
#define _STRUCTS_H_

#include <stdint.h>

#define REG_COUNT 10
#define HRAM_SIZE 127

#define VRAM_SIZE 8192
#define WRAM_SIZE 8192
#define OAM_SIZE 160
#define IO_REGS_SIZE 80

#define REG_A 0
#define REG_F 1
#define REG_B 2
#define REG_C 3
#define REG_D 4
#define REG_E 5
#define REG_H 6
#define REG_L 7
#define REG_SP 8

#define MAX_COMMAND_LENGTH 20

typedef struct {
	void (*func)();
	char* command;
	uint8_t cycles;
	uint8_t op1;
	uint8_t op2;
	uint8_t imm_length;
} pb_inst_t;

typedef enum {
	INST_FETCH_DECODE, CPU_WAIT, INST_EXECUTE
} pb_cpu_state_t;

typedef enum {
	NORMAL, HALT, STOP
} pb_cpu_mode_t;

typedef enum {
	ROM_ONLY = 0x00, MBC1 = 0x01, MBC1_RAM = 0x02, MBC1_RAM_BATT = 0x03, MBC_2 = 0x05,
	MBC2_RAM_BATT = 0x06, ROM_RAM = 0x08, ROM_RAM_BATT = 0x09, MMM_01 = 0x0b, MMM_01_RAM = 0x0c,
	MMM_01_SRAM_BATT = 0x0d, MBC_3_TIMER_BATT = 0x0f, MBC3_TIMER_RAM_BATT = 0x10,
	MBC3 = 0x11, MBC3_RAM = 0x12, MBC3_RAM_BATT = 0x13,
	MBC5 = 0x19, MBC5_RAM = 0x1a, MBC5_RAM_BATT = 0x1b, MBC5_RUMMBLE = 0x1c, MBC5_RAM_RUMMBLE = 0x1d,
	MBC5_RAM_RUMMBLE_BATT = 0x1e, MBC6_RAM_BATT = 0x20, MBC7_RAM_BATT_ACCEL = 0x22,
	POCKET_CAMERA = 0xfc, BANDAI_TAMA5 = 0xfd, HUC3 = 0xfe, HUC1_RAM_BATT = 0xff
} pb_cart_mb_controller_t;

typedef struct {
	uint8_t registers[REG_COUNT];
	uint8_t hram[HRAM_SIZE];
	uint8_t ime;
	uint16_t programm_counter;
	pb_cpu_state_t cpu_state;
	pb_cpu_mode_t cpu_mode;
	uint8_t wait_cycles;
	uint16_t imm;
	pb_inst_t* instr;
} pb_cpu_t;

typedef struct {
	uint8_t *rom;
	uint8_t *ram;
	uint8_t ram_enabled;
	uint8_t rom_size;
	uint8_t ram_size;
	uint8_t active_rom_bank;
	uint8_t active_ram_bank;
	uint8_t rom_ram_mode;
	pb_cart_mb_controller_t mbc_mode;
} pb_cart_t;

typedef struct {
	pb_cpu_t cpu;
	pb_cart_t cart;
	uint8_t pb_run;
	uint8_t video_ram[VRAM_SIZE];
	uint8_t w_ram[WRAM_SIZE];
	uint8_t sprite_table[OAM_SIZE];
	uint8_t io_registers[IO_REGS_SIZE];
	uint8_t ie_register;
	uint16_t divider_register;
	uint8_t div_delay;
} pb_vm_t;

#endif
