/*
 * Copyright (C) 2019, Ksenia Balistreri
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the BSD license.  See the LICENSE file for details.
 */

#include <stdlib.h>
#include <stdio.h>
#include "structs.h"
#include "vm.h"
#include "cart.h"

#define ROM0_BASIC_ADDR 	0x0000
#define ROMX_BASIC_ADDR 	0x4000
#define VRAM_BASIC_ADDR 	0x8000
#define SRAM_BASIC_ADDR 	0xa000
#define WRAM0_BASIC_ADDR 	0xc000
#define ECHO_BASIC_ADDR 	0xe000
#define OAM_BASIC_ADDR 		0xfe00
#define UNUSED_ADDR			0xfea0
#define IO_REG_BASIC_ADDR 	0xff00
#define HRAM_BASIC_ADDR 	0xff80
#define IE_REG_ADDR 		0xffff

#define R_BANK_00 0x00
#define R_BANK_20 0x20
#define R_BANK_40 0x40
#define R_BANK_60 0x60

#define NO_RAM_REPLY	0xff

#define RAM_BANK_SIZE 8192

#define ROM_BANK_ADD_1(cond, bank) cond ? bank + 1 : bank

#define RAM_ENABLE_MASK 	0x0a
#define ROM_BANK_MASK		0x1f
#define RAM_ROM_BANK_MASK	0x03
#define ROM_RAM_MODE_MASK	0x01

#define RAM_ENABLE_ADDR 		0x1fff
#define ROM_BANK_SELECT_ADDR 	0x3fff
#define RAM_BANK_SELECT_ADDR	0x5fff
#define ROM_RAM_MODE			0x7fff

#define ROM_BANKING_MODE 0x00
#define RAM_BANKING_MODE 0x01

#define IO_REG_DIV 0xff04

void update_rom_bank(pb_vm_t* vm, uint8_t data) {
	vm->cart.active_rom_bank = (data << 6) | (vm->cart.active_rom_bank & ROM_BANK_MASK);
}

void select_ram_bank(pb_vm_t* vm, uint8_t data) {
	vm->cart.active_ram_bank = data;
}

uint8_t pb_rom_read(pb_vm_t* vm, uint16_t addr) {
	uint16_t mem_addr = (addr < ROMX_BASIC_ADDR) ? addr : (ROMX_BASIC_ADDR * vm->cart.active_rom_bank) + (addr - ROMX_BASIC_ADDR);
	return vm->cart.rom[mem_addr];
}

uint8_t pb_ram_read(pb_vm_t* vm, uint16_t addr) {
	uint16_t mem_addr = (RAM_BANK_SIZE * vm->cart.active_ram_bank) + (addr - SRAM_BASIC_ADDR);
	return ((vm->cart.ram_enabled == 1) ? vm->cart.ram[mem_addr] : 0xff);
}

void pb_rom_write(pb_vm_t* vm, uint16_t addr, uint8_t data) {
	if (addr <= RAM_ENABLE_ADDR) {
		vm->cart.ram_enabled = ((data & RAM_ENABLE_MASK) == RAM_ENABLE_MASK);
	} else if ((addr > RAM_ENABLE_ADDR) && (addr <= ROM_BANK_SELECT_ADDR)) {
		uint8_t active_rom_bank = (data & ROM_BANK_MASK) | (vm->cart.active_rom_bank & RAM_ROM_BANK_MASK);
		vm->cart.active_rom_bank = ROM_BANK_ADD_1(((active_rom_bank == R_BANK_00) || (active_rom_bank == R_BANK_20) ||
		(active_rom_bank == R_BANK_40) || (active_rom_bank == R_BANK_60)), active_rom_bank);
	} else if ((addr > ROM_BANK_SELECT_ADDR) && (addr <= RAM_BANK_SELECT_ADDR)) {
		uint8_t ram_rom_bank_mask = data | RAM_ROM_BANK_MASK;
		vm->cart.rom_ram_mode ==  ROM_RAM_MODE_MASK ? update_rom_bank(vm , ram_rom_bank_mask) : select_ram_bank(vm, ram_rom_bank_mask);
	} else if (addr > RAM_BANK_SELECT_ADDR) {
		vm->cart.rom_ram_mode = data & ROM_RAM_MODE_MASK;
	}
}

void pb_ram_write(pb_vm_t* vm, uint16_t addr, uint8_t data) {
	uint16_t mem_addr = (RAM_BANK_SIZE * vm->cart.active_ram_bank) + (addr - SRAM_BASIC_ADDR);
	if (vm->cart.ram_enabled) {
		vm->cart.ram[mem_addr] = data;
	}
}

uint8_t pb_cart_mem_read(pb_vm_t* vm, uint16_t addr) {
	return (addr < SRAM_BASIC_ADDR) ? pb_rom_read(vm, addr) : pb_ram_read(vm, addr);
}

void pb_cart_mem_write(pb_vm_t* vm, uint16_t addr, uint8_t data) {
	(addr < SRAM_BASIC_ADDR) ? pb_rom_write(vm, addr, data) : pb_ram_write(vm, addr, data);
}

uint8_t pb_io_regs_read(pb_vm_t* vm, uint16_t addr) {
	return (addr == IO_REG_DIV) ? (vm->divider_register >> 8) : vm->io_registers[addr - IO_REG_BASIC_ADDR];
}

void pb_io_regs_write(pb_vm_t* vm, uint16_t addr, uint8_t data) {
	if (addr == IO_REG_DIV) {
		vm->divider_register = 0;
	} else {
		vm->io_registers[addr - IO_REG_BASIC_ADDR] = data;
	}
}

uint8_t pb_mem_read(pb_vm_t* vm, uint16_t addr) {
	uint8_t data = 0;

	if ((addr < VRAM_BASIC_ADDR) || ((addr >= SRAM_BASIC_ADDR) && (addr < WRAM0_BASIC_ADDR))) {
		data = pb_cart_mem_read(vm, addr);
	} else if ((addr >= VRAM_BASIC_ADDR) && (addr < SRAM_BASIC_ADDR)) {
		data = vm->video_ram[addr - VRAM_BASIC_ADDR];
	} else if ((addr >= WRAM0_BASIC_ADDR) && (addr < ECHO_BASIC_ADDR)) {
		data = vm->w_ram[addr - WRAM0_BASIC_ADDR];
	} else if ((addr >= OAM_BASIC_ADDR) && (addr < UNUSED_ADDR)) {
		data = vm->sprite_table[addr - OAM_BASIC_ADDR];
	} else if ((addr >= IO_REG_BASIC_ADDR) && (addr < HRAM_BASIC_ADDR)) {
		data = pb_io_regs_read(vm, addr);
	} else if ((addr >= HRAM_BASIC_ADDR) && (addr < IE_REG_ADDR)) {
		data = vm->cpu.hram[addr - HRAM_BASIC_ADDR];
	} else if (addr == IE_REG_ADDR) {
		data = vm->ie_register;
	}

	return data;
}

void pb_mem_write(pb_vm_t* vm, uint16_t addr, uint8_t data) {
	if ((addr < VRAM_BASIC_ADDR) || ((addr >= SRAM_BASIC_ADDR) && (addr < WRAM0_BASIC_ADDR))) {
		pb_cart_mem_write(vm, addr, data);
	} else if ((addr >= VRAM_BASIC_ADDR) && (addr < SRAM_BASIC_ADDR)) {
		vm->video_ram[addr - VRAM_BASIC_ADDR] = data;
	} else if ((addr >= WRAM0_BASIC_ADDR) && (addr < ECHO_BASIC_ADDR)) {
		vm->w_ram[addr - WRAM0_BASIC_ADDR] = data;
	} else if ((addr >= OAM_BASIC_ADDR) && (addr < UNUSED_ADDR)) {
		vm->sprite_table[addr - OAM_BASIC_ADDR] = data;
	} else if ((addr >= IO_REG_BASIC_ADDR) && (addr < HRAM_BASIC_ADDR)) {
		pb_io_regs_write(vm, addr, data);

		if (addr == 0xFF02 && data == 0x81) {
			fprintf(stderr, "%c", pb_mem_read(vm, 0xFF01));
		}
	} else if ((addr >= HRAM_BASIC_ADDR) && (addr < IE_REG_ADDR)) {
		vm->cpu.hram[addr - HRAM_BASIC_ADDR] = data;
	} else if (addr == IE_REG_ADDR) {
		vm->ie_register = data;
	}
}



