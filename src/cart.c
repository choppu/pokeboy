/*
 * Copyright (C) 2019, Ksenia Balistreri
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the BSD license.  See the LICENSE file for details.
 */

#include <stdint.h>
#include <stdlib.h>
#include "cart.h"

#define KB2_RAM		1
#define KB8_RAM 	2
#define KB32_RAM 	3
#define KB128_RAM 	4

#define KB2			2048
#define KB8			8192
#define KB32		32768
#define KB128		131072

#define RAM_DISABLED 0
#define RAM_ENABLED  1

#define MBC 		0x0147
#define ROM_SIZE 	0x0148
#define RAM_SIZE 	0x0149

#define ROM_32_KB	0x00
#define ROM_64_KB	0x01
#define ROM_128_KB	0x02
#define ROM_256_KB	0x03
#define ROM_512_KB	0x04
#define ROM_1_MB	0x05
#define ROM_2_MB	0x06
#define ROM_4_MB	0x07
#define ROM_8_MB	0x08



void pb_cart_init(pb_cart_t* cart) {
	cart->ram_enabled = RAM_DISABLED;

	switch (cart->rom[RAM_SIZE]) {
		case KB2_RAM:
			cart->ram = malloc(KB2);
			break;
		case KB8_RAM:
			cart->ram = malloc(KB8);
			break;
		case KB32_RAM:
			cart->ram = malloc(KB32);
			break;
		case KB128_RAM:
			cart->ram = malloc(KB128);
			break;
	    default:
	    	cart->ram = NULL;
	    	break;
	}

	cart->active_rom_bank = 1;
	cart->active_ram_bank = 0;
	cart->mbc_mode = cart->rom[MBC];
	cart->rom_size = cart->rom[ROM_SIZE];
	cart->ram_size = cart->rom[RAM_SIZE];
	cart->rom_ram_mode = 0;
}
