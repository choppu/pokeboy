/*
 * Copyright (C) 2019, Ksenia Balistreri
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the BSD license.  See the LICENSE file for details.
 */

#ifndef _MEM_H_
#define _MEM_H_

#include <stdint.h>
#include "vm.h"

uint8_t pb_cart_mem_read(pb_vm_t* vm, uint16_t addr);
void pb_cart_mem_write(pb_vm_t* vm, uint16_t addr, uint8_t data);
uint8_t pb_mem_read(pb_vm_t* vm, uint16_t addr);
void pb_mem_write(pb_vm_t* vm, uint16_t addr, uint8_t data);

#endif
