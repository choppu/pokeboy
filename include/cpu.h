/*
 * Copyright (C) 2019, Ksenia Balistreri
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the BSD license.  See the LICENSE file for details.
 */

#ifndef _CPU_H_
#define _CPU_H_

#include <stdint.h>
#include "vm.h"

#define REG_ACC 0
#define REG_FLAGS 1

void pb_set_uint16_reg(pb_cpu_t* cpu, int i, uint16_t data);
uint16_t pb_get_uint_16_reg(pb_cpu_t* cpu, int i);
void pb_cpu_stack_push(pb_vm_t* vm, uint16_t data);
uint16_t pb_cpu_stack_pop(pb_vm_t* vm);
void pb_cpu_init(pb_cpu_t* cpu);
int pb_cpu_cycle(pb_vm_t* vm);

#endif
