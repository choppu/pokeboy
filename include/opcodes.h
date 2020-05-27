/*
 * Copyright (C) 2019, Ksenia Balistreri
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the BSD license.  See the LICENSE file for details.
 */

#ifndef _OPCODES_H_
#define _OPCODES_H_

#include <stdint.h>
#include "vm.h"

#define TABLE_SIZE 256

extern pb_inst_t pb_cpu_opcodes[TABLE_SIZE];
extern pb_inst_t pb_pref_cb_opcodes[TABLE_SIZE];

#endif
