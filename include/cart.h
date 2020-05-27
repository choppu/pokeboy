/*
 * Copyright (C) 2019, Ksenia Balistreri
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the BSD license.  See the LICENSE file for details.
 */

#ifndef _CART_H_
#define _CART_H_

#include <stdint.h>
#include "vm.h"

#define CART_TYPE 0x0147
#define RAM_SIZE 0x0149

#define RAM_NONE	0x00
#define RAM_2_KB	0x01
#define RAM_8_KB	0x02
#define RAM_32_KB	0x03
#define RAM_128_KB	0x04
#define RAM_64_KB	0x05

void pb_cart_init(pb_cart_t* cart);

#endif
