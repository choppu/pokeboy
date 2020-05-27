/*
 * Copyright (C) 2019, Ksenia Balistreri
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the BSD license.  See the LICENSE file for details.
 */

#ifndef _AUDIO_H_
#define _AUDIO_H_

#include <stdint.h>
#include "vm.h"

void pb_audio_init();
int pb_audio_play(pb_vm_t* vm);

#endif
