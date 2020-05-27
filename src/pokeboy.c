/*
 * Copyright (C) 2019, Ksenia Balistreri
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the BSD license.  See the LICENSE file for details.
 */

#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <errno.h>
#include <unistd.h>
#include <string.h>
#include <fcntl.h>
#include <SDL2/SDL.h>
#include "vm.h"

int pb_load_rom(const char* path, pb_vm_t* vm) {
	int fd = open(path, O_RDONLY);

	if (fd == -1) {
		return errno;
	}

	struct stat file_info;

	if (fstat(fd, &file_info) == -1) {
		close(fd);
		return errno;
	};

	vm->cart.rom = malloc(file_info.st_size);

	if (read(fd, vm->cart.rom, file_info.st_size) == -1) {
		close(fd);
		return errno;
	};

	close(fd);

	return 0;
}

int main(int argc, char* argv[]) {
	pb_vm_t vm;

	if(pb_load_rom(argv[1], &vm) != 0) {
		printf("Error loading file");
		return EXIT_FAILURE;
	}

	pb_vm_run(&vm);
	return 0;
}
