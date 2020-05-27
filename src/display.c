/*
 * Copyright (C) 2019, Ksenia Balistreri
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the BSD license.  See the LICENSE file for details.
 */

#include <SDL2/SDL.h>
#include "vm.h"
#include "display.h"


#define DISPLAY_SCALING 10
#define DISPLAY_SCREEN_WIDTH 	(SCREEN_WIDTH * DISPLAY_SCALING)
#define DISPLAY_SCREEN_HEIGHT 	(SCREEN_HEIGHT * DISPLAY_SCALING)

static SDL_Window	*pb_display_window;
static SDL_Renderer *pb_display_renderer;
static SDL_Texture *pb_display_texture;

void pb_display_init() {
	pb_display_window = SDL_CreateWindow("Pokeboy", SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED, DISPLAY_SCREEN_WIDTH, DISPLAY_SCREEN_HEIGHT, SDL_WINDOW_SHOWN);
	pb_display_renderer = SDL_CreateRenderer(pb_display_window, -1, 0);
	SDL_RenderSetLogicalSize(pb_display_renderer, DISPLAY_SCREEN_WIDTH, DISPLAY_SCREEN_HEIGHT);
	pb_display_texture = SDL_CreateTexture(pb_display_renderer, SDL_PIXELFORMAT_ARGB8888, SDL_TEXTUREACCESS_STREAMING, SCREEN_WIDTH, SCREEN_HEIGHT);
}

int pb_display_draw(pb_vm_t* vm) {
	return 0;
}

void pb_display_destroy() {
	SDL_DestroyTexture(pb_display_texture);
	SDL_DestroyRenderer(pb_display_renderer);
	SDL_DestroyWindow(pb_display_window);
}
