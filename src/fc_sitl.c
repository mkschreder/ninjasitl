/*
	Copyright (c) 2016 Martin Schröder <mkschreder.uk@gmail.com>

	This program is free software: you can redistribute it and/or modify
	it under the terms of the GNU General Public License as published by
	the Free Software Foundation, either version 3 of the License, or
	(at your option) any later version.

	This program is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
	GNU General Public License for more details.

	You should have received a copy of the GNU General Public License
	along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

/**
 * @addtogroup sitl-interface
 * @{
 */
/**
 * @file fc_sitl.c
 */

#include <stdio.h>
#include <stdlib.h>
#include <dlfcn.h>

#include "system_calls.h"
#include "fc_sitl.h"

#include <unistd.h>
#include <time.h>

//char _flash[SITL_EEPROM_PAGE_SIZE * SITL_EEPROM_NUM_PAGES] = {0};
/**
 * Loads a flight controller shared library and returns interface to the flight controller.
 */
struct fc_sitl_server_interface *fc_sitl_create_instance(const char *dll, const struct system_calls *system){
	void *dl = dlopen(dll, RTLD_LAZY|RTLD_LOCAL);
	if(dl == NULL){
		printf("could not load shared library: %s\n", dlerror());
		return NULL;
	}
	// get pointer to the creation method and create a new aircraft instance
	struct fc_sitl_server_interface *(*create)(const struct system_calls *) = dlsym(dl, "fc_sitl_create_aircraft");
	if(create == NULL){
		printf("library is not a sitl module: %s\n", dlerror());
		return NULL;
	}
	struct fc_sitl_server_interface *server = create(system);
	return server;
}

/** @} */
