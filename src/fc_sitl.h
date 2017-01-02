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
#pragma once

/**
 * @addtogroup sitl-interface
 * @{
 */
/**
 * @file fc_sitl.h
 */

#include <stdint.h>
#include <stdbool.h>

//! number of pwm channels that sitl supports
#define FC_SITL_PWM_CHANNELS 16
/**
 * Interface passed to the SITL by the flight controller
 * Currently this is very simple and mostly just a placeholder.
 */
struct fc_sitl_server_interface {
	uint8_t dummy;
};

#ifdef __cplusplus
extern "C" {
#endif

struct system_calls;
typedef void* fc_handle_t;

fc_handle_t fc_sitl_create_instance(const char *dll, const struct system_calls *system);

#ifdef __cplusplus
}
#endif

/** @} */
