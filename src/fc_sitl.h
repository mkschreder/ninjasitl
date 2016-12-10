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
 * Interface passed to the flight controller shared library
 */
struct fc_sitl_client_interface {
	//! read rc channel value
	uint16_t (*read_rc)(struct fc_sitl_client_interface *self, uint8_t chan);
	//! read gyro value
	void (*read_gyro)(struct fc_sitl_client_interface *self, float gyro[3]);
	//! read accelerometer
	void (*read_accel)(struct fc_sitl_client_interface *self, float accel[3]);
	//! read compass
	void (*read_mag)(struct fc_sitl_client_interface *self, float mag[3]);
	//! read range information
	int (*read_range)(struct fc_sitl_client_interface *self, uint16_t deg, uint16_t *range);
	//! write led value
	void (*led_on)(struct fc_sitl_client_interface *self, uint8_t id, bool value);
	//! toggle a led
	void (*led_toggle)(struct fc_sitl_client_interface *self, uint8_t id);
	//! write pwm output values
	void (*write_pwm)(struct fc_sitl_client_interface *self, int8_t chan, uint16_t value);
	//! write euler angles
	void (*write_euler_angles)(struct fc_sitl_client_interface *self, int16_t roll, int16_t pitch, int16_t yaw);
	//! SITL specific data
	void *data;
};

/**
 * Interface passed to the SITL by the flight controller
 * Currently this is very simple and mostly just a placeholder.
 */
struct fc_sitl_server_interface {
	struct fc_sitl_client_interface *client;
};

#ifdef __cplusplus
extern "C" {
#endif

struct fc_sitl_server_interface *fc_sitl_create_instance(const char *dll, struct fc_sitl_client_interface *client);

#ifdef __cplusplus
}
#endif

/** @} */
