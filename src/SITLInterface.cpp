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
 * @addtogroup sitl-interface SITL Interface
 * @{
 */
/**
 * @file SITLInterface.cpp
 */
/**
 * @addtogroup sitl-interface
 * @page sitl-interface
 *
 * Implements a shared interface for interfacing with flight controllers.
 * Application usually exports fc_sitl_client_interface to the flight
 * controller and then flight controller polls the SITL for new data as well as
 * sending back control inputs to the sitl.
 *
 * Since each flight controller is running in a separate thread and sitl
 * application is reading the same data that flight controller is writing, this
 * interface also provides means to synchronize data access between sitl and
 * the flight controller. For the most part pthread mutexes are enough for
 * this.
 */

#include <pthread.h>
#include "SITLInterface.h"
#include "fc_sitl.h"

SITLInterface* SITLInterface::_load_sitl(const char *dlname){
	struct fc_sitl_client_interface *cl = (struct fc_sitl_client_interface*)calloc(1, sizeof(struct fc_sitl_client_interface));
	cl->read_rc = read_rc;
	cl->read_gyro = read_gyro;
	cl->read_accel = read_accel;
	cl->read_mag = read_mag;
	cl->read_range = read_range;
	cl->write_pwm = write_pwm;
	cl->led_on = led_on;
	cl->led_toggle = led_toggle;
	cl->write_euler_angles = write_euler_angles;
	cl->data = new SITLInterface(cl);
	struct fc_sitl_server_interface *s = fc_sitl_create_instance(dlname, cl);
	if(!s) {
		delete ((SITLInterface*)cl->data);
		free(cl);
		return NULL;
	}
	return (SITLInterface*)cl->data;
}

#include <string.h>

SITLInterface::SITLInterface(struct fc_sitl_client_interface *client){
	this->client = client;
	for(int c = 0; c < FC_SITL_PWM_CHANNELS; c++){
		this->_rc[c] = 1500;
		this->_pwm[c] = 1000;
	}
	for(int c = 0; c < SITL_RANGE_SENSOR_COUNT; c++){
		_range[c] = -1;
	}
	memset(_leds, 0, sizeof(_leds));
	pthread_mutex_init(&_lock, NULL);
}

SITLInterface::~SITLInterface(){
	free(client);
	pthread_mutex_destroy(&_lock);
}

/**
 * Creates an instance of the sitl interface for a specific shared library (or
 * flight controller).
 *
 * @param type on of the supported flight controller types
 */
SITLInterface* SITLInterface::create(sitl_controller_type_t type){
	switch(type){
		case SITL_NINJAFLIGHT:
			return _load_sitl("./fc_ninjaflight.so");
	}
	return NULL;
}

/**
 * This method is used by SITL to make current accelerometer reading available
 * to flight controller. Units are ms/s.
 */
void SITLInterface::write_accel(float x, float y, float z){
	pthread_mutex_lock(&_lock);
	_accel.x = x;
	_accel.y = y;
	_accel.z = z;
	pthread_mutex_unlock(&_lock);
}

/**
 * Used by SITL to send gyro readings to flight controller. Units are rad/s.
 */
void SITLInterface::write_gyro(float x, float y, float z){
	pthread_mutex_lock(&_lock);
	_gyro.x = x;
	_gyro.y = y;
	_gyro.z = z;
	pthread_mutex_unlock(&_lock);
}

/**
 * Used by SITL to send rc inputs to the flight controller. Units are pwm
 * values 1000-2000.
 */
void SITLInterface::write_rc(uint8_t chan, float value){
	if(chan > FC_SITL_PWM_CHANNELS) return;
	pthread_mutex_lock(&_lock);
	_rc[chan] = value;
	pthread_mutex_unlock(&_lock);
}

/**
 * Used by SITL to read current led state for a led
 */
bool SITLInterface::read_led(uint8_t led){
	pthread_mutex_lock(&_lock);
	bool res = _leds[led];
	pthread_mutex_unlock(&_lock);
	return res;
}

/**
 * Used by flight controller to read latest pwm values
 */
uint16_t SITLInterface::read_rc(struct fc_sitl_client_interface *self, uint8_t chan){
	SITLInterface *s = (SITLInterface*)self->data;
	if(chan > FC_SITL_PWM_CHANNELS) return 1000;
	pthread_mutex_lock(&s->_lock);
	uint16_t rc = s->_rc[chan];
	pthread_mutex_unlock(&s->_lock);
	return rc;
}

/**
 * Used by flight controller to read latest gyro values
 */
void SITLInterface::read_gyro(struct fc_sitl_client_interface *self, float gyro[3]){
	SITLInterface *s = (SITLInterface*)self->data;
	pthread_mutex_lock(&s->_lock);
	gyro[0] = s->_gyro.x;
	gyro[1] = s->_gyro.y;
	gyro[2] = s->_gyro.z;
	pthread_mutex_unlock(&s->_lock);
}

/**
 * Used by flight controller to read latest accelerometer values
 */
void SITLInterface::read_accel(struct fc_sitl_client_interface *self, float accel[3]){
	SITLInterface *s = (SITLInterface*)self->data;
	pthread_mutex_lock(&s->_lock);
	accel[0] = s->_accel.x;
	accel[1] = s->_accel.y;
	accel[2] = s->_accel.z;
	pthread_mutex_unlock(&s->_lock);
}

/**
 * Used by flight controller to read latest compass values
 */
void SITLInterface::read_mag(struct fc_sitl_client_interface *self, float mag[3]){
	SITLInterface *s = (SITLInterface*)self->data;
	pthread_mutex_lock(&s->_lock);
	mag[0] = s->_mag.x;
	mag[1] = s->_mag.y;
	mag[2] = s->_mag.z;
	pthread_mutex_unlock(&s->_lock);
}

/**
 * Used by flight controller to turn on a led
 */
void SITLInterface::led_on(struct fc_sitl_client_interface *self, uint8_t led, bool on){
	SITLInterface *s = (SITLInterface*)self->data;
	if(led > 3) return;
	pthread_mutex_lock(&s->_lock);
	s->_leds[led] = on;
	pthread_mutex_unlock(&s->_lock);
}

/**
 * Used by flight controller to toggle a led
 */
void SITLInterface::led_toggle(struct fc_sitl_client_interface *self, uint8_t led){
	SITLInterface *s = (SITLInterface*)self->data;
	if(led > 3) return;
	pthread_mutex_lock(&s->_lock);
	s->_leds[led] = !s->_leds[led];
	pthread_mutex_unlock(&s->_lock);
}

/**
 * Used by SITL to read pwm values
 */
uint16_t SITLInterface::read_pwm(uint8_t chan){
	if(chan > FC_SITL_PWM_CHANNELS) return 1000;
	pthread_mutex_lock(&_lock);
	uint16_t pwm = _pwm[chan];
	pthread_mutex_unlock(&_lock);
	return pwm;
}

/**
 * Used by SITL to send latest range readings to flight controller
 * @param deg direction of the reading
 * @param range distance of the reading in cm
 */
void SITLInterface::write_range(uint16_t deg, uint16_t range){
	if(deg > 360) deg = 360;
	int i = deg / (360 / SITL_RANGE_SENSOR_COUNT);
	pthread_mutex_lock(&_lock);
	_range[i] = range;
	pthread_mutex_unlock(&_lock);
}

/**
 * Used by flight controller to read range sensor.
 */
int SITLInterface::read_range(struct fc_sitl_client_interface *self, uint16_t deg, uint16_t *range){
	SITLInterface *s = (SITLInterface*)self->data;
	if(deg > 360) deg = 360;
	int i = deg / (360 / SITL_RANGE_SENSOR_COUNT);
	pthread_mutex_lock(&s->_lock);
	uint16_t r = s->_range[i];
	pthread_mutex_unlock(&s->_lock);
	if(r == -1) return -1;
	*range = r;
	return 0;
}

/**
 * Used by SITL to get rotation as a quaternion
 */
glm::quat SITLInterface::get_rotation(){
	pthread_mutex_lock(&_lock);
	glm::quat qr(cosf(_euler[0]/2.0f), sinf(_euler[0] / 2.0f), 0.0f, 0.0f);
	glm::quat qp(cosf(_euler[1]/2.0f), 0.0f, sinf(_euler[1] / 2.0f), 0.0f);
	glm::quat qy(cosf(_euler[2]/2.0f), 0.0f, 0.0f, sinf(_euler[2] / 2.0f));
	glm::quat res = qy * qp * qr;
	pthread_mutex_unlock(&_lock);
	return res;
}

/**
 * Used by flight controller to update euler angles
 * @param roll roll angle in decidegrees
 * @param pitch pitch angle in decidegrees
 * @param yaw yaw angle in decidegrees
 */
void SITLInterface::write_euler_angles(struct fc_sitl_client_interface *self, int16_t roll, int16_t pitch, int16_t yaw){
	SITLInterface *s = (SITLInterface*)self->data;
	pthread_mutex_lock(&s->_lock);
	s->_euler[0] = glm::radians(0.1f * roll);
	s->_euler[1] = glm::radians(0.1f * pitch);
	s->_euler[2] = glm::radians(0.1f * yaw);
	pthread_mutex_unlock(&s->_lock);
}

/**
 * Used by flight controller to write latest motor and servo pwm values to sitl
 */
void SITLInterface::write_pwm(struct fc_sitl_client_interface *self, int8_t chan, uint16_t value){
	SITLInterface *s = (SITLInterface*)self->data;
	if(chan > FC_SITL_PWM_CHANNELS) return;
	pthread_mutex_lock(&s->_lock);
	s->_pwm[chan] = value;
	pthread_mutex_unlock(&s->_lock);
}

/** @} */
/** @} */
