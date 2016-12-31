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
#include <unistd.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <stdio.h>

#include "SITLInterface.h"
#include "fc_sitl.h"

#define SITL_EEPROM_PAGE_SIZE 4096
#define SITL_EEPROM_NUM_PAGES 1
#define SITL_DATAFLASH_PAGE_SIZE 512
#define SITL_DATAFLASH_NUM_PAGES 4096


// since _XOPEN_SOURCE (or posix 2008) usleep is deprecated and nanosleep should be used instead.
#if _XOPEN_SOURCE > 500
int usleep(uint32_t us){
	struct timespec req = {
		.tv_sec = (__time_t)(us / 1000000UL),
		.tv_nsec = (__time_t)((us % 1000000UL) * 1000)
	};
	struct timespec rem;
	return nanosleep(&req, &rem);
}
#endif

#include <string.h>
static int32_t _micros(const struct system_calls_time *time){
	(void)time;
	struct timespec ts;
	static struct timespec start_ts = {0, 0};
	clock_gettime(CLOCK_MONOTONIC, &ts);
	if(start_ts.tv_sec == 0) memcpy(&start_ts, &ts, sizeof(start_ts));
	int32_t t = (ts.tv_sec - start_ts.tv_sec) * 1000000 + ts.tv_nsec / 1000;
	//printf("read time: %d\n", t);
	return t;
}

SITLInterface* SITLInterface::_load_sitl(const char *dlname){
	SITLInterface *self = new SITLInterface();

	self->eeprom_fd = open("sitl_eeprom.bin", O_RDWR | O_CREAT, 0644);
	posix_fallocate(self->eeprom_fd, 0, SITL_EEPROM_PAGE_SIZE * SITL_EEPROM_NUM_PAGES);
	self->dataflash_fd = open("sitl_dataflash.bin", O_RDWR | O_CREAT, 0644);
	posix_fallocate(self->dataflash_fd, 0, SITL_DATAFLASH_PAGE_SIZE * SITL_DATAFLASH_NUM_PAGES);

	struct system_calls calls;
	calls.pwm.write_motor = _write_motor;
	calls.pwm.write_servo = _write_servo;
	calls.pwm.read_ppm = _read_ppm;
	calls.pwm.read_pwm = _read_pwm;
	calls.imu.read_gyro = _read_gyro;
	calls.imu.read_acc = _read_accel;
	calls.imu.read_pressure = _read_pressure;
	calls.imu.read_temperature = _read_temperature;
	calls.leds.on = _led_on;
	calls.leds.toggle = _led_toggle;
	calls.beeper.on = _beeper;
	calls.time.micros = _micros;
	calls.eeprom.read = _eeprom_read;
	calls.eeprom.write = _eeprom_write;
	calls.eeprom.erase_page = _eeprom_erase_page;
	calls.eeprom.get_info = _eeprom_get_info;
	calls.logger.write = _logger_write;
	calls.range.read_range = _read_range;

	memcpy(&self->system, &calls, sizeof(calls));

	struct fc_sitl_server_interface *s = fc_sitl_create_instance(dlname, &self->system);
	if(!s) {
		delete self;
		return NULL;
	}
	return self;
}

#include <string.h>

SITLInterface::SITLInterface(){
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

template<class P, class M>
size_t _offsetof(const M P::*member)
{
    return (size_t) &( reinterpret_cast<P*>(0)->*member);
}

template<class P, class M>
P* container_of(M* ptr, const M P::*member)
{
    return (P*)( (char*)ptr - _offsetof(member));
}

template<class P, class M>
const P* container_of(const M* ptr, const M P::*member)
{
    return (const P*)( (const char*)ptr - _offsetof(member));
}

#define constrainf(var, a, b) (((var) < a)?(a):(((var) > (b))?(b):(var)))

typedef struct system_calls scall_t;

/**
 * Used by flight controller to write latest motor and servo pwm values to sitl
 */
void SITLInterface::_write_motor(const struct system_calls_pwm *pwm, uint8_t chan, uint16_t value){
	const struct system_calls *sys = container_of(pwm, &scall_t::pwm);
	SITLInterface *s = (SITLInterface*)container_of(sys, &SITLInterface::system);
	if(chan > FC_SITL_PWM_CHANNELS) return;
	pthread_mutex_lock(&s->_lock);
	s->_pwm[chan] = value;
	pthread_mutex_unlock(&s->_lock);
}

void SITLInterface::_write_servo(const struct system_calls_pwm *pwm, uint8_t chan, uint16_t value){
	_write_motor(pwm, chan + 8, value);
}

/**
 * Used by flight controller to read latest pwm values
 */
uint16_t SITLInterface::_read_pwm(const struct system_calls_pwm *pwm, uint8_t chan){
	const struct system_calls *sys = container_of(pwm, &scall_t::pwm);
	SITLInterface *s = (SITLInterface*)container_of(sys, &SITLInterface::system);
	if(chan > FC_SITL_PWM_CHANNELS) return 1000;
	pthread_mutex_lock(&s->_lock);
	uint16_t rc = s->_rc[chan];
	pthread_mutex_unlock(&s->_lock);
	return rc;
}

uint16_t SITLInterface::_read_ppm(const struct system_calls_pwm *pwm, uint8_t chan){
	return _read_pwm(pwm, chan);
}

/**
 * Used by flight controller to read latest gyro values
 */
int SITLInterface::_read_gyro(const struct system_calls_imu *imu, int16_t output[3]){
	const struct system_calls *sys = container_of(imu, &scall_t::imu);
	SITLInterface *s = (SITLInterface*)container_of(sys, &SITLInterface::system);
	float gyr[3];

	pthread_mutex_lock(&s->_lock);
	gyr[0] = s->_gyro.x;
	gyr[1] = s->_gyro.y;
	gyr[2] = s->_gyro.z;
	pthread_mutex_unlock(&s->_lock);

	// convert into standard gyro readings to be used by the fc
	static const float scale = 3.14f * SYSTEM_GYRO_RANGE / 180;
	int16_t maxrange = 0x7fff;
	// scale gyro to int16 range. 35.0f is 2000 deg/s max gyro range in radians.
	// incoming gyro data from sim is in rad/s
	output[0] = constrainf(gyr[0] / scale, -1.0f, 1.0f) * maxrange;
	output[1] = constrainf(gyr[1] / scale, -1.0f, 1.0f) * maxrange;
	output[2] = constrainf(gyr[2] / scale, -1.0f, 1.0f) * maxrange;

	return 0;
}

/**
 * Used by flight controller to read latest accelerometer values
 */
int SITLInterface::_read_accel(const struct system_calls_imu *imu, int16_t output[3]){
	const struct system_calls *sys = container_of(imu, &scall_t::imu);
	SITLInterface *s = (SITLInterface*)container_of(sys, &SITLInterface::system);

	float accel[3];

	pthread_mutex_lock(&s->_lock);
	accel[0] = s->_accel.x;
	accel[1] = s->_accel.y;
	accel[2] = s->_accel.z;
	pthread_mutex_unlock(&s->_lock);

	// convert to integer values to be used by fc
	output[0] = (accel[0] / 9.82f) * SYSTEM_ACC_1G;
	output[1] = (accel[1] / 9.82f) * SYSTEM_ACC_1G;
	output[2] = (accel[2] / 9.82f) * SYSTEM_ACC_1G;

	return 0;
}

/**
 * Used by flight controller to read latest compass values
 */
int SITLInterface::_read_mag(const struct system_calls_imu *imu, int16_t output[3]){
	const struct system_calls *sys = container_of(imu, &scall_t::imu);
	SITLInterface *s = (SITLInterface*)container_of(sys, &SITLInterface::system);

	float mag[3];
	pthread_mutex_lock(&s->_lock);
	mag[0] = s->_mag.x;
	mag[1] = s->_mag.y;
	mag[2] = s->_mag.z;
	pthread_mutex_unlock(&s->_lock);

	// TODO: make sure we are outputing valid magnetometer readings
	output[0] = mag[0];
	output[1] = mag[1];
	output[2] = mag[2];

	return 0;
}

/**
 * Used by flight controller to turn on a led
 */
void SITLInterface::_led_on(const struct system_calls_leds *leds, uint8_t led, bool on){
	const struct system_calls *sys = container_of(leds, &scall_t::leds);
	SITLInterface *s = (SITLInterface*)container_of(sys, &SITLInterface::system);

	if(led > 3) return;
	pthread_mutex_lock(&s->_lock);
	s->_leds[led] = on;
	pthread_mutex_unlock(&s->_lock);
}

/**
 * Used by flight controller to toggle a led
 */
void SITLInterface::_led_toggle(const struct system_calls_leds *leds, uint8_t led){
	const struct system_calls *sys = container_of(leds, &scall_t::leds);
	SITLInterface *s = (SITLInterface*)container_of(sys, &SITLInterface::system);

	if(led > 3) return;
	pthread_mutex_lock(&s->_lock);
	s->_leds[led] = !s->_leds[led];
	pthread_mutex_unlock(&s->_lock);
}

/**
 * Used by flight controller to turn beeper on and off
 */
void SITLInterface::_beeper(const struct system_calls_beeper *beep, bool on){
	const struct system_calls *sys = container_of(beep, &scall_t::beeper);
	SITLInterface *s = (SITLInterface*)container_of(sys, &SITLInterface::system);

	pthread_mutex_lock(&s->_lock);
	s->_beep = on;
	pthread_mutex_unlock(&s->_lock);
}

int SITLInterface::_eeprom_read(const struct system_calls_bdev *eeprom, void *dst, uint16_t addr, size_t size){
	const struct system_calls *sys = container_of(eeprom, &scall_t::eeprom);
	SITLInterface *s = (SITLInterface*)container_of(sys, &SITLInterface::system);
	printf("EEPROM read from %04x, size %lu\n", addr, size);
	/*if((addr + size) > sizeof(_flash)){
		size = sizeof(_flash) - addr;
	}*/
	lseek(s->eeprom_fd, addr, SEEK_SET);
	return read(s->eeprom_fd, dst, size);
}

int SITLInterface::_eeprom_write(const struct system_calls_bdev *eeprom, uint16_t addr, const void *data, size_t size){
	const struct system_calls *sys = container_of(eeprom, &scall_t::eeprom);
	SITLInterface *s = (SITLInterface*)container_of(sys, &SITLInterface::system);
	printf("EEPROM write to %04x, size %lu\n", addr, size);
	lseek(s->eeprom_fd, addr, SEEK_SET);
	return write(s->eeprom_fd, data, size);
}

int SITLInterface::_eeprom_erase_page(const struct system_calls_bdev *eeprom, uint16_t addr){
	const struct system_calls *sys = container_of(eeprom, &scall_t::eeprom);
	SITLInterface *s = (SITLInterface*)container_of(sys, &SITLInterface::system);
	uint8_t page[SITL_EEPROM_PAGE_SIZE];
	memset(page, 0xff, sizeof(page));
	addr = (addr / SITL_EEPROM_PAGE_SIZE) * SITL_EEPROM_PAGE_SIZE;
	// erase page
	lseek(s->eeprom_fd, addr, SEEK_SET);
	int ret = write(s->eeprom_fd, page, sizeof(page));
	if(ret < 0) return -1;
	return !!ret;
}

void SITLInterface::_eeprom_get_info(const struct system_calls_bdev *self, struct system_bdev_info *info){
	(void)self;
	info->page_size = SITL_EEPROM_PAGE_SIZE;
	info->num_pages = SITL_EEPROM_NUM_PAGES;
}

int16_t SITLInterface::_logger_write(const struct system_calls_logger *logger, const void *data, int16_t size){
	const struct system_calls *sys = container_of(logger, &scall_t::logger);
	SITLInterface *s = (SITLInterface*)container_of(sys, &SITLInterface::system);
	int ret = write(s->dataflash_fd, data, size);
	//usleep(10000);
	if(ret < 0) return -1;
	return ret;
}

/**
 * Used by flight controller to read range sensor.
 */
int SITLInterface::_read_range(const struct system_calls_range *range, uint16_t deg, uint16_t *output){
	const struct system_calls *sys = container_of(range, &scall_t::range);
	SITLInterface *s = (SITLInterface*)container_of(sys, &SITLInterface::system);
	if(deg > 360) deg = 360;
	int i = deg / (360 / SITL_RANGE_SENSOR_COUNT);
	pthread_mutex_lock(&s->_lock);
	uint16_t r = s->_range[i];
	pthread_mutex_unlock(&s->_lock);
	if(r == -1) return -1;
	*output = r;
	return 0;
}

int SITLInterface::_read_pressure(const struct system_calls_imu *imu, uint32_t *pressure){
	const struct system_calls *sys = container_of(imu, &scall_t::imu);
	SITLInterface *s = (SITLInterface*)container_of(sys, &SITLInterface::system);
	pthread_mutex_lock(&s->_lock);
	*pressure = s->_pressure;
	pthread_mutex_unlock(&s->_lock);
	return 0;
}

int SITLInterface::_read_temperature(const struct system_calls_imu *imu, int16_t *temperature){
	const struct system_calls *sys = container_of(imu, &scall_t::imu);
	SITLInterface *s = (SITLInterface*)container_of(sys, &SITLInterface::system);
	pthread_mutex_lock(&s->_lock);
	*temperature = s->_temperature;
	pthread_mutex_unlock(&s->_lock);
	return 0;
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
 * Used by SITL to read beeper state
 */
bool SITLInterface::read_beeper(){
	pthread_mutex_lock(&_lock);
	bool res = _beep;
	pthread_mutex_unlock(&_lock);
	return res;
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
 * Used by SITL to send latest pressure to flight controller
 */
void SITLInterface::write_pressure(uint32_t pressure){
	pthread_mutex_lock(&_lock);
	_pressure = pressure;
	pthread_mutex_unlock(&_lock);
}

void SITLInterface::write_temperature(int16_t temperature){
	pthread_mutex_lock(&_lock);
	_temperature = temperature;
	pthread_mutex_unlock(&_lock);
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
/*
void SITLInterface::write_euler_angles(struct fc_sitl_client_interface *self, int16_t roll, int16_t pitch, int16_t yaw){
	SITLInterface *s = (SITLInterface*)self->data;
	pthread_mutex_lock(&s->_lock);
	s->_euler[0] = glm::radians(0.1f * roll);
	s->_euler[1] = glm::radians(0.1f * pitch);
	s->_euler[2] = glm::radians(0.1f * yaw);
	pthread_mutex_unlock(&s->_lock);
}
*/
/** @} */
/** @} */
