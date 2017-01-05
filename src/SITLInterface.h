#pragma once
#include "system_calls.h"
#include "fc_sitl.h"
#include <stdint.h>
#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

#define SITL_RANGE_SENSOR_COUNT 4

typedef enum {
	SITL_NINJAFLIGHT
} sitl_controller_type_t;

class SITLInterface {
	/*
	static uint16_t _sitl_read_rc(struct fc_sitl_client_interface *self, uint8_t chan);
	static void _sitl_read_gyro(struct fc_sitl_client_interface *self, float gyro[3]);
	static void _sitl_read_accel(struct fc_sitl_client_interface *self, float accel[3]);
	static void _sitl_read_mag(struct fc_sitl_client_interface *self, float mag[3]);
	static void _sitl_write_pwm(struct fc_sitl_client_interface *self, int8_t chan, uint16_t value);
	static void _sitl_write_euler(struct fc_sitl_client_interface *self, int16_t, int16_t, int16_t);
	static void _sitl_led_on(struct fc_sitl_client_interface *self, uint8_t, bool);
	static void _sitl_led_toggle(struct fc_sitl_client_interface *self, uint8_t);
	static int _sitl_read_range(struct fc_sitl_client_interface *self, uint16_t, uint16_t*);
	*/
	static SITLInterface* _load_sitl(const char *dlname);
public:
	static SITLInterface *create(sitl_controller_type_t type);

	void write_accel(float x, float y, float z);
	void write_gyro(float x, float y, float z);
	void write_pressure(uint32_t pressure);
	void write_temperature(int16_t temp);
	void write_rc(uint8_t chan, float value);
	void write_range(uint16_t deg, uint16_t distance);
	uint16_t read_pwm(uint8_t chan);
	bool read_led(uint8_t id);
	bool read_beeper();
	glm::quat get_rotation();
protected:
	SITLInterface();
	~SITLInterface();

	static void _write_motor(const struct system_calls_pwm *pwm, uint8_t chan, uint16_t value);
	static void _write_servo(const struct system_calls_pwm *pwm, uint8_t chan, uint16_t value);
	static uint16_t _read_pwm(const struct system_calls_pwm *pwm, uint8_t chan);
	static uint16_t _read_ppm(const struct system_calls_pwm *pwm, uint8_t chan);
	static int _gyro_sync(const struct system_calls_imu *imu);

	static int32_t _micros(const struct system_calls_time *time);
	static int _read_gyro(const struct system_calls_imu *imu, int16_t output[3]);
	static int _read_accel(const struct system_calls_imu *imu, int16_t output[3]);
	static int _read_mag(const struct system_calls_imu *imu, int16_t output[3]);
	static int _read_pressure(const struct system_calls_imu *imu, uint32_t *pressure);
	static int _read_temperature(const struct system_calls_imu *imu, int16_t *temperature);
	static void _led_on(const struct system_calls_leds *leds, uint8_t led, bool on);
	static void _led_toggle(const struct system_calls_leds *leds, uint8_t led);
	static void _beeper(const struct system_calls_beeper *beep, bool on);
	static int _eeprom_read(const struct system_calls_bdev *eeprom, void *dst, uint16_t addr, size_t size);
	static int _eeprom_write(const struct system_calls_bdev *eeprom, uint16_t addr, const void *data, size_t size);
	static int _eeprom_erase_page(const struct system_calls_bdev *eeprom, uint16_t addr);
	static void _eeprom_get_info(const struct system_calls_bdev *self, struct system_bdev_info *info);
	static int16_t _logger_write(const struct system_calls_logger *logger, const void *data, int16_t size);
	static int _read_range(const struct system_calls_range *range, uint16_t deg, uint16_t *output);

	/*
	static uint16_t read_rc(struct fc_sitl_client_interface *self, uint8_t chan);
	static void read_gyro(struct fc_sitl_client_interface *self, float gyro[3]);
	static void read_accel(struct fc_sitl_client_interface *self, float accel[3]);
	static void read_mag(struct fc_sitl_client_interface *self, float mag[3]);
	static int read_pressure(struct fc_sitl_client_interface *self, uint32_t *pressure);
	static int read_temperature(struct fc_sitl_client_interface *self, int16_t *temperature);
	static int read_range(struct fc_sitl_client_interface *self, uint16_t deg, uint16_t *range);
	static void led_on(struct fc_sitl_client_interface *self, uint8_t led, bool on);
	static void led_toggle(struct fc_sitl_client_interface *self, uint8_t led);
	static void beeper(struct fc_sitl_client_interface *self, uint8_t on);
	static void write_pwm(struct fc_sitl_client_interface *self, int8_t chan, uint16_t value);
	static void write_euler_angles(struct fc_sitl_client_interface *self, int16_t roll, int16_t pitch, int16_t yaw);
	*/
private:
	glm::vec3 _gyro, _accel, _mag;
	uint16_t _rc[FC_SITL_PWM_CHANNELS];
	uint16_t _pwm[FC_SITL_PWM_CHANNELS];
	uint16_t _range[SITL_RANGE_SENSOR_COUNT];
	uint32_t _pressure;
	int16_t _temperature;
	float _euler[3];
	bool _leds[3];
	bool _beep;
	struct fc_sitl_client_interface *client; // reference to client interface that is pointing back to us
	pthread_mutex_t _lock;
	int eeprom_fd;
	int dataflash_fd;
	struct system_calls system;
	struct timespec start_ts;
};
