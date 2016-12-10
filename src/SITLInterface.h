#pragma once
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
	void write_rc(uint8_t chan, float value);
	void write_range(uint16_t deg, uint16_t distance);
	uint16_t read_pwm(uint8_t chan);
	bool read_led(uint8_t id);
	glm::quat get_rotation();
protected:
	SITLInterface(struct fc_sitl_client_interface *client);
	~SITLInterface();
	static uint16_t read_rc(struct fc_sitl_client_interface *self, uint8_t chan);
	static void read_gyro(struct fc_sitl_client_interface *self, float gyro[3]);
	static void read_accel(struct fc_sitl_client_interface *self, float accel[3]);
	static void read_mag(struct fc_sitl_client_interface *self, float mag[3]);
	static int read_range(struct fc_sitl_client_interface *self, uint16_t deg, uint16_t *range);
	static void led_on(struct fc_sitl_client_interface *self, uint8_t led, bool on);
	static void led_toggle(struct fc_sitl_client_interface *self, uint8_t led);
	static void write_pwm(struct fc_sitl_client_interface *self, int8_t chan, uint16_t value);
	static void write_euler_angles(struct fc_sitl_client_interface *self, int16_t roll, int16_t pitch, int16_t yaw);
private:
	glm::vec3 _gyro, _accel, _mag;
	uint16_t _rc[FC_SITL_PWM_CHANNELS];
	uint16_t _pwm[FC_SITL_PWM_CHANNELS];
	uint16_t _range[SITL_RANGE_SENSOR_COUNT];
	float _euler[3];
	bool _leds[3];
	struct fc_sitl_client_interface *client; // reference to client interface that is pointing back to us
	pthread_mutex_t _lock;
};
