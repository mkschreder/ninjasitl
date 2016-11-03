#pragma once
#include "fc_sitl.h"
#include <stdint.h>
#include <glm/glm.hpp>

typedef enum {
	SITL_NINJAFLIGHT
} sitl_controller_type_t;

class SITLInterface {
	friend uint16_t _sitl_read_rc(struct fc_sitl_client_interface *self, uint8_t chan);
	friend void _sitl_read_gyro(struct fc_sitl_client_interface *self, float gyro[3]);
	friend void _sitl_read_accel(struct fc_sitl_client_interface *self, float accel[3]);
	friend void _sitl_read_mag(struct fc_sitl_client_interface *self, float mag[3]);
	friend void _sitl_write_pwm(struct fc_sitl_client_interface *self, int8_t chan, uint16_t value);
	friend SITLInterface* _load_sitl(const char *dlname);
public:
	static SITLInterface *create(sitl_controller_type_t type);
	void write_accel(float x, float y, float z);
	void write_gyro(float x, float y, float z);
	void write_rc(uint8_t chan, float value);
	uint16_t read_pwm(uint8_t chan);
protected:
	SITLInterface(struct fc_sitl_client_interface *client);
	uint16_t read_rc(uint8_t chan);
	void read_gyro(float gyro[3]);
	void read_accel(float accel[3]);
	void read_mag(float mag[3]);
	void write_pwm(int8_t chan, uint16_t value);
private:
	glm::vec3 _gyro, _accel, _mag;
	uint16_t _rc[FC_SITL_PWM_CHANNELS];
	uint16_t _pwm[FC_SITL_PWM_CHANNELS];
	struct fc_sitl_client_interface *client; // reference to client interface that is pointing back to us
};
