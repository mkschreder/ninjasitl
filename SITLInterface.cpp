#include "SITLInterface.h"
#include "fc_sitl.h"

uint16_t _sitl_read_rc(struct fc_sitl_client_interface *self, uint8_t chan){
	SITLInterface *s = (SITLInterface*)self->data;
	return s->read_rc(chan);
}

void _sitl_read_gyro(struct fc_sitl_client_interface *self, float gyro[3]){
	SITLInterface *s = (SITLInterface*)self->data;
	s->read_gyro(gyro);
}

void _sitl_read_accel(struct fc_sitl_client_interface *self, float accel[3]){
	SITLInterface *s = (SITLInterface*)self->data;
	s->read_accel(accel);
}

void _sitl_read_mag(struct fc_sitl_client_interface *self, float mag[3]){
	SITLInterface *s = (SITLInterface*)self->data;
	s->read_mag(mag);
}

void _sitl_write_pwm(struct fc_sitl_client_interface *self, int8_t chan, uint16_t value){
	SITLInterface *s = (SITLInterface*)self->data;
	s->write_pwm(chan, value);
}

void _sitl_update_euler(struct fc_sitl_client_interface *self, int16_t r, int16_t p, int16_t y){
	SITLInterface *s = (SITLInterface*)self->data;
	s->update_euler_angles(r, p, y);
}

void _sitl_led_on(struct fc_sitl_client_interface *self, uint8_t chan, bool value){
	SITLInterface *s = (SITLInterface*)self->data;
	s->led_on(chan, value);
}

void _sitl_led_toggle(struct fc_sitl_client_interface *self, uint8_t chan){
	SITLInterface *s = (SITLInterface*)self->data;
	s->led_toggle(chan);
}


SITLInterface* _load_sitl(const char *dlname){
	struct fc_sitl_client_interface *cl = (struct fc_sitl_client_interface*)calloc(1, sizeof(struct fc_sitl_client_interface));
	cl->read_rc = _sitl_read_rc;
	cl->read_gyro = _sitl_read_gyro;
	cl->read_accel = _sitl_read_accel;
	cl->read_mag = _sitl_read_mag;
	cl->write_pwm = _sitl_write_pwm;
	cl->led_on = _sitl_led_on;
	cl->led_toggle = _sitl_led_toggle;
	cl->update_euler_angles = _sitl_update_euler;
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
	memset(_leds, 0, sizeof(_leds));
}

SITLInterface* SITLInterface::create(sitl_controller_type_t type){
	switch(type){
		case SITL_NINJAFLIGHT:
			return _load_sitl("./fc_ninjaflight.so");
	}
	return NULL;
}

void SITLInterface::write_accel(float x, float y, float z){
	_accel.x = x;
	_accel.y = y;
	_accel.z = z;
}

void SITLInterface::write_gyro(float x, float y, float z){
	_gyro.x = x;
	_gyro.y = y;
	_gyro.z = z;
}

void SITLInterface::write_rc(uint8_t chan, float value){
	if(chan > FC_SITL_PWM_CHANNELS) return;
	_rc[chan] = value;
}

uint16_t SITLInterface::read_rc(uint8_t chan){
	if(chan > FC_SITL_PWM_CHANNELS) return 1000;
	return _rc[chan];
}

void SITLInterface::read_gyro(float gyro[3]){
	gyro[0] = _gyro.x;
	gyro[1] = _gyro.y;
	gyro[2] = _gyro.z;
}

void SITLInterface::read_accel(float accel[3]){
	accel[0] = _accel.x;
	accel[1] = _accel.y;
	accel[2] = _accel.z;
}

void SITLInterface::read_mag(float mag[3]){
	mag[0] = _mag.x;
	mag[1] = _mag.y;
	mag[2] = _mag.z;
}

void SITLInterface::led_on(uint8_t led, bool on){
	if(led > 3) return;
	_leds[led] = on;
}

void SITLInterface::led_toggle(uint8_t led){
	if(led > 3) return;
	_leds[led] = !_leds[led];
}

bool SITLInterface::get_led(uint8_t led){
	return _leds[led];
}

uint16_t SITLInterface::read_pwm(uint8_t chan){
	if(chan > FC_SITL_PWM_CHANNELS) return 1000;
	return _pwm[chan];
}

glm::quat SITLInterface::get_rotation(){
	glm::quat qr(cosf(_euler[0]/2.0f), sinf(_euler[0] / 2.0f), 0.0f, 0.0f);
	glm::quat qp(cosf(_euler[1]/2.0f), 0.0f, sinf(_euler[1] / 2.0f), 0.0f);
	glm::quat qy(cosf(_euler[2]/2.0f), 0.0f, 0.0f, sinf(_euler[2] / 2.0f));
	return qy * qp * qr;
}

void SITLInterface::update_euler_angles(int16_t roll, int16_t pitch, int16_t yaw){
	_euler[0] = glm::radians(0.1f * roll);
	_euler[1] = glm::radians(0.1f * pitch);
	_euler[2] = glm::radians(0.1f * yaw);
}

void SITLInterface::write_pwm(int8_t chan, uint16_t value){
	if(chan > FC_SITL_PWM_CHANNELS) return;
	_pwm[chan] = value;
}


