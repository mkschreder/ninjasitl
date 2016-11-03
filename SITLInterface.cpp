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

SITLInterface* _load_sitl(const char *dlname){
	struct fc_sitl_client_interface *cl = (struct fc_sitl_client_interface*)calloc(1, sizeof(struct fc_sitl_client_interface));
	cl->read_rc = _sitl_read_rc;
	cl->read_gyro = _sitl_read_gyro;
	cl->read_accel = _sitl_read_accel;
	cl->read_mag = _sitl_read_mag;
	cl->write_pwm = _sitl_write_pwm;
	cl->data = new SITLInterface(cl);
	struct fc_sitl_server_interface *s = fc_sitl_create_instance(dlname, cl);
	if(!s) {
		delete ((SITLInterface*)cl->data);
		free(cl);
		return NULL;
	}
	return (SITLInterface*)cl->data;
}

SITLInterface::SITLInterface(struct fc_sitl_client_interface *client){
	this->client = client;
	for(int c = 0; c < FC_SITL_PWM_CHANNELS; c++){
		this->_rc[c] = 1500;
		this->_pwm[c] = 1000;
	}
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

uint16_t SITLInterface::read_pwm(uint8_t chan){
	if(chan > FC_SITL_PWM_CHANNELS) return 1000;
	return _pwm[chan];
}

void SITLInterface::write_pwm(int8_t chan, uint16_t value){
	if(chan > FC_SITL_PWM_CHANNELS) return;
	_pwm[chan] = value;
}


