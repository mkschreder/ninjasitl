#pragma once

#include <stdint.h>

#define FC_SITL_PWM_CHANNELS 8

// this is data sent to us by a sitl that supports server mode simulation (if
// mode is set to SERVER in data frames received from sitl)
struct fc_sitl_server_sim_data {
	float pos[3];
	float vel[3];
	float euler[3];
	float acc[3];
	float mag[3];
};

struct fc_sitl_pwm_values {
	uint16_t channel[FC_SITL_PWM_CHANNELS];
};

// packets sent from flight controller to simulator
struct fc_sitl_server_packet {
	uint8_t version; // protocol version (must match with client)
	unsigned long long time; // timestamp of each packet in ms
	// simulation mode (CLIENT or SERVER). Arducopter has server
	// sitl mode where simplified simulation is done on the sitl and client only
	// renders the 3d model. Default is CLIENT so that we do simulation in bullet
	// physics.  uint8_t frame;
	uint8_t mode;
	// pwm outputs in PWM values (1000-2000) for each pwm channel
	uint16_t pwmout[8];
	struct fc_sitl_server_sim_data simdata;
};

// packets sent from simulator to flight controller
// all coordinates are in flight dynamics coordinate frame (x forward, y right, z down)
struct fc_sitl_client_packet {
	uint8_t version; // protocol version (must match with client)
	unsigned long long time; // timestamp of each packet in ms
	// gyro readings in radians/s^2
	float gyro[3];
	// accelerometer readings in m/s/s
	float accel[3];
	// magnetometer readings in gauss
	float mag[3];
	// velocity information in ms/s
	float vel[3];
	// rc input values as a pwm range 1000-2000
	// mapping: roll, pitch, throttle, yaw, aux1, aux2, aux3, aux4
	int16_t rcin[FC_SITL_PWM_CHANNELS];
	// gps location as lat/lon/heading information
	float loc[3];
};

struct fc_sitl_client_interface {
	uint16_t (*read_rc)(struct fc_sitl_client_interface *self, uint8_t chan);
	void (*read_gyro)(struct fc_sitl_client_interface *self, float gyro[3]);
	void (*read_accel)(struct fc_sitl_client_interface *self, float accel[3]);
	void (*read_mag)(struct fc_sitl_client_interface *self, float mag[3]);
	void (*write_pwm)(struct fc_sitl_client_interface *self, int8_t chan, uint16_t value);
	void *data;
};

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
