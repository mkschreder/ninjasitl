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

#include "Application.h"
#include "Aircraft.h"
#include "MagneticField.h"

using namespace irr; 
using namespace scene; 
using namespace core;

Aircraft::Aircraft(Application *app) : 
	_application(app){

	_velocity = glm::vec3(0, 0, 0); 
	_linear_acceleration = glm::vec3(0, 0, 0); 
	_accel = glm::vec3(0, 0, 0); 
	_mag = glm::vec3(0, 0, 0); 
	_location = glm::ivec3(0, 0, 0); 
	_home = glm::ivec3(0, 0, 0); 
}
 
void Aircraft::init(){
	setupMotors(); 

	_frame_body = createFrameRigidBody(); 
	_frame_node = createFrameSceneNode(); 
}

void Aircraft::updateLocation(){
	glm::vec3 position = getPosition(); 

	// scaling factor from 1e-7 degrees to meters at equater
	// == 1.0e-7 * DEG_TO_RAD * RADIUS_OF_EARTH
	#define LOCATION_SCALING_FACTOR 0.011131884502145034f
	// inverse of LOCATION_SCALING_FACTOR
	#define LOCATION_SCALING_FACTOR_INV 89.83204953368922f

	float lat = _home.z; 
	float lng = _home.x; 
	float scale = cosf(glm::radians(lat * 1.0e-7f));
	if(scale < 0.01f) scale = 0.01f; else if(scale > 1.0f) scale = 1.0f; 

	int32_t dlat = position.z * LOCATION_SCALING_FACTOR_INV;
	int32_t dlng = (position.x * LOCATION_SCALING_FACTOR_INV) / scale;

	_location.x = lng + dlng;
	_location.y = _home.y + position.y * 100.0f; 
	_location.z = lat + dlat;
}

glm::vec3 Aircraft::calcAcceleration(){
	//return glm::inverse(getRotation()) * (_linear_acceleration + glm::vec3(0, 9.82, 0)); 
	return glm::inverse(getRotation()) * (glm::vec3(0, 9.82, 0)); 
}

glm::vec3 Aircraft::calcMagFieldIntensity(){
	glm::quat rot = getRotation(); 

	// get the magnetic field intensity and orientation
	float intensity;
	float declination;                  
	float inclination;
	MagneticField::get_field_from_location_ef(_location.z*1e-7f,_location.x*1e-7f,intensity,declination,inclination);

	// create a field vector and rotate to the required orientation
	glm::vec3 mag_ef(0, 0, 1e3f * intensity); 
	glm::quat mrot = 
		glm::angleAxis(glm::radians(declination), glm::vec3(0, 1, 0)) *    
		glm::angleAxis(glm::radians(inclination), glm::vec3(1, 0, 0));  

	return glm::inverse(rot) * (mrot * mag_ef);                

	/*
	// calculate frame height above ground   
	float frame_height_agl = fmaxf(position.y, 0.0f);
	  
	// calculate scaling factor that varies from 1 at ground level to 1/8 at sitl->mag_anomaly_hgt
	// Assume magnetic anomaly strength scales with 1/R**3
	float mag_anomaly_hgt = 0.0; 
	glm::vec3 mag_anomaly_ned(0.0, 0.0, 0.0); 
	float anomaly_scaler = (mag_anomaly_hgt / (frame_height_agl + mag_anomaly_hgt));
	anomaly_scaler = anomaly_scaler * anomaly_scaler * anomaly_scaler;
	  
	// add scaled anomaly to earth field
	mag_ef += mag_anomaly_ned * anomaly_scaler;
	 */ 
	//_mag = mag_ef; 
	  
	//::printf("mag: %f %f %f\n", _mag.x, _mag.y, _mag.z); 
	// add motor interference           
	//_mag += mag_mot * battery_current;
}

glm::vec3 Aircraft::getPosition(){
	btVector3 pos = _frame_body->getCenterOfMassPosition();
	return glm::vec3(pos[0], pos[1], pos[2]); 
}

glm::ivec3 Aircraft::getLocation(){
	return _location; 
}

glm::vec3 Aircraft::getMagneticField(){
	return _mag; 
}

glm::quat Aircraft::getRotation(){
	const btQuaternion& quat = _frame_body->getOrientation();
	return glm::quat(quat.w(), quat.x(), quat.y(), quat.z()); 
}

void Aircraft::setPosition(const glm::vec3 &pos){
	btTransform t;

	_frame_body->getMotionState()->getWorldTransform(t);
	t.setOrigin(btVector3(pos.x, pos.y, pos.z));
	_frame_body->setWorldTransform(t);
	_frame_body->getMotionState()->setWorldTransform(t);

	_frame_body->clearForces();
	_frame_body->setLinearVelocity(btVector3(0.0f,0.0f,0.0f));
	_frame_body->setAngularVelocity(btVector3(0.0f,0.0f,0.0f));
}

void Aircraft::setHomeLocation(const glm::vec3 &loc){
	_home = glm::ivec3(loc.x * 1.0e7, loc.y * 1e2, loc.z * 1.0e7); 
}
 
void Aircraft::setRotation(const glm::quat &rot){
	btTransform t = _frame_body->getWorldTransform();
	btQuaternion q(rot.x, rot.y, rot.z, rot.w); 
	t.setRotation(q); 
	_frame_body->setWorldTransform(t); 
}

void Aircraft::setAngularVelocity(const glm::vec3 &v){
	_frame_body->setAngularVelocity(btVector3(v.x, v.y, v.z)); 
}

void Aircraft::setLinearVelocity(const glm::vec3 &v){
	//if(_simulate)
		_frame_body->setLinearVelocity(btVector3(v.x, v.y, v.z)); 
	//else
	//	_velocity = v; 
}

void Aircraft::setAccelerometer(const glm::vec3 &v){
	_accel = v; 
}

void Aircraft::setMagneticField(const glm::vec3 &v){
	_mag = v; 
}

glm::vec3 Aircraft::getAccel(){
	return _accel; 
}

glm::vec3 Aircraft::getGyro(){
	glm::quat rot = getRotation(); 
	glm::vec3 lx = rot * glm::vec3(1, 0, 0); 
	glm::vec3 ly = rot * glm::vec3(0, 1, 0); 
	glm::vec3 lz = rot * glm::vec3(0, 0, 1); 

	btVector3 av = _frame_body->getAngularVelocity(); 

	float xr = lx.x * av[0] + lx.y * av[1] + lx.z * av[2]; 
	float yr = ly.x * av[0] + ly.y * av[1] + ly.z * av[2]; 
	float zr = lz.x * av[0] + lz.y * av[1] + lz.z * av[2]; 
	
	//btVector3 tmp = _frame_body->getAngularVelocity(); 
	// pitch, yaw, roll
	return glm::vec3(xr, yr, zr);
}

glm::vec3 Aircraft::getVelocity(){
	btVector3 v = _frame_body->getLinearVelocity();
	return glm::vec3(v[0], v[1], v[2]); 
}

void Aircraft::applyFrameForce(const glm::vec3 &force, const glm::vec3 &force_pos){
	glm::vec3 pos = getPosition(); 
	glm::quat rot = getRotation(); 

	// force is in body frame and position is relative to center of the body
	// so we need to rotate it into craft body frame 

	glm::vec3 p = rot * force_pos; 
	glm::vec3 f = rot * force; 
	
	// forces are applied to the frame rigid body
	_frame_body->applyForce(btVector3(f.x, f.y, f.z), btVector3(p.x, p.y, p.z)); 
}

void Aircraft::update(float dt){
	//btVector3 tot = _frame_body->getTotalForce(); 
	//printf("tot: %f %f %f\n", tot[0], tot[1], tot[2]); 
	// + frame
	
	updateForces(); 

	updateLocation(); 

	// update accel
	glm::vec3 vel = getVelocity(); 
	_linear_acceleration = (vel - _velocity) / dt; // / 0.018f; 
	_velocity = vel; 

	_accel = calcAcceleration(); 
	_mag = calcMagFieldIntensity(); 

	// update the scene node
	// Set position
	btVector3 p = _frame_body->getCenterOfMassPosition();
	_frame_node->setPosition(vector3df((f32)p[0], (f32)p[1], (f32)p[2]));

	// Set rotation
	vector3df euler;
	const btQuaternion& _q = _frame_body->getOrientation();
	quaternion q(_q.getX(), _q.getY(), _q.getZ(), _q.getW());
	q.toEuler(euler);
	euler *= RADTODEG;
	_frame_node->setRotation(euler);
}
