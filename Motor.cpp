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

#include "Motor.h"

Motor::Motor(const glm::vec3 &pos, Motor::motor_dir_t dir, float power){
	_pos = pos; 
	_spin_dir = dir; 
	_power = power; 
	_rotation = glm::quat(1, 0, 0, 0); 
}

void Motor::setThrust(float value){
	if(value < 0) value = 0; if(value > 1.0) value = 1.0; 
	glm::vec3 t = _rotation * glm::vec3(0, value, 0); 
	_thrust = t * _power; 
	// calculate torque as perpendecular to motor position vector from center of frame and dependent on spin direction
	if(_spin_dir == MOTOR_CW)
		_torque = glm::cross(glm::normalize(_pos), t) * _power * 0.4f; 
	else if(_spin_dir == MOTOR_CCW)
		_torque = -glm::cross(glm::normalize(_pos), t) * _power * 0.4f; 
}

const glm::vec3 &Motor::getPosition(){
	return _pos; 
}

void Motor::setPower(float power){
	_power = power; 
}

void Motor::setRotation(const glm::quat &q){
	_rotation = q; 
}

const glm::vec3 &Motor::getThrustVector(){
	return _thrust; 
}

const glm::vec3 &Motor::getTorqueVector(){
	return _torque; 
}


