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
#pragma once

#include <glm/glm.hpp>
#include <glm/vec3.hpp>
#include <glm/gtc/quaternion.hpp>

class Motor {
public: 
	typedef enum {
		MOTOR_CW = 0, 
		MOTOR_CCW = 1
	} motor_dir_t; 

	Motor(const glm::vec3 &pos, Motor::motor_dir_t dir, float power = 20.0f); 

	const glm::vec3 &getPosition(); 
	const glm::vec3 &getThrustVector(); 
	const glm::vec3 &getTorqueVector(); 

	void setPower(float power); 
	void setThrust(float value); 
	void setRotation(const glm::quat &q); 
private:
	// motor position relative to body
	glm::vec3 _pos; 

	// motor spin direction (used for calculating torque)
	Motor::motor_dir_t _spin_dir; 
	
	// motor rotation relative to body frame
	glm::quat _rotation;  

	// motor thrust direction
	glm::vec3 _thrust; 
	
	// motor torque direction (perpendecular to thrust and defined by _spin_dir)
	glm::vec3 _torque; 

	// motor power in newtons
	float _power; 
};
