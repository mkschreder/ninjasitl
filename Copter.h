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

#include <irrlicht.h>

#define GLM_FORCE_RADIANS

#include <glm/glm.hpp>
#include <glm/vec3.hpp>
#include <glm/mat3x3.hpp>
#include <glm/gtc/quaternion.hpp>

#include <btBulletCollisionCommon.h>
#include <btBulletDynamicsCommon.h>

//#include <FlightController.hpp>

using namespace glm; 

class Application;

class Copter {
public:
	Copter(Application *app);

	virtual void update(float dt);
	virtual void render(irr::video::IVideoDriver *drv);
	
	void setOutputThrust(unsigned int id, float thrust); 

	virtual irr::scene::ISceneNode *getSceneNode(){return mNode;}
	virtual void attachCamera(irr::scene::ICameraSceneNode *cam); 

	glm::quat getRotation(); 
	glm::vec3 getVelocity(); 
	glm::vec3 getPosition(); 
	glm::vec3 getAccel(); 
	glm::vec3 getGyro(); 
	
	void setPosition(const glm::vec3 &pos); 
	void setRotation(const glm::quat &rot); 
	void setAngularVelocity(const glm::vec3 &v); 
	void setLinearVelocity(const glm::vec3 &v); 
private:
	typedef enum {
		MOTOR_CW = 0, 
		MOTOR_CCW = 1
	} motor_dir_t; 
	struct motor {
		glm::vec3 pos; 
		motor_dir_t dir; 
		float thrust; 
		glm::vec3 torque; 
	} _motors[8]; 
	btRigidBody *mBody;
	Application *mApp; 

	irr::scene::ISceneNode *mNode;
	irr::scene::ICameraSceneNode *mCamera; 
};