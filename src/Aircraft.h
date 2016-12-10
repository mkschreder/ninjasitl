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

#include <glm/glm.hpp>
#include <glm/vec3.hpp>
#include <glm/mat3x3.hpp>
#include <glm/gtc/quaternion.hpp>

#include <btBulletCollisionCommon.h>
#include <btBulletDynamicsCommon.h>

class Application; 

class Aircraft {
public: 
	glm::quat getRotation(); 
	glm::vec3 getVelocity(); 
	glm::vec3 getPosition(); 
	glm::ivec3 getLocation(); 
	glm::vec3 getMagneticField(); 
	glm::vec3 getAccel(); 
	glm::vec3 getGyro(); 
	
	void setPosition(const glm::vec3 &pos); 
	void setRotation(const glm::quat &rot); 
	void setAngularVelocity(const glm::vec3 &v); 
	void setLinearVelocity(const glm::vec3 &v); 
	
	void setHomeLocation(const glm::vec3 &loc); 

	// for forcing values in supervised mode
	void setAccelerometer(const glm::vec3 &v); 
	void setMagneticField(const glm::vec3 &v); 

	virtual void updateForces() {};	
	virtual void update(float dt); 
	virtual void render(){}; 
	virtual void setOutput(unsigned int id, float value) {}; 
	
	virtual void onCollision(const btCollisionObject *a, const btCollisionObject *b) {} ; 

	void init(); 	
protected: 
	Aircraft(Application *app); 

	virtual btRigidBody *createFrameRigidBody() { return NULL; }; 
	virtual irr::scene::ISceneNode *createFrameSceneNode() { return NULL; }; 
	virtual void setupMotors(void) {} ; 

	void applyFrameForce(const glm::vec3 &force, const glm::vec3 &force_pos); 
	
	Application *_application; 

private:
	void updateLocation(); 

	glm::vec3 calcMagFieldIntensity(); 
	glm::vec3 calcAcceleration(); 

	glm::vec3 _velocity; 
	glm::vec3 _linear_acceleration; 
	glm::vec3 _accel; 
	glm::vec3 _mag; 
	glm::ivec3 _location; 
	glm::ivec3 _home; 

	irr::scene::ISceneNode *_frame_node; 
protected:
	btRigidBody *_frame_body; 
}; 
