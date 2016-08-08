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

#include <vector>

#include "Aircraft.h"
#include "Motor.h"

using namespace glm; 

class Application;

class Copter : public Aircraft {
public:
	typedef enum {
		QUAD_PLUS, 
		QUAD_X
	} frame_type_t; 

	Copter(Application *app, Copter::frame_type_t frame_type);

	virtual void updateForces(); 
	virtual void render();
	
	void setOutputThrust(unsigned int id, float thrust); 

	void setFrameType(Copter::frame_type_t frame_type); 

	void setSimulationOn(bool on); 
protected: 
	// for setup 
	virtual btRigidBody *createFrameRigidBody(); 
	virtual irr::scene::ISceneNode *createFrameSceneNode(); 

private:
	std::vector<Motor*> _motors; 
	Application *mApp; 
	
	bool raytest(const glm::vec3 &_start, const glm::vec3 &dir, glm::vec3 &end, glm::vec3 &norm); 

	bool _simulate; 
};
