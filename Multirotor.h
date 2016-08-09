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

class Multirotor : public Aircraft {
public:
	Multirotor(Application *app);

	virtual void updateForces(); 
	virtual void render();
	
	virtual void setOutput(unsigned int id, float thrust) override; 
	
	void setSimulationOn(bool on); 
protected: 
	// implement aircraft methods 
	virtual btRigidBody *createFrameRigidBody() override; 
	virtual irr::scene::ISceneNode *createFrameSceneNode() override; 

	// for derived classes
	virtual void setupMotors() override {}; 

	std::vector<Motor*> _motors; 
private:
	
	bool raytest(const glm::vec3 &_start, const glm::vec3 &dir, glm::vec3 &end, glm::vec3 &norm); 
	
	irr::scene::IAnimatedMeshSceneNode *_body_node; 

	bool _simulate; 
};
