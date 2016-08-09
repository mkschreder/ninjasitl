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
#include "TiltXRotor.h"

#include <BulletDynamics/ConstraintSolver/btGeneric6DofSpringConstraint.h>

TiltXRotor::TiltXRotor(Application *app) : GenericXRotor(app) {

}

irr::scene::ISceneNode *TiltXRotor::createFrameSceneNode(){
	_frame_node = (irr::scene::IAnimatedMeshSceneNode*)Multirotor::createFrameSceneNode(); 
	irr::scene::ISceneManager *scene = _application->getSceneManager(); 
	_frame_node->setMesh(scene->getMesh("tilt_body.obj")); 

	_front_arm = scene->addAnimatedMeshSceneNode(scene->getMesh("tilt_arm_pair.obj")); 
	_back_arm = scene->addAnimatedMeshSceneNode(scene->getMesh("tilt_arm_pair.obj")); 

	_front_arm->setPosition(vector3df(0, 0, 0.1f)); 
	_back_arm->setPosition(vector3df(0, 0, -0.1f)); 

	_frame_node->addChild(_front_arm); 
	_frame_node->addChild(_back_arm);

	return _frame_node; 
}

btRigidBody *TiltXRotor::createFrameRigidBody(){
	btRigidBody *frame = Multirotor::createFrameRigidBody(); 
	
	// create shapes for propellers
	for(int c = 0; c < 4; c++){
		btTransform tr;
		glm::vec3 mp = _motors[c]->getPosition(); 
		tr.setIdentity();
		tr.setOrigin(btVector3(mp.x, mp.y, mp.z));

		btDefaultMotionState *ms = new btDefaultMotionState(tr);
		btSphereShape *shape = new btSphereShape(0.1f); 

		btVector3 inertia;
		shape->calculateLocalInertia(0.1, inertia);

		// Create the rigid body object
		btRigidBody::btRigidBodyConstructionInfo info(0.1, ms, shape, inertia); //motion state would actually be non-null in most real usages
		btRigidBody *prop = new btRigidBody(info);
	
		btFixedConstraint *cons = new btFixedConstraint(*frame, *prop, btTransform::getIdentity(), tr); 

		_application->getDynamicsWorld()->addConstraint(cons); 
		_application->getDynamicsWorld()->addRigidBody(prop, COLLIDE_PROP, COLLIDE_WORLD); 

		_props.push_back(prop); 
	}
	return frame; 
}

void TiltXRotor::onCollision(const btCollisionObject *a, const btCollisionObject *b) {
	///if(a == _frame_body || b == _frame_body){
		btVector3 vel = _frame_body->getLinearVelocity(); 
		vel[1] = 0; 
		unsigned int id = 0; 
		for(std::vector<btRigidBody*>::iterator i = _props.begin(); 
			i != _props.end(); 
			++i, id++){
			if(*i == a || *i == b){
				printf("prop collision! %f \n", vel.length()); 
				//if(vel.length() > 0.1f && id < _motors.size()){
				//	_motors[id]->setPower(0);  
				//}

			}
		}
		//}
}

void TiltXRotor::setOutput(unsigned int id, float value){
	if(id == 7){
		float pitch = (value - 0.5f) * 90.0f; 
		// tilt all rotors forward or backwards depending on supplied pitch
		for(std::vector<Motor*>::iterator i = _motors.begin(); 
			i != _motors.end(); ++i){
			(*i)->setRotation(glm::quat(cos(glm::radians(pitch / 2.0f)), -sin(glm::radians(pitch / 2.0f)), 0, 0)); 
		}

		_front_arm->setRotation(vector3df(-pitch, 0, 0)); 
		_back_arm->setRotation(vector3df(-pitch, 0, 0)); 
	} else {
		GenericXRotor::setOutput(id, value); 
	}
}
