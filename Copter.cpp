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
#include "Copter.h"

Copter::Copter(Application *app, Copter::frame_type_t frame_type):Aircraft(app){
	mApp = app;

	Aircraft::configure(createFrameSceneNode(), createFrameRigidBody()); 
	
	setFrameType(frame_type); 

	// Create a box rigid body

	//_frame_body->setUserPointer((void *)(Node));

	// Add it to the world
	//mApp->World->addRigidBody(_frame_body);
	//mApp->Objects.push_back(_frame_body);
}

ISceneNode *Copter::createFrameSceneNode(){
	ISceneManager *scene = mApp->getSceneManager();
	
	IAnimatedMesh *mesh = scene->getMesh("quad_body.obj"); 
	ISceneNode *node = scene->addAnimatedMeshSceneNode(mesh);

	node->setMaterialTexture(0, mApp->_drv->getTexture("rust0.jpg"));
	
	return node; 
}

btRigidBody *Copter::createFrameRigidBody(){
	float mass = 2.0f; 

	btTransform tr;
	tr.setIdentity();
	tr.setOrigin(btVector3(0, 0.5, 0));

	btDefaultMotionState *ms = new btDefaultMotionState(tr);

	btBoxShape *shape = new btBoxShape(btVector3(0.5f, 0.05f, 0.5f)); 

	// Add mass
	btVector3 inertia;
	shape->calculateLocalInertia(mass, inertia);

	// Create the rigid body object
	btRigidBody::btRigidBodyConstructionInfo info(mass, ms, shape, inertia); //motion state would actually be non-null in most real usages
	info.m_restitution = 2.0f;
	info.m_friction = 1.5f;

	btRigidBody *frame = new btRigidBody(info);
	
	frame->setDamping(0.05, 0.1); 
	frame->setActivationState(DISABLE_DEACTIVATION); 

	_app->getDynamicsWorld()->addRigidBody(frame); 

	return frame; 
}

void Copter::setFrameType(Copter::frame_type_t frame_type){
	// delete all motors 
	for(std::vector<Motor*>::iterator i = _motors.begin(); 
		i != _motors.end(); ++i){
		delete *i; 
	}
	_motors.clear(); 

	// setup motor matrix
	switch(frame_type){
		case QUAD_PLUS: 
			_motors.push_back(new Motor(glm::vec3( 0.15, 0.05, 0.0), Motor::MOTOR_CCW));
			_motors.push_back(new Motor(glm::vec3(-0.15, 0.05, 0.0), Motor::MOTOR_CCW));
			_motors.push_back(new Motor(glm::vec3( 0.0, 0.05, 0.15), Motor::MOTOR_CW));
			_motors.push_back(new Motor(glm::vec3( 0.0, 0.05,-0.15), Motor::MOTOR_CW));
			break; 
		case QUAD_X: 	
			_motors.push_back(new Motor(glm::vec3( 0.15, 0.0, 0.1), Motor::MOTOR_CCW));
			_motors.push_back(new Motor(glm::vec3(-0.15, 0.0,-0.1), Motor::MOTOR_CCW));
			_motors.push_back(new Motor(glm::vec3(-0.15, 0.0, 0.1), Motor::MOTOR_CW));
			_motors.push_back(new Motor(glm::vec3( 0.15, 0.0,-0.1), Motor::MOTOR_CW));
			break; 
		default: 
			break; 
	}
}
/*
void Copter::setSimulationOn(bool on){
	mApp->World->removeRigidBody(_frame_body);
	if(on){
		_frame_body->setActivationState(DISABLE_DEACTIVATION);
		_frame_body->setActivationState(ACTIVE_TAG);
		//_frame_body->setDamping(0.5, 0.8); 
	} else {	
		_frame_body->setActivationState(DISABLE_SIMULATION);
		//_frame_body->setDamping(0, 0); 
	}
	mApp->World->addRigidBody(_frame_body);
	_simulate = on; 
}
*/
void Copter::setOutputThrust(unsigned int id, float value){
	if(id >= _motors.size()) return; 
	_motors[id]->setThrust(value); 	
}


void Copter::render(){
	Aircraft::render(); 

	irr::video::IVideoDriver *drv = mApp->getVideoDriver(); 

	glm::quat rot = getRotation();
	glm::vec3 pos = getPosition(); // + rot * glm::vec3(0, 0.2, 0);
	glm::vec3 acc = rot * getAccel();  
	glm::vec3 mag = (rot * getMagneticField()) * 0.001f;  
	glm::vec3 vel = getVelocity(); 
	glm::vec3 ax = rot * glm::vec3(0.1, 0.0, 0.0); 
	glm::vec3 ay = rot * glm::vec3(0.0, 0.1, 0.0); 
	glm::vec3 az = rot * glm::vec3(0.0, 0.0, 0.1); 

	drv->setTransform(irr::video::ETS_WORLD, irr::core::IdentityMatrix);

	SMaterial m;
	m.Lighting = false;
	drv->setMaterial(m);
	
	glm::vec3 hit, norm; 
	/*
	if(mApp->clipRay(pos, pos + rot * glm::vec3(100.0, 0, 0.0), &hit, &norm)){
		drv->draw3DBox(aabbox3df(vector3df(hit.x - 0.1, hit.y - 0.1, hit.z - 0.1), vector3df(hit.x + 0.1, hit.y + 0.1, hit.z + 0.1)), SColor(255, 255, 0, 0)); 
		drv->draw3DLine(vector3df(pos.x, pos.y, pos.z),
			vector3df(hit.x, hit.y, hit.z), 
			SColor( 255, 255, 0, 0 ));
	}*/

	drv->draw3DLine(vector3df(pos.x, pos.y, pos.z),
		vector3df(pos.x + ax.x, pos.y + ax.y, pos.z + ax.z), 
		SColor( 255, 255, 0, 0 ));
	
	drv->draw3DLine(vector3df(pos.x, pos.y, pos.z),
		vector3df(pos.x + ay.x, pos.y + ay.y, pos.z + ay.z), 
		SColor( 255, 0, 255, 0 ));

	drv->draw3DLine(vector3df(pos.x, pos.y, pos.z),
		vector3df(pos.x + az.x, pos.y + az.y, pos.z + az.z), 
		SColor( 255, 0, 0, 255 ));

	drv->draw3DLine(vector3df(pos.x, pos.y, pos.z),
		vector3df(pos.x + acc.x, pos.y + acc.y, pos.z + acc.z), 
		SColor( 255, 255, 255, 255 ));

	drv->draw3DLine(vector3df(pos.x, pos.y, pos.z),
		vector3df(pos.x + mag.x, pos.y + mag.y, pos.z + mag.z), 
		SColor( 255, 255, 255, 255 ));

	/*
	drv->draw3DLine(vector3df(pos.x, pos.y, pos.z),
		vector3df(pos.x + _linear_acceleration.x, pos.y + _linear_acceleration.y, pos.z + _linear_acceleration.z), 
		SColor( 255, 255, 0, 255 ));
	*/

	drv->draw3DLine(vector3df(pos.x, pos.y, pos.z),
		vector3df(pos.x + vel.x, pos.y + vel.y, pos.z + vel.z), 
		SColor( 255, 255, 0, 255 ));

	
	for(std::vector<Motor*>::iterator i = _motors.begin();
		i != _motors.end(); 
		++i){
		Motor *m = *i; 

		glm::vec3 p = rot * m->getPosition() + pos; 
		glm::vec3 f = rot * m->getThrustVector() * 0.1f; 
		glm::vec3 a = rot * m->getTorqueVector() * 0.1f; 
		
		// draw direction of thrust
		drv->draw3DLine(vector3df(p.x, p.y, p.z),
			vector3df(p.x + f.x, p.y + f.y, p.z + f.z), SColor(255, 255, 255, 0));

		// draw direction of torque 
		drv->draw3DLine(vector3df(p.x, p.y, p.z),
			vector3df(p.x + a.x, p.y + a.y, p.z + a.z), SColor(255, 255, 0, 0));
	}	
}

void Copter::updateForces(){
	for(std::vector<Motor*>::iterator i = _motors.begin(); i != _motors.end(); ++i){
		Aircraft::applyFrameForce((*i)->getThrustVector(), (*i)->getPosition()); 	
		Aircraft::applyFrameForce((*i)->getTorqueVector(), (*i)->getPosition()); 	
	}
}

