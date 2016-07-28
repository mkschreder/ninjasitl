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

Copter::Copter(Application *app){
	float mass = 2.0f; 
	mApp = app;
	
	// Create a box rigid body
	ISceneManager *irrScene = mApp->irrScene;
	
	ISceneNode *Node = irrScene->addCubeSceneNode(1.0f);
	Node->setScale(vector3df(1, 0.1, 1));
	Node->setMaterialFlag(EMF_LIGHTING, 1);
	Node->setMaterialFlag(EMF_NORMALIZE_NORMALS, true);
	Node->setMaterialTexture(0, mApp->irrDriver->getTexture("rust0.jpg"));
	mNode = Node;

	// add motor cubes	
	glm::mat4 pmat(
		glm::vec4(0.0, 0.05, 0.5, 0.0),
		glm::vec4(0.5, 0.05, 0.0, 0.0),
		glm::vec4(0.0, 0.05, -0.5, 0.0),
		glm::vec4(-0.5,0.05, 0.0, 0.0));
	for(int c = 0; c < 4; c++){
		glm::vec4 pos = pmat[c];
		ISceneNode *pn = irrScene->addCubeSceneNode(1.0f);
		pn->setPosition(vector3df(pos.x, pos.y, pos.z)); 
		pn->setScale(vector3df(0.5, 1.0, 0.5));
		pn->setMaterialFlag(EMF_LIGHTING, 1);
		pn->setMaterialFlag(EMF_NORMALIZE_NORMALS, true);
		pn->setMaterialTexture(0, mApp->irrDriver->getTexture("rust0.jpg"));
		Node->addChild(pn); 
	}
	// Set the initial position of the object
	btTransform Transform;
	Transform.setIdentity();
	Transform.setOrigin(btVector3(0, 0.5, 0));

	btDefaultMotionState *MotionState = new btDefaultMotionState(Transform);

	// Create the shape
	btVector3 HalfExtents(0.5f, 0.05f, 0.5f);
	btCollisionShape *Shape = new btBoxShape(HalfExtents);
	
	// Add mass
	btVector3 LocalInertia;
	Shape->calculateLocalInertia(mass, LocalInertia);

	// Create the rigid body object
	btRigidBody::btRigidBodyConstructionInfo info(mass, MotionState, Shape, LocalInertia); //motion state would actually be non-null in most real usages
	info.m_restitution = 2.0f;
	info.m_friction = 1.5f;

	mBody = new btRigidBody(info);
	mBody->setActivationState(DISABLE_DEACTIVATION);
	//mBody->forceActivationState(DISABLE_SIMULATION);

	// Store a pointer to the irrlicht node so we can update it later
	mBody->setUserPointer((void *)(Node));

	// Add it to the world
	mApp->World->addRigidBody(mBody);
	mApp->Objects.push_back(mBody);

	mCamera = 0; 

	_motors[1-1] = (struct motor){ glm::vec3( 0.5, 0.05, 0.0), MOTOR_CCW };
	_motors[2-1] = (struct motor){ glm::vec3(-0.5, 0.05, 0.0), MOTOR_CCW };
	_motors[3-1] = (struct motor){ glm::vec3( 0.0, 0.05, 0.5), MOTOR_CW };
	_motors[4-1] = (struct motor){ glm::vec3( 0.0, 0.05,-0.5), MOTOR_CW };

	for(int c = 0; c < 4; c++){
		_motors[c].thrust = 0.2; 
	}
	
	//FlightController::setSensorProvider(new Copter(mApp->World, RigidBody));
}

void Copter::attachCamera(irr::scene::ICameraSceneNode *cam){
	mCamera = cam;
	//mNode->addChild(mCamera); 
}

void Copter::setOutputThrust(unsigned int id, float value){
	if(id > (sizeof(_motors) / sizeof(_motors[0]))) return; 
	if(value < 0) value = 0; if(value > 1.0) value = 1.0; 
	_motors[id].thrust = value; 
	if(_motors[id].dir == MOTOR_CW)
		_motors[id].torque = glm::cross(glm::normalize(_motors[id].pos), glm::vec3(0, 1.0f, 0)) * value; 
	else if(_motors[id].dir == MOTOR_CCW)
		_motors[id].torque = glm::cross(glm::normalize(_motors[id].pos), glm::vec3(0, -1.0f, 0)) * value; 
}

void Copter::render(irr::video::IVideoDriver *drv){
	btTransform trans; 
	mBody->getMotionState()->getWorldTransform(trans); 
	btQuaternion qrot = trans.getRotation(); 
	btMatrix3x3 mat = trans.getBasis(); 
	btVector3 p = trans.getOrigin(); 
	glm::quat rot = getRotation();
	glm::vec3 pos = getPosition() + rot * glm::vec3(0, 0.2, 0);
	glm::vec3 acc = rot * getAccel();  
	glm::vec3 ax = rot * glm::vec3(1.0, 0.0, 0.0); 
	glm::vec3 ay = rot * glm::vec3(0.0, 1.0, 0.0); 
	glm::vec3 az = rot * glm::vec3(0.0, 0.0, 1.0); 

	drv->setTransform(irr::video::ETS_WORLD, irr::core::IdentityMatrix);

	SMaterial m;
	m.Lighting = false;
	drv->setMaterial(m);

	drv->draw3DLine(vector3df(pos.x, pos.y, pos.z),
		vector3df(pos.x + ax.x, pos.y + ax.y, pos.z + ax.z), 
		SColor( 255, 255, 0, 0 ));
	
	drv->draw3DLine(vector3df(pos.x, pos.y, pos.z),
		vector3df(pos.x + ay.x, pos.y + ay.y, pos.z + ay.z), 
		SColor( 255, 0, 255, 0 ));

	drv->draw3DLine(vector3df(pos.x, pos.y, pos.z),
		vector3df(pos.x + az.x, pos.y + az.y, pos.z + az.z), 
		SColor( 255, 0, 0, 255 ));

	//glm::vec3 front = rot * glm::vec3(0.0, 0.0, 1.0); 
	drv->draw3DLine(vector3df(pos.x, pos.y, pos.z),
		vector3df(pos.x + acc.x, pos.y + acc.y, pos.z + acc.z), 
		SColor( 255, 0, 0, 0 ));

	for(int c = 0; c < 4; c++){
		float th = _motors[c].thrust; 
		glm::vec3 p = rot * _motors[c].pos + pos; 
		glm::vec3 f = rot * glm::vec3(0, 1.0, 0) * th; 
		glm::vec3 a = rot * _motors[c].torque; 
		
		// draw direction of thrust
		drv->draw3DLine(vector3df(p.x, p.y, p.z),
			vector3df(p.x + f.x, p.y + f.y, p.z + f.z), SColor(255, 255, 0, 0));

		// draw direction of torque 
		drv->draw3DLine(vector3df(p.x, p.y, p.z),
			vector3df(p.x + a.x, p.y + a.y, p.z + a.z), SColor(0, 255, 0, 0));

	}	
}

glm::vec3 Copter::getPosition(){
	btVector3 pos = mBody->getCenterOfMassPosition();
	return glm::vec3(pos[0], pos[1], pos[2]); 
}

glm::quat Copter::getRotation(){
	const btQuaternion& quat = mBody->getOrientation();
	return glm::quat(quat.w(), quat.x(), quat.y(), quat.z()); 
}

void Copter::setPosition(const glm::vec3 &pos){
	btTransform t = mBody->getCenterOfMassTransform();
	t.setOrigin(btVector3(pos.x, pos.y, pos.z)); 
	mBody->setCenterOfMassTransform(t); 
}

void Copter::setRotation(const glm::quat &rot){
	btTransform t = mBody->getCenterOfMassTransform();
	btQuaternion q(rot.x, rot.y, rot.z, rot.w); 
	t.setRotation(q); 
	mBody->setCenterOfMassTransform(t); 
}

void Copter::setAngularVelocity(const glm::vec3 &v){
	mBody->setAngularVelocity(btVector3(v.x, v.y, v.z)); 
}

void Copter::setLinearVelocity(const glm::vec3 &v){
	mBody->setLinearVelocity(btVector3(v.x, v.y, v.z)); 
}

glm::vec3 Copter::getAccel(){
	glm::quat rot = getRotation(); 
	return glm::inverse(rot) * glm::vec3(0, 9.82, 0); 
	//return rot * glm::vec3(0, -9.82, 0); 
	/*btTransform t; 
	mBody->getMotionState()->getWorldTransform(t); 
	btMatrix3x3 trans = t.getBasis(); 
	btVector3 acc = (trans * btVector3(0, 0, 9.82)); 

	return glm::vec3(acc[0], acc[1], acc[2]); */
}

glm::vec3 Copter::getGyro(){
	glm::quat rot = getRotation(); 
	glm::vec3 lx = rot * glm::vec3(1, 0, 0); 
	glm::vec3 ly = rot * glm::vec3(0, 1, 0); 
	glm::vec3 lz = rot * glm::vec3(0, 0, 1); 

	btVector3 av = mBody->getAngularVelocity(); 

	float xr = lx.x * av[0] + lx.y * av[1] + lx.z * av[2]; 
	float yr = ly.x * av[0] + ly.y * av[1] + ly.z * av[2]; 
	float zr = lz.x * av[0] + lz.y * av[1] + lz.z * av[2]; 
	
	//btVector3 tmp = mBody->getAngularVelocity(); 
	// pitch, yaw, roll
	return glm::vec3(xr, yr, zr);
}

glm::vec3 Copter::getVelocity(){
	btVector3 v = mBody->getLinearVelocity();
	return glm::vec3(v[0], v[1], v[2]); 
}

/*
bool Copter::RaycastWorld(const btVector3 &Start, btVector3 &End, btVector3 &Normal) {
	btCollisionWorld::ClosestRayResultCallback RayCallback(Start, End);
	//RayCallback.m_collisionFilterMask = FILTER_CAMERA;

	// Perform raycast
	World->rayTest(Start, End, RayCallback);
	if(RayCallback.hasHit()) {
		End = RayCallback.m_hitPointWorld;
		Normal = RayCallback.m_hitNormalWorld;
		return true;
	}
	return false;
}
void Copter::update(float dt){
	if(!dt) dt = 1; // at least one millisecond

	// measure distance straight down 
	btVector3 end, normal; 
	btVector3 pos = mBody->getCenterOfMassPosition();
	const btQuaternion& quat = mBody->getOrientation(); //mBody->getWorldTransform().getRotation(); 
	btVector3 down = btTransform(quat) * btVector3(0, -1, 0); 
	//btVector3 gravity = World->getGravity();
	btVector3 velocity = mBody->getLinearVelocity();
	btVector3 tmp;
	
	if(RaycastWorld(down, end, normal)){
		mDistance = (end-pos).length() * 1000;
	} else {
		mDistance = -1;
	}

	mAltitude = pos[1] * 100;
	//glm::vec3 euler = glm::eulerAngles()); 
	
	//printf("EULER: %f %f %f\n", glm::degrees(euler.x), 
	//	glm::degrees(euler.y), glm::degrees(euler.z)); 
	
	btTransform t; 
	mBody->getMotionState()->getWorldTransform(t); 
	btMatrix3x3 trans = t.getBasis(); 
	btVector3 acc = (trans * btVector3(0, 0, 9.82)); 
	
	mAcc = glm::vec3(acc[0], acc[1], acc[2]);
	mOldVelocity = velocity;

	tmp = mBody->getAngularVelocity(); 
	mGyro = glm::vec3(tmp[0], tmp[1], tmp[2]);

	//tmp = (trans * btVector3(0, 0, 1)); 
	//mCompass = glm::vec3(1, 0, 0); //glm::vec3(tmp[0] * 32768, tmp[1] * 32768, tmp[2] * 32768); 
}
*/

void Copter::update(float dt){
	glm::quat rot = getRotation(); 
	glm::vec3 pos = getPosition(); 
	// + frame
	printf("thrust: "); 
	for(int c = 0; c < 4; c++){
		float th = _motors[c].thrust * 10.0f; 
		printf("%f ", th); 
		glm::vec3 p = rot * _motors[c].pos;
		glm::vec3 f = rot * glm::vec3(0, 1.0, 0) * th; 
		glm::vec3 a = rot * _motors[c].torque; 
		mBody->applyForce(btVector3(f.x, f.y, f.z), btVector3(p.x, p.y, p.z)); 
		mBody->applyForce(btVector3(a.x, a.y, a.z), btVector3(p.x, p.y, p.z)); 
	}
	
	printf("\n"); 

	if(mCamera){
		vector3df e;
		const btQuaternion& TQuat = mBody->getOrientation();
		btVector3 pos = mBody->getCenterOfMassPosition();
		quaternion q(TQuat.getX(), TQuat.getY(), TQuat.getZ(), TQuat.getW());
		btVector3 look = btTransform(TQuat) * btVector3(0, 0, 1) + pos; 
		q.toEuler(e);
		e *= RADTODEG;
		//printf("EULER: %f %f %f\n", e.X, e.Y, e.Z); 
		mCamera->setTarget(vector3df(look[0], look[1], look[2]));
		mCamera->setPosition(vector3df(pos[0], pos[1], pos[2])); 
		mCamera->setRotation(vector3df(e.X, e.Y, e.Z)); 
	}
}
