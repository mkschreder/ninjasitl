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

#include <btBulletCollisionCommon.h>
#include <btBulletDynamicsCommon.h>

#include "Application.h"

static int paused = 0; 
Application::Application():sock(true){
	mRCThrottle = 0; 
	mRCPitch = 0.5; 
	mRCYaw = 0.5; 
	mRCRoll = 0.5; 
	
	memset(_key_down, 0, sizeof(_key_down)); 

	_spin = 0; 
	_angle = 0; 

	irrDevice = createDevice(video::EDT_OPENGL, dimension2d<u32>(640, 480), 32, false, false, false, this);
	irrGUI = irrDevice->getGUIEnvironment();
	irrTimer = irrDevice->getTimer();
	irrScene = irrDevice->getSceneManager();
	irrDriver = irrDevice->getVideoDriver();

	irrDevice->getCursorControl()->setVisible(0);

	// load a map
	irrDevice->getFileSystem()->addFileArchive("map-20kdm2.pk3");
	scene::IAnimatedMesh* mesh = irrScene->getMesh("20kdm2.bsp");
	scene::ISceneNode* node = 0;

	if(!mesh){
		printf("could not load map mesh!\n"); 
		exit(0); 
	}

	node = irrScene->addOctreeSceneNode(mesh->getMesh(0), 0, -1, 1024);

	node->setScale(core::vector3df(0.1, 0.1, 0.1)); 
	node->setPosition(core::vector3df(-130,-14.4,-124.9));

	// Initialize bullet
	CollisionConfiguration = new btDefaultCollisionConfiguration();
	BroadPhase = new btAxisSweep3(btVector3(-1000, -1000, -1000), btVector3(1000, 1000, 1000));
	Dispatcher = new btCollisionDispatcher(CollisionConfiguration);
	Solver = new btSequentialImpulseConstraintSolver();
	World = new btDiscreteDynamicsWorld(Dispatcher, BroadPhase, Solver, CollisionConfiguration);
	World->setGravity(btVector3(0, -9.82, 0)); 

	// Add camera
	//mCamera = irrScene->addCameraSceneNodeFPS(0, 100, 0.01);
	mCamera = irrScene->addCameraSceneNode();
	mCamera->setPosition(vector3df(0, 0, 0));
	mCamera->setRotation(vector3df(0, 0, 0)); 
	//Camera->setUpVector(vector3df(0, 0, 1.0)); 
	//Camera->setTarget(vector3df(1, 0, 0));
	
	// Preload textures
	irrDriver->getTexture("ice0.jpg");
	irrDriver->getTexture("rust0.jpg");

	// Create the initial scene
	irrScene->addLightSceneNode(0, core::vector3df(2, 5, -2), SColorf(4, 4, 4, 1));
	irrScene->addLightSceneNode(0, core::vector3df(2, -5, -2), SColorf(4, 4, 4, 1));
	CreateStartScene();

	// Main loop
	TimeStamp = irrTimer->getTime();

	activeQuad = new Copter(this);

	sock.bind("127.0.0.1", 9002); 
}

Application::~Application(){
	ClearObjects();
	delete World;
	delete Solver;
	delete Dispatcher;
	delete BroadPhase;
	delete CollisionConfiguration;

	irrDevice->drop();
}

bool Application::clipRay(const glm::vec3 &_start, const glm::vec3 &_end, glm::vec3 *end, glm::vec3 *norm) {
	btVector3 s(_start.x, _start.y, _start.z); 
	btVector3 e(_end.x, _end.y, _end.z); 

	btCollisionWorld::ClosestRayResultCallback RayCallback(s, e);
	//RayCallback.m_collisionFilterMask = FILTER_CAMERA;

	// Perform raycast
	World->rayTest(s, e, RayCallback);
	if(RayCallback.hasHit()) {
		e = RayCallback.m_hitPointWorld;
		btVector3 n = RayCallback.m_hitNormalWorld;
		end->x = e[0]; end->y = e[1]; end->z = e[2]; 
		norm->x = n[0]; norm->y = n[1]; norm->z = n[2]; 
		return true;
	}
	return false;
}

void Application::handleInput(double dt){
	// Throttle
	if(_key_down[KEY_KEY_W]){
		mRCThrottle += dt * 0.3; 	
	} else if(_key_down[KEY_KEY_S]){
		mRCThrottle -= dt * 0.3; 
	} else {
		
	}

	// Yaw
	if(_key_down[KEY_KEY_D]){
		if(mRCYaw < 0.5) mRCYaw = 0.5; 
		mRCYaw += dt * 0.2; 	
	} else if(_key_down[KEY_KEY_A]){
		if(mRCYaw > 0.5) mRCYaw = 0.5; 
		mRCYaw -= dt * 0.2; 
	} else {
		mRCYaw = 0.5; 
	}	

	// Pitch 
	if(_key_down[KEY_UP]){
		if(mRCPitch < 0.5) mRCPitch = 0.5; 
		mRCPitch += dt * 0.2; 	
	} else if(_key_down[KEY_DOWN]){
		if(mRCPitch > 0.5) mRCPitch = 0.5; 
		mRCPitch -= dt * 0.2; 
	} else {
		mRCPitch = 0.5; 
	}	

	// Roll 
	if(_key_down[KEY_RIGHT]){
		if(mRCRoll < 0.5) mRCRoll = 0.5; 
		mRCRoll += dt * 0.2; 	
	} else if(_key_down[KEY_LEFT]){
		if(mRCRoll > 0.5) mRCRoll = 0.5; 
		mRCRoll -= dt * 0.2; 
	} else {
		mRCRoll = 0.5; 
	}	

	if(mRCThrottle > 1.0f) mRCThrottle = 1.0f; if(mRCThrottle < 0.0f) mRCThrottle = 0.0f; 
	if(mRCYaw > 1.0f) mRCYaw = 1.0f; if(mRCYaw < 0.0f) mRCYaw = 0.0f; 
	if(mRCPitch > 1.0f) mRCPitch = 1.0f; if(mRCPitch < 0.0f) mRCPitch = 0.0f; 
	if(mRCRoll > 1.0f) mRCRoll = 1.0f; if(mRCRoll < 0.0f) mRCRoll = 0.0f; 
}

void Application::updateCamera(){
	glm::quat rot = activeQuad->getRotation(); 
	glm::vec3 pos = activeQuad->getPosition(); 

	glm::vec3 cpos = pos + rot * glm::vec3(0, 0, -3); 
	cpos.y = pos.y + 2; 
	glm::vec3 cnorm; 
	clipRay(pos, cpos, &cpos, &cnorm); 

	mCamera->setPosition(vector3df(cpos.x, cpos.y, cpos.z)); 
	mCamera->setTarget(vector3df(pos.x, pos.y, pos.z));
}

void Application::run(){
	long long time = irrTimer->getTime(); 
	double dt = (time - TimeStamp) * 0.001f;
	TimeStamp = time; 

	_angle += _spin * dt; 
	glm::quat q(cos(glm::radians(_angle / 2)), 0, sin(glm::radians(_angle / 2)), 0); 
	//activeQuad->setRotation(q); 

	if(!paused){
		updatePhysics(0.018);
		activeQuad->update(dt);
		updateNetwork(dt); 
		updateCamera(); 
		handleInput(dt); 
	}
	
	irrDriver->beginScene(true, true, SColor(255, 20, 0, 0));
	
	irrScene->drawAll();
	
	// debug
	activeQuad->render(irrDriver); 
	
	irrGUI->drawAll();
	irrDriver->endScene();
	irrDevice->run();
}


bool Application::OnEvent(const SEvent &ev) {
	static int ph = 0; 

	if(ev.EventType == EET_KEY_INPUT_EVENT){
		_key_down[ev.KeyInput.Key] = ev.KeyInput.PressedDown; 
	}

	if(ev.EventType == EET_KEY_INPUT_EVENT && !ev.KeyInput.PressedDown) {
		switch(ev.KeyInput.Key) {
			case KEY_ESCAPE:
				Done = true;
			break;
			case KEY_KEY_Q: 
				paused = ~paused; 
				break; 
			case KEY_KEY_O: 
				_spin += 0.5; 
				break; 
			case KEY_KEY_L: 
				_spin -= 0.5; 
				break; 
			case KEY_KEY_6: {
				static int id = 0; 
				glm::vec3 vel[4] = {
					glm::vec3(0, 0, 0), 
					glm::vec3(1, 0, 0), 
					glm::vec3(0, 1, 0), 
					glm::vec3(0, 0, 1)
				}; 
				activeQuad->setRotation(glm::quat(1, 0, 0, 0)); 
				activeQuad->setAngularVelocity(vel[id++]); 
				id = id % (sizeof(vel) / sizeof(vel[0])); 
				break; 
			}
			case KEY_KEY_7: {
				static int id = 0; 
				glm::quat y = glm::quat(cos(glm::radians(90.0/2)), 0, sin(glm::radians(90.0/2)), 0); 
				glm::quat rots[5] = {
					glm::quat(cos(glm::radians(0.0) / 2), 0, 0, sin(glm::radians(0.0) / 2)), // level
					glm::quat(cos(glm::radians(-45.0) / 2), 0, 0, sin(glm::radians(-45.0) / 2)), // left 
					glm::quat(cos(glm::radians(45.0) / 2), 0, 0, sin(glm::radians(45.0) / 2)), // right
					glm::quat(cos(glm::radians(-45.0) / 2), sin(glm::radians(-45.0) / 2), 0, 0), // front
					glm::quat(cos(glm::radians(45.0) / 2), sin(glm::radians(45.0) / 2), 0, 0) // back
				}; 
				activeQuad->setPosition(glm::vec3(0, 10, 0)); 
				activeQuad->setRotation(rots[id++] * y); 
				activeQuad->setAngularVelocity(glm::vec3(0, 0, 0)); 
				id = id % (sizeof(rots) / sizeof(rots[0])); 
				break; 
			}
			case KEY_KEY_8: {
				static int id = 0; 
				glm::quat rots[7] = {
					glm::quat(cos(glm::radians(0.0) / 2), 0, 0, sin(glm::radians(0.0) / 2)), // level
					glm::quat(cos(glm::radians(90.0) / 2), 0, 0, sin(glm::radians(90.0) / 2)), // left 
					glm::quat(cos(glm::radians(-90.0) / 2), 0, 0, sin(glm::radians(-90.0) / 2)), // right
					glm::quat(cos(glm::radians(90.0) / 2), sin(glm::radians(90.0) / 2), 0, 0), // front
					glm::quat(cos(glm::radians(-90.0) / 2), sin(glm::radians(-90.0) / 2), 0, 0), // back
					glm::quat(cos(glm::radians(0.0) / 2), 0, 0, sin(glm::radians(0.0) / 2)), // level
					glm::quat(cos(glm::radians(180.0) / 2), 0, 0, sin(glm::radians(180.0) / 2)) // upsidedown
				}; 
				activeQuad->setRotation(rots[id++]); 
				id = id % 7; 
				break; 
			}
			case KEY_KEY_1:
				CreateBox(btVector3(GetRandInt(10) - 5.0f, 7.0f, GetRandInt(10) - 5.0f), vector3df(GetRandInt(3) + 0.5f, GetRandInt(3) + 0.5f, GetRandInt(3) + 0.5f), 500.0f);
			break;
			case KEY_KEY_X:
				// reset
			break;
			default:
				return false;
			break;
		}
		
		return true;
	}

	return false;
}

// Runs the physics simulation.
// - TDeltaTime tells the simulation how much time has passed since the last frame so the simulation can run independently of the frame rate.
void Application::updatePhysics(float dt) {
	World->stepSimulation(dt, 60);

	// Relay the object's orientation to irrlicht
	for(list<btRigidBody *>::Iterator Iterator = Objects.begin(); Iterator != Objects.end(); ++Iterator) {
		UpdateRender(*Iterator);
	}	
}

// Creates a base box
void Application::CreateStartScene() {
	ClearObjects();
	CreateBox(btVector3(0.0f, 0.0f, 0.0f), vector3df(50.0f, 0.5f, 50.0f), 0.0f, "ice0.jpg");
	CreateBox(btVector3(20.0f, 0.0f, 0.0f), vector3df(0.5f, 50.0f, 50.0f), 0.0f, "ice0.jpg");
	CreateBox(btVector3(0.0f, 0.0f, 20.0f), vector3df(50.0f, 50.0f, 0.5f), 0.0f, "ice0.jpg");
}

// Create a box rigid body
btRigidBody *Application::CreateBox(const btVector3 &TPosition, const vector3df &TScale, btScalar TMass, const char *texture) {

	ISceneNode *Node = irrScene->addCubeSceneNode(1.0f);
	Node->setScale(TScale);
	Node->setMaterialFlag(EMF_LIGHTING, 1);
	Node->setMaterialFlag(EMF_NORMALIZE_NORMALS, true);
	Node->setMaterialTexture(0, irrDriver->getTexture(texture));

	// Set the initial position of the object
	btTransform Transform; 
	Transform.setIdentity(); 
	Transform.setOrigin(TPosition); 
	//(btQuaternion(sin(glm::radians(5.0f)), 0.0f, 0.0f, cos(glm::radians(5.0f))), TPosition);

	btDefaultMotionState *MotionState = new btDefaultMotionState(Transform);

	// Create the shape
	btVector3 HalfExtents(TScale.X * 0.5f, TScale.Y * 0.5f, TScale.Z * 0.5f);
	btCollisionShape *Shape = new btBoxShape(HalfExtents);

	// Add mass
	btVector3 LocalInertia;
	Shape->calculateLocalInertia(TMass, LocalInertia);

	// Create the rigid body object
	btRigidBody *RigidBody = new btRigidBody(TMass, MotionState, Shape, LocalInertia);

	// Store a pointer to the irrlicht node so we can update it later
	RigidBody->setUserPointer((void *)(Node));

	// Add it to the world
	World->addRigidBody(RigidBody);
	Objects.push_back(RigidBody);

	return RigidBody; 
}


// Passes bullet's orientation to irrlicht
void Application::UpdateRender(btRigidBody *TObject) {
	ISceneNode *Node = static_cast<ISceneNode *>(TObject->getUserPointer());

	// Set position
	btVector3 Point = TObject->getCenterOfMassPosition();
	Node->setPosition(vector3df((f32)Point[0], (f32)Point[1], (f32)Point[2]));

	// Set rotation
	vector3df Euler;
	const btQuaternion& TQuat = TObject->getOrientation();
	quaternion q(TQuat.getX(), TQuat.getY(), TQuat.getZ(), TQuat.getW());
	q.toEuler(Euler);
	Euler *= RADTODEG;
	Node->setRotation(Euler);
}

void Application::updateNetwork(double dt){
	#define MODE_CLIENT_SIM 0
	#define MODE_SERVER_SIM 1
	struct server_packet {
		uint8_t mode; 
		int16_t servo[8]; 
		double pos[3]; 
		double vel[3]; 
		double euler[3]; 
		double acc[3]; 
	}; 

	struct client_packet {
		double timestamp; 
		double gyro[3]; 
		double accel[3]; 
		double euler[3]; 
		double pos[3]; 
		double vel[3]; 
		double rcin[8]; 
	}; 

	struct server_packet pkt; 

	static double time = 0; 
	static glm::vec3 angles(0, 0, 0); 

	if(sock.recv(&pkt, sizeof(pkt), 0) == sizeof(pkt)){
		float roll = pkt.euler[0]; 
		float pitch = pkt.euler[1]; 
		float yaw = pkt.euler[2]; 

		if(pkt.mode == MODE_SERVER_SIM){
			activeQuad->setSimulationOn(false); 
			activeQuad->setPosition(glm::vec3(pkt.pos[1], -pkt.pos[2], pkt.pos[0])); 
			// yaw = y, pitch = x, roll = z; 
			glm::quat rr(cos(-roll / 2), 0, 0, sin(-roll / 2));  
			glm::quat rp(cos(-pitch / 2), sin(-pitch / 2), 0, 0);  
			glm::quat ry(cos(yaw / 2), 0, sin(yaw / 2), 0);  

			activeQuad->setRotation(ry * rp * rr); 
			activeQuad->setLinearVelocity(glm::vec3(pkt.vel[1], -pkt.vel[2], pkt.vel[0])); 
			//activeQuad->setLinearAcceleration(glm::vec3(pkt.acc[1], -pkt.acc[2], pkt.acc[0])); 

			//printf("pos(%f %f %f) vel(%f %f %f)\n", pkt.pos[0], pkt.pos[1], pkt.pos[2], pkt.vel[0], pkt.vel[1], pkt.vel[2]); 
		} else if(pkt.mode == MODE_CLIENT_SIM){
			activeQuad->setSimulationOn(true); 
		}

		//printf("servo(%d %d %d %d) rpy(%f %f %f)\n", pkt.servo[0], pkt.servo[1], pkt.servo[2], pkt.servo[3], roll, pitch, yaw); 
		for(unsigned c = 0; c < 8; c++){
			activeQuad->setOutputThrust(c, (pkt.servo[c] - 1000) / 1000.0f); 
		}

		struct client_packet state; 
		memset(&state, 0, sizeof(state)); 

		// create a 3d world to ned frame rotation
		glm::quat r0(cos(glm::radians(-90.0 / 2)), sin(glm::radians(-90.0 / 2)), 0, 0); 
		glm::quat r1(cos(glm::radians(90.0 / 2)), 0, sin(glm::radians(90.0 / 2)), 0); 
		glm::quat r2(cos(glm::radians(-90.0 / 2)), 0, 0, sin(glm::radians(-90.0 / 2))); 
		glm::quat r2ef = r2 * r1; 

		glm::vec3 pos = activeQuad->getPosition(); 
		glm::quat rot = activeQuad->getRotation(); 
		glm::vec3 accel = activeQuad->getAccel(); 
		glm::vec3 vel = glm::inverse(rot) * activeQuad->getVelocity(); 
		glm::vec3 gyro = activeQuad->getGyro(); 
		
		if(paused){
			gyro = glm::vec3(0, 0, 0); 
			//glm::quat r(cos(glm::radians(45.0 / 2)), sin(glm::radians(45.0 / 2)), 0, 0); 
			//accel = r * glm::vec3(0, 0, -9.82); 
		}
		
		glm::vec3 euler = glm::eulerAngles(rot);
		glm::vec3 bx = rot * glm::vec3(1, 0, 0); 
		glm::vec3 by = rot * glm::vec3(0, 1, 0); 
		glm::vec3 bz = rot * glm::vec3(0, 0, 1); 
		glm::vec3 px = bx - glm::vec3(0, bx.y, 0); 
		glm::vec3 pz = bz - glm::vec3(0, bz.y, 0); 

		float p = glm::orientedAngle(px, bx, glm::vec3(0, 1, 0));  
		float r = glm::orientedAngle(glm::vec3(0, 0, 1), bz, glm::vec3(0, 1, 0));  
		float y = 0; 
		//printf("euler: %f %f %f %f %f %f\n", glm::degrees(euler.z), glm::degrees(euler.x), glm::degrees(euler.y), glm::degrees(p), glm::degrees(r), glm::degrees(y)); 

		//state.euler[0] = r; state.euler[1] = -p; state.euler[2] = -y; 
		state.euler[0] = euler.z; state.euler[1] = euler.x; state.euler[2] = euler.y; 
		state.gyro[0] = -gyro.z; state.gyro[1] = -gyro.x; state.gyro[2] = gyro.y; 
		state.accel[0] = accel.z; state.accel[1] = accel.x; state.accel[2] = -accel.y; 
		//state.accel[0] = 0; state.accel[1] = 0; state.accel[2] = -9.82; 
		state.pos[0] = pos.z; state.pos[1] = pos.x; state.pos[2] = -pos.y; 
		state.vel[0] = vel.z; state.vel[1] = vel.x; state.vel[2] = -vel.y; 

		state.rcin[0] = mRCRoll; 
		state.rcin[1] = -mRCPitch; 
		state.rcin[2] = mRCThrottle; 
		state.rcin[3] = mRCYaw; 
		/*
		printf("sending: acc(%f %f %f) gyr(%f %f %f) rc(%f %f %f %f)\n", 
			state.accel[0], state.accel[1], state.accel[2],
			state.gyro[0], state.gyro[1], state.gyro[2],
			state.rcin[0], state.rcin[1], state.rcin[2], state.rcin[3]); 
			*/
		sock.sendto(&state, sizeof(state), "127.0.0.1", 9003); 
	}	
}

// Removes all objects from the world
void Application::ClearObjects() {
	for(list<btRigidBody *>::Iterator Iterator = Objects.begin(); Iterator != Objects.end(); ++Iterator) {
		btRigidBody *Object = *Iterator;

		// Delete irrlicht node
		ISceneNode *Node = static_cast<ISceneNode *>(Object->getUserPointer());
		Node->remove();

		// Remove the object from the world
		World->removeRigidBody(Object);

		// Free memory
		delete Object->getMotionState();
		delete Object->getCollisionShape();
		delete Object;
	}

	Objects.clear();
}
