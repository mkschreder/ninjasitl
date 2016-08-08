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
#include <btBulletCollisionCommon.h>
#include <btBulletDynamicsCommon.h>
#include <cstdlib>
#include <glm/glm.hpp>
#include <glm/gtx/vector_angle.hpp>

#include "Socket.h"
#include "Copter.h"

using namespace irr;
using namespace core;
using namespace scene;
using namespace video;
using namespace io;
using namespace gui;

class Application : public IEventReceiver{
public:
	Application();
	~Application();
	
	void CreateStartScene();
	btRigidBody * CreateBox(const btVector3 &TPosition, const vector3df &TScale, btScalar TMass, const char *texture = "rust0.jpg");
	void CreateSphere(const btVector3 &TPosition, btScalar TRadius, btScalar TMass);
	void updatePhysics(float dt);
	void updateCamera(); 
	void UpdateRender(btRigidBody *TObject);
	void updateNetwork(double dt); 
	void ClearObjects();
	int GetRandInt(int TMax) { return rand() % TMax; }
	void run();
	Copter *getActiveQuad(){ return activeQuad; }
	virtual bool OnEvent(const SEvent &TEvent);
	
	bool clipRay(const glm::vec3 &_start, const glm::vec3 &_end, glm::vec3 *end, glm::vec3 *norm = NULL); 

	irr::video::IVideoDriver *getVideoDriver(); 
	irr::scene::ISceneManager *getSceneManager(); 
	btDiscreteDynamicsWorld *getDynamicsWorld();

	// Globals
	bool Done = false;
	btDiscreteDynamicsWorld *World;
	btSequentialImpulseConstraintSolver *Solver;
	btDefaultCollisionConfiguration *CollisionConfiguration;
	btCollisionDispatcher *Dispatcher; 
	btBroadphaseInterface *BroadPhase; 
	IrrlichtDevice *irrDevice;
	IVideoDriver *_drv;
	ISceneManager *_scene;
	IGUIEnvironment *irrGUI;
	ICameraSceneNode *mCamera; 
	//IGUIStaticText* irrStatusText; 
	IFileSystem *irrFile;
	ITimer *irrTimer;
	ILogger *irrLog;
	Copter *activeQuad; 
	list<btRigidBody *> Objects;
	double mRCThrottle, mRCYaw, mRCPitch, mRCRoll, mRCAux1, mRCAux2; 
	float _spin; 
	float _angle; 
	uint8_t _mode; 
	//SocketAPM sock; 
	u32 TimeStamp; 
	bool _key_down[KEY_KEY_CODES_COUNT]; 

private: 
	enum {
		CAMERA_THIRD_PERSON, 
		CAMERA_FIRST_PERSON,
		CAMERA_MODE_COUNT
	}; 

	void initSharedMemory(); 
	void handleInput(double dt); 

	void scanRange(void); 
	void renderRange(void); 

	glm::vec3 get_player_start(IQ3LevelMesh *level); 
	void add_triangle_mesh(IMesh *mesh, float scale); 

	bool _calibration; 
	uint32_t _sent_count; 
	
	float _range_scan[6]; 
	
	int _camera_mode; 

	// shared memory stuff
	char *_shmout; char *_shmin; 
};

