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

#include <sys/types.h>
#include <sys/ipc.h>
#include <sys/shm.h>
#include <stdio.h>

#include "GenericXRotor.h"
#include "TiltXRotor.h"

#define MODE_CLIENT_SIM 0
#define MODE_SERVER_SIM 1

#define Q3_WORLD_SCALE 0.015f

struct server_packet {
	uint8_t mode; 
	uint8_t frame; 
	int16_t servo[8]; 
	float pos[3]; 
	float vel[3]; 
	float euler[3]; 
	float acc[3]; 
	float mag[3]; 
}; 

struct client_packet {
	uint32_t id; 
	float timestamp; 
	float gyro[3]; 
	float accel[3]; 
	float euler[3]; 
	float pos[3]; 
	float vel[3]; 
	float rcin[8]; 
	int32_t loc[3]; 
	float mag[3]; 
	float range[6]; 
}; 

glm::vec3 Application::get_player_start(IQ3LevelMesh *level){
	quake3::tQ3EntityList &entityList = level->getEntityList();

	quake3::IEntity search;
	search.name = "info_player_deathmatch";

	s32 index = entityList.binary_search(search);
	if (index >= 0)
	{
		s32 notEndList;
		do
		{
			const quake3::SVarGroup *group = entityList[index].getGroup(1);

			u32 parsepos = 0;
			const core::vector3df pos =
				quake3::getAsVector3df(group->get("origin"), parsepos);

			parsepos = 0;
			const f32 angle = quake3::getAsFloat(group->get("angle"), parsepos);

			core::vector3df target(0.f, 0.f, 1.f);
			target.rotateXZBy(angle);

			return glm::vec3(pos.X, pos.Y, pos.Z); 

			//camera->setPosition(pos);
			//camera->setTarget(pos + target);

			++index;
			notEndList = index == 2;
		} while ( notEndList );
	}
	return glm::vec3(0, 0, 0); 
}

void Application::add_triangle_mesh(IMesh *mesh, float scale){
	btTriangleMesh *trimesh = new btTriangleMesh();

	for (unsigned int i = 0; i < mesh->getMeshBufferCount(); i++){
		irr::scene::IMeshBuffer* mb = mesh->getMeshBuffer(i);
	   
		if (mb->getVertexType()==irr::video::EVT_STANDARD){
			::printf("Standard vertex\n"); 
		}
		if (mb->getVertexType()==irr::video::EVT_2TCOORDS){
			irr::video::S3DVertex2TCoords* mb_vertices=(irr::video::S3DVertex2TCoords*)mb->getVertices();
			u16* mb_indices = mb->getIndices();
			for (unsigned int j=0; j < mb->getIndexCount(); j += 3){
				video::S3DVertex2TCoords *v0 = &mb_vertices[mb_indices[j]]; 
				video::S3DVertex2TCoords *v1 = &mb_vertices[mb_indices[j+1]]; 
				video::S3DVertex2TCoords *v2 = &mb_vertices[mb_indices[j+2]]; 
				trimesh->addTriangle(
					btVector3(v0->Pos.X, v0->Pos.Y, v0->Pos.Z) * scale, 
					btVector3(v1->Pos.X, v1->Pos.Y, v1->Pos.Z) * scale, 
					btVector3(v2->Pos.X, v2->Pos.Y, v2->Pos.Z) * scale
				); 
			}
		}
	}
	btCollisionShape *shape = new btBvhTriangleMeshShape(trimesh, false);
	
	float mass = 0.0f; 
	btTransform Transform; 
	Transform.setIdentity(); 
	Transform.setOrigin(btVector3(0, 0, 0)); 

	btDefaultMotionState *mstate = new btDefaultMotionState(Transform);

	// Add mass
	btVector3 inertia;
	shape->calculateLocalInertia(mass, inertia);

	// Create the rigid body object
	btRigidBody *body = new btRigidBody(mass, mstate, shape, inertia);

	// Add it to the world
	World->addRigidBody(body, COLLIDE_WORLD, COLLIDE_FRAME | COLLIDE_PROP);
}

void Application::_dynamicsTickCallback(btDynamicsWorld *world, btScalar timeStep) {
	Application *self = (Application*)world->getWorldUserInfo(); 	

	int numManifolds = world->getDispatcher()->getNumManifolds();
    for (int i=0;i<numManifolds;i++)
    {
        btPersistentManifold* contactManifold =  world->getDispatcher()->getManifoldByIndexInternal(i);
        const btCollisionObject* obA = static_cast<const btCollisionObject*>(contactManifold->getBody0());
        const btCollisionObject* obB = static_cast<const btCollisionObject*>(contactManifold->getBody1());

        int numContacts = contactManifold->getNumContacts();
        for (int j=0;j<numContacts;j++)
        {
            btManifoldPoint& pt = contactManifold->getContactPoint(j);
            if (pt.getDistance()<0.f)
            {
                const btVector3& ptA = pt.getPositionWorldOnA();
                const btVector3& ptB = pt.getPositionWorldOnB();
                const btVector3& normalOnB = pt.m_normalWorldOnB;
				
				self->onCollision(obA, obB); 	
            }
        }
    }
}
 
static int paused = 0; 
Application::Application()
//	sock(true){
{
	mRCThrottle = 0; 
	mRCPitch = 0.5; 
	mRCYaw = 0.5; 
	mRCRoll = 0.5; 
	mRCAux1 = 0.2; 
	mRCAux2 = 0.2; 

	memset(_key_down, 0, sizeof(_key_down)); 

	_camera_mode = CAMERA_THIRD_PERSON; 

	_spin = 0; 
	_angle = 0; 

	initSharedMemory(); 

	irrDevice = createDevice(video::EDT_OPENGL, dimension2d<u32>(640, 480), 32, false, false, false, this);
	irrGUI = irrDevice->getGUIEnvironment();
	irrTimer = irrDevice->getTimer();
	_scene = irrDevice->getSceneManager();
	_drv = irrDevice->getVideoDriver();
	_scene->setAmbientLight(video::SColorf(1.0f, 1.0f, 1.0f, 1.0f)); 
	irrDevice->getCursorControl()->setVisible(0);

	// load a map
	irrDevice->getFileSystem()->addFileArchive("base");
	irrDevice->getFileSystem()->addFileArchive("base/pak0.pk3");
	//irrDevice->getFileSystem()->addFileArchive("base/map-20kdm2.pk3");
	//irrDevice->getFileSystem()->addFileArchive("base/q3dmp29.pk3");
	irrDevice->getFileSystem()->addFileArchive("base/q3dmp23.pk3");
	//irrDevice->getFileSystem()->addFileArchive("base/q3dmp29.pk3");
	//irrDevice->getFileSystem()->addFileArchive("base/q3dmp23.pk3");

	//scene::IQ3LevelMesh *mesh = (scene::IQ3LevelMesh*)_scene->getMesh("q3dmp29.bsp"); 
	//scene::IQ3LevelMesh *mesh = (scene::IQ3LevelMesh*)_scene->getMesh("q3dmp29.bsp"); 
	scene::IQ3LevelMesh *mesh = (scene::IQ3LevelMesh*)_scene->getMesh("q3dmp23.bsp"); 
	if(!mesh){
		printf("could not load map mesh!\n"); 
		exit(0); 
	}

	_scene->getParameters()->setAttribute(scene::ALLOW_ZWRITE_ON_TRANSPARENT, true); 
	// the additional mesh can be quite huge and is unoptimized
	scene::IMesh * const additional_mesh = mesh->getMesh(quake3::E_Q3_MESH_ITEMS);

	scene::IMesh * const geom = mesh->getMesh(quake3::E_Q3_MESH_GEOMETRY); 
	scene::ISceneNode *node = _scene->addOctreeSceneNode(geom, 0, -1, 4096);
	
	node->setScale(core::vector3df(Q3_WORLD_SCALE, Q3_WORLD_SCALE, Q3_WORLD_SCALE)); 
	//node->setPosition(core::vector3df(-130,-14.4,-124.9));

	for ( u32 i = 0; i!= additional_mesh->getMeshBufferCount(); ++i ){
		const IMeshBuffer* meshBuffer = additional_mesh->getMeshBuffer(i);
		const video::SMaterial& material = meshBuffer->getMaterial();

		// The ShaderIndex is stored in the material parameter
		const s32 shaderIndex = (s32) material.MaterialTypeParam2;

		// the meshbuffer can be rendered without additional support, or it has no shader
		const quake3::IShader *shader = mesh->getShader(shaderIndex);
		if (!shader){
			continue;
		}

		scene::ISceneNode *n = _scene->addQuake3SceneNode(meshBuffer, shader);
		node->addChild(n); 

	}
	ITriangleSelector *sel = _scene->createOctreeTriangleSelector(geom, node, 128); 
	node->setTriangleSelector(sel); 
	sel->drop(); 

	// Initialize bullet
	CollisionConfiguration = new btDefaultCollisionConfiguration();
	BroadPhase = new btAxisSweep3(btVector3(-1000, -1000, -1000), btVector3(1000, 1000, 1000));
	Dispatcher = new btCollisionDispatcher(CollisionConfiguration);
	Solver = new btSequentialImpulseConstraintSolver();
	World = new btDiscreteDynamicsWorld(Dispatcher, BroadPhase, Solver, CollisionConfiguration);
	World->setGravity(btVector3(0, -9.82, 0)); 
	World->setInternalTickCallback(_dynamicsTickCallback); 
	World->setWorldUserInfo(this); 

	add_triangle_mesh(additional_mesh, Q3_WORLD_SCALE); 
	add_triangle_mesh(geom, Q3_WORLD_SCALE); 
	_player_start = get_player_start(mesh) * Q3_WORLD_SCALE; 
	// Add camera
	//mCamera = _scene->addCameraSceneNodeFPS(0, 100, 0.01);
	mCamera = _scene->addCameraSceneNode();
	mCamera->setNearValue(0.01); 
	mCamera->setFarValue(1000.0); 
	mCamera->setPosition(vector3df(_player_start.x, _player_start.y, _player_start.z));
	mCamera->setRotation(vector3df(0, 0, 0)); 
	//Camera->setUpVector(vector3df(0, 0, 1.0)); 
	//Camera->setTarget(vector3df(1, 0, 0));
	
	// Preload textures
	_drv->getTexture("ice0.jpg");
	_drv->getTexture("rust0.jpg");

	// Create the initial scene
	_scene->addLightSceneNode(0, core::vector3df(2, 5, -2), SColorf(4, 4, 4, 1));
	_scene->addLightSceneNode(0, core::vector3df(2, -5, -2), SColorf(4, 4, 4, 1));
	CreateStartScene();

	// Main loop
	TimeStamp = irrTimer->getTime();

	_aircraft = new TiltXRotor(this);
	_aircraft->init(); 
	
	_aircraft->setPosition(_player_start + glm::vec3(0, 4, 0)); 
	_aircraft->setHomeLocation(glm::vec3(149.165230, 584, -35.363261)); 

	// platform
	CreateBox(btVector3(_player_start.x, _player_start.y-3, _player_start.z), vector3df(5.0f, 1.5f, 5.0f), 0.0f, "ice0.jpg");
	//sock.bind("127.0.0.1", 9002); 
	//sock.set_blocking(false); 

	::fprintf(stdout, "QuadSim started!\n"); 
}

void Application::onCollision(const btCollisionObject *a, const btCollisionObject *b){
	_aircraft->onCollision(a, b); 
}

void Application::initSharedMemory(){
	int out = shmget(9005, 0, 0666); 
	int in = shmget(9003, 0, 0666); 
	
	struct shmid_ds stat; 
	// delete existing ones
	if(out >= 0) { 
		if(shmctl(out, IPC_STAT, &stat) != -1 && stat.shm_segsz != sizeof(client_packet)){ 
			printf("Deleting 'out' shared segment because size does not match!\n"); 
			shmctl(out, IPC_RMID, NULL); 
			out = shmget(9005, sizeof(struct client_packet), IPC_CREAT | 0666); 
		} else {
			out = shmget(9005, sizeof(struct client_packet), 0666); 
		}
	} else {
		out = shmget(9005, sizeof(struct client_packet), IPC_CREAT | 0666); 
	}

	if(in >= 0) {
		if(shmctl(in, IPC_STAT, &stat) != -1 && stat.shm_segsz != sizeof(server_packet)){ 
			printf("Deleting 'in' shared segment because size does not match!\n"); 
			shmctl(in, IPC_RMID, NULL); 
			in = shmget(9003, sizeof(struct server_packet), IPC_CREAT | 0666); 
		} else {
			in = shmget(9003, sizeof(struct server_packet), 0666); 
		}
	} else {
		in = shmget(9003, sizeof(struct server_packet), IPC_CREAT | 0666); 
	}
	
	if(out < 0 || in < 0) {
		perror("shmget"); 
		printf("Could not get shared memory segment for communication with autopilot! Exiting..\n"); 
		exit(1); 
	}

	_shmout = (char*)shmat(out, NULL, 0); 
	_shmin = (char*)shmat(in, NULL, 0); 

	if(_shmout == 0 || _shmin == 0){
		printf("Could not attach shared memory segment for communicaiton with autopilot!\n"); 
		exit(1); 
	}

	// clear the input memory
	memset(_shmin, 0, sizeof(server_packet)); 

	printf("Shared memory initialized.\n");
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

irr::video::IVideoDriver *Application::getVideoDriver(){
	return _drv; 
}

irr::scene::ISceneManager *Application::getSceneManager(){
	return _scene; 
}

btDiscreteDynamicsWorld *Application::getDynamicsWorld(){
	return World; 
}

bool Application::clipRay(const glm::vec3 &_start, const glm::vec3 &_end, glm::vec3 *end, glm::vec3 *norm) {
	ISceneCollisionManager *cm = _scene->getSceneCollisionManager(); 
	line3d<f32> ray; 
	ray.start = vector3df(_start.x, _start.y, _start.z); 
	ray.end = vector3df(_end.x, _end.y, _end.z); 
	triangle3df hittri; 
	vector3df isect; 
	ISceneNode *node = cm->getSceneNodeAndCollisionPointFromRay(ray, 
		isect, hittri, 0, 0); 
	if(node){
		end->x = isect.X; end->y = isect.Y, end->z = isect.Z; 
		return true; 
	} 
	*end = _end; 
	return false; 
	/*
	// bullet collision system
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
	*/
}

void Application::handleInput(double dt){
	// Throttle
	if(_key_down[KEY_KEY_W]){
		mRCThrottle += dt * 0.6; 	
	} else if(_key_down[KEY_KEY_S]){
		mRCThrottle -= dt * 0.6; 
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
		mRCPitch += dt * 0.8; 	
	} else if(_key_down[KEY_DOWN]){
		if(mRCPitch > 0.5) mRCPitch = 0.5; 
		mRCPitch -= dt * 0.8; 
	} else {
		mRCPitch = 0.5; 
	}	

	// Roll 
	if(_key_down[KEY_RIGHT]){
		if(mRCRoll < 0.5) mRCRoll = 0.5; 
		mRCRoll += dt * 0.8; 	
	} else if(_key_down[KEY_LEFT]){
		if(mRCRoll > 0.5) mRCRoll = 0.5; 
		mRCRoll -= dt * 0.8; 
	} else {
		mRCRoll = 0.5; 
	}	

	if(_key_down[KEY_KEY_I]){
		mRCAux1 += dt * 0.2; 	
	} else if(_key_down[KEY_KEY_K]){
		mRCAux1 -= dt * 0.2; 
	} 

	if(_key_down[KEY_KEY_J]){
		mRCAux2 += dt * 0.2; 	
	} else if(_key_down[KEY_KEY_L]){
		mRCAux2 -= dt * 0.2; 
	} 

	if(mRCThrottle > 1.0f) mRCThrottle = 1.0f; if(mRCThrottle < 0.0f) mRCThrottle = 0.0f; 
	if(mRCYaw > 1.0f) mRCYaw = 1.0f; if(mRCYaw < 0.0f) mRCYaw = 0.0f; 
	if(mRCPitch > 1.0f) mRCPitch = 1.0f; if(mRCPitch < 0.0f) mRCPitch = 0.0f; 
	if(mRCRoll > 1.0f) mRCRoll = 1.0f; if(mRCRoll < 0.0f) mRCRoll = 0.0f; 
	if(mRCAux1 > 1.0f) mRCAux1 = 1.0f; if(mRCAux1 < 0.0f) mRCAux1 = 0.0f; 
	if(mRCAux2 > 1.0f) mRCAux2 = 1.0f; if(mRCAux2 < 0.0f) mRCAux2 = 0.0f; 
}

static const glm::vec3 dir[6] = {
	glm::vec3(0, 0, -1), 
	glm::vec3(0, 0, 1),
	glm::vec3(-1, 0, 0),
	glm::vec3(1, 0, 0),
	glm::vec3(0, -1, 0),
	glm::vec3(0, 1, 0)
}; 

void Application::renderRange(){
	glm::vec3 pos = _aircraft->getPosition(); 
	glm::quat rot = _aircraft->getRotation(); 
	
	for(int c = 0; c < 6; c++){
		glm::vec3 hit = pos + rot * dir[c] * _range_scan[c]; 
		SColor color; 

		if(_range_scan[c] < 1.99f)
			color = SColor( 255, 0, 255, 0 ); 
		else
			color = SColor( 255, 255, 0, 0); 

		_drv->draw3DBox(aabbox3df(vector3df(hit.x - 0.1, hit.y - 0.1, hit.z - 0.1), vector3df(hit.x + 0.1, hit.y + 0.1, hit.z + 0.1)), color); 
		_drv->draw3DLine(vector3df(pos.x, pos.y, pos.z),
			vector3df(hit.x, hit.y, hit.z), 
			color);
	}
}

void Application::scanRange(){
	glm::vec3 pos = _aircraft->getPosition(); 
	glm::quat rot = _aircraft->getRotation(); 
	
	for(int c = 0; c < 6; c++){
		glm::vec3 end; 
		clipRay(pos, pos + rot * dir[c] * 2.0f, &end); 
		_range_scan[c] = glm::length(end - pos); 	
	}
}

void Application::updateCamera(){
	glm::quat rot = _aircraft->getRotation(); 
	glm::vec3 pos = _aircraft->getPosition(); 
	glm::vec3 e = glm::eulerAngles(rot); 

	switch(_camera_mode){
		case CAMERA_THIRD_PERSON: {
			glm::vec3 cpos = pos + rot * glm::vec3(0, 0, -2); 
			cpos.y = pos.y + 1; 
			glm::vec3 cnorm; 
			clipRay(pos, cpos, &cpos, &cnorm); 
			mCamera->setPosition(vector3df(cpos.x, cpos.y, cpos.z)); 
			mCamera->setTarget(vector3df(pos.x, pos.y, pos.z));
		} break; 
		case CAMERA_SIDE: {
			glm::vec3 cpos = pos + rot * glm::vec3(2, 0, 0); 
			cpos.y = pos.y + 1; 
			glm::vec3 cnorm; 
			clipRay(pos, cpos, &cpos, &cnorm); 
			mCamera->setPosition(vector3df(cpos.x, cpos.y, cpos.z)); 
			mCamera->setTarget(vector3df(pos.x, pos.y, pos.z));
		} break; 
		case CAMERA_FIRST_PERSON: {
			glm::vec3 cp = pos + rot * glm::vec3(0, 0.5, 0.5); 
			glm::vec3 ct = pos + rot * glm::vec3(0, 0.6, 1.0); 
			glm::vec3 cu = rot * glm::vec3(0, 1.0, 0.0); 
			mCamera->setPosition(vector3df(cp.x, cp.y, cp.z)); 
			mCamera->setTarget(vector3df(ct.x, ct.y, ct.z));
			mCamera->setUpVector(vector3df(cu.x, cu.y, cu.z));
			mCamera->setRotation(vector3df(glm::degrees(e.x), glm::degrees(e.y), glm::degrees(e.z))); 
		} break; 
		default: 
			break; 
	}
}

void Application::run(){
	long long time = irrTimer->getTime(); 
	long long dti = (time - TimeStamp); 
	double dt = dti * 0.001f;
	TimeStamp = time; 

	if(!paused){
		World->stepSimulation(dt, 400, 0.001);

		scanRange(); 
		updateNetwork(dt); 
		updateCamera(); 
		handleInput(dt); 

		_aircraft->update(dt);

		// Relay the object's orientation to irrlicht
		for(list<btRigidBody *>::Iterator Iterator = Objects.begin(); Iterator != Objects.end(); ++Iterator) {
			UpdateRender(*Iterator);
		}	

	}
	
	_drv->beginScene(true, true, SColor(255, 20, 0, 0));
	
	_scene->drawAll();
	
	// debug
	_aircraft->render(); 
	renderRange(); 

	irrGUI->drawAll();
	_drv->endScene();
	irrDevice->run();
}


bool Application::OnEvent(const SEvent &ev) {
	if(ev.EventType == EET_KEY_INPUT_EVENT){
		_key_down[ev.KeyInput.Key] = ev.KeyInput.PressedDown; 
	}

	if(ev.EventType == EET_KEY_INPUT_EVENT && !ev.KeyInput.PressedDown) {
		switch(ev.KeyInput.Key) {
			case KEY_ESCAPE:
				Done = true;
			break;
			case KEY_KEY_Q: 
				_aircraft->setPosition(_player_start + glm::vec3(0, 4, 0)); 
				_aircraft->setAngularVelocity(glm::vec3(0, 0, 0)); 
				_aircraft->setLinearVelocity(glm::vec3(0, 0, 0)); 
				break; 
			case KEY_KEY_O: 
				_spin += 0.5; 
				break; 
			case KEY_KEY_L: 
				_spin -= 0.5; 
				break; 
			case KEY_KEY_C: 
				_camera_mode = (_camera_mode + 1) % CAMERA_MODE_COUNT; 
				break; 
			case KEY_KEY_6: {
				static int id = 0; 
				glm::vec3 vel[4] = {
					glm::vec3(0, 0, 0), 
					glm::vec3(1, 0, 0), 
					glm::vec3(0, 1, 0), 
					glm::vec3(0, 0, 1)
				}; 
				_aircraft->setRotation(glm::quat(1, 0, 0, 0)); 
				_aircraft->setAngularVelocity(vel[id++]); 
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
				_aircraft->setPosition(glm::vec3(0, 10, 0)); 
				_aircraft->setRotation(rots[id++] * y); 
				_aircraft->setAngularVelocity(glm::vec3(0, 0, 0)); 
				id = id % (sizeof(rots) / sizeof(rots[0])); 
				break; 
			}
			case KEY_KEY_0: {
				if(_mode != MODE_CLIENT_SIM){
					printf("Can only calibrate in client sim mode!\n"); 
					break; 
				}
				_calibration = !_calibration; 
				if(!_calibration){
					//_aircraft->setSimulationOn(true); 
					World->setGravity(btVector3(0, -9.82, 0)); 
					printf("Left calibration mode!\n"); 
				} else {
					//_aircraft->setSimulationOn(false); 
					World->setGravity(btVector3(0, 0, 0)); 
					printf("Entered calibration mode\n");
				}
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
				_aircraft->setPosition(glm::vec3(0, 10, 0)); 
				_aircraft->setAngularVelocity(glm::vec3(0, -1, 0)); 
				_aircraft->setRotation(rots[id++]); 
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
}

// Creates a base box
void Application::CreateStartScene() {
	ClearObjects();
	//CreateBox(btVector3(0.0f, 0.0f, 0.0f), vector3df(50.0f, 0.5f, 50.0f), 0.0f, "ice0.jpg");
	//CreateBox(btVector3(20.0f, 0.0f, 0.0f), vector3df(0.5f, 50.0f, 50.0f), 0.0f, "ice0.jpg");
	//CreateBox(btVector3(0.0f, 0.0f, 20.0f), vector3df(50.0f, 50.0f, 0.5f), 0.0f, "ice0.jpg");
}

// Create a box rigid body
btRigidBody *Application::CreateBox(const btVector3 &TPosition, const vector3df &TScale, btScalar TMass, const char *texture) {

	ISceneNode *Node = _scene->addCubeSceneNode(1.0f);
	Node->setScale(TScale);
	Node->setMaterialFlag(EMF_LIGHTING, 1);
	Node->setMaterialFlag(EMF_NORMALIZE_NORMALS, true);
	Node->setMaterialTexture(0, _drv->getTexture(texture));

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
	World->addRigidBody(RigidBody, COLLIDE_WORLD, 0xffff);
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
	struct server_packet pkt; 

	static glm::vec3 angles(0, 0, 0); 

	//if(sock.recv(&pkt, sizeof(pkt), 1) == sizeof(pkt)){
	memcpy(&pkt, _shmin, sizeof(server_packet)); 
	float roll = pkt.euler[0]; 
	float pitch = pkt.euler[1]; 
	float yaw = pkt.euler[2]; 

	_mode = pkt.mode; 
	if(pkt.mode == MODE_SERVER_SIM){
		//_aircraft->setSimulationOn(false); 
		//_aircraft->setFrameType((Copter::frame_type_t)pkt.frame); 
		_aircraft->setPosition(glm::vec3(pkt.pos[1], -pkt.pos[2], pkt.pos[0])); 
		// yaw = y, pitch = x, roll = z; 
		glm::quat rr(cos(-roll / 2), 0, 0, sin(-roll / 2));  
		glm::quat rp(cos(-pitch / 2), sin(-pitch / 2), 0, 0);  
		glm::quat ry(cos(yaw / 2), 0, sin(yaw / 2), 0);  

		_aircraft->setRotation(ry * rp * rr); 
		_aircraft->setLinearVelocity(glm::vec3(pkt.vel[1], -pkt.vel[2], pkt.vel[0])); 
		_aircraft->setAccelerometer(glm::vec3(pkt.acc[1], -pkt.acc[2], pkt.acc[0])); 
		_aircraft->setMagneticField(glm::vec3(pkt.mag[1], -pkt.mag[2], pkt.mag[0])); 
	
		//glm::vec3 lm = _aircraft->calcMagFieldIntensity(); 
		//printf("mas(%f %f %f)\n", pkt.mag[1], -pkt.mag[2], pkt.mag[0]); 
		//printf("mal(%f %f %f)\n", lm.x, lm.y, lm.z);  
		//printf("pos(%f %f %f) vel(%f %f %f)\n", pkt.pos[0], pkt.pos[1], pkt.pos[2], pkt.vel[0], pkt.vel[1], pkt.vel[2]); 
	} else if(pkt.mode == MODE_CLIENT_SIM){
		//_aircraft->setSimulationOn(true); 
	}

	//printf("servo(%d %d %d %d) rpy(%f %f %f)\n", pkt.servo[0], pkt.servo[1], pkt.servo[2], pkt.servo[3], roll, pitch, yaw); 
	for(unsigned c = 0; c < 8; c++){
		_aircraft->setOutput(c, (pkt.servo[c] - 1000) / 1000.0f); 
	}

	// send our state
	struct client_packet state; 
	memset(&state, 0, sizeof(state)); 

	glm::vec3 pos = _aircraft->getPosition(); 
	glm::quat rot = _aircraft->getRotation(); 
	glm::vec3 accel = _aircraft->getAccel(); 
	glm::ivec3 loc = _aircraft->getLocation(); 
	glm::vec3 mag = _aircraft->getMagneticField();
	glm::vec3 vel = _aircraft->getVelocity(); 
	glm::vec3 gyro = _aircraft->getGyro(); 
	glm::vec3 euler = glm::eulerAngles(rot); 

	state.id = _sent_count++; 
	state.euler[0] = euler.z; state.euler[1] = euler.x; state.euler[2] = euler.y; 
	state.gyro[0] = -gyro.z; state.gyro[1] = -gyro.x; state.gyro[2] = gyro.y; 
	state.accel[0] = accel.z; state.accel[1] = accel.x; state.accel[2] = -accel.y; 
	//state.accel[0] = 0; state.accel[1] = 0; state.accel[2] = -9.82; 
	state.pos[0] = pos.z; state.pos[1] = pos.x; state.pos[2] = -pos.y; 
	state.loc[0] = loc.z; state.loc[1] = loc.x; state.loc[2] = loc.y; 
	state.mag[0] = mag.z; state.mag[1] = mag.x; state.mag[2] = -mag.y; 
	state.vel[0] = vel.z; state.vel[1] = vel.x; state.vel[2] = -vel.y; 
	
	state.rcin[0] = mRCRoll; 
	state.rcin[1] = mRCPitch; 
	state.rcin[2] = mRCThrottle; 
	state.rcin[3] = mRCYaw; 
	state.rcin[4] = mRCAux1; 
	state.rcin[5] = mRCAux2; 

	memcpy(state.range, _range_scan, min(sizeof(state.range), sizeof(_range_scan))); 

#if 0 
	printf("sending: acc(%f %f %f) gyr(%f %f %f) mag(%f %f %f) vel(%f %f %f) loc(%d %d %d) rc(%f %f %f %f)\n", 
		state.accel[0], state.accel[1], state.accel[2],
		state.gyro[0], state.gyro[1], state.gyro[2],
		state.mag[0], state.mag[1], state.mag[2],
		state.vel[0], state.vel[1], state.vel[2], 
		state.loc[0], state.loc[1], state.loc[2],
		state.rcin[0], state.rcin[1], state.rcin[2], state.rcin[3]); 
#endif
	memcpy(_shmout, &state, sizeof(state)); 
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
