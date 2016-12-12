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
#include "SITLInterface.h"

#include <sys/types.h>
#include <sys/ipc.h>
#include <sys/shm.h>
#include <stdio.h>
#include <glob.h>

#include "GenericXRotor.h"
#include "TiltXRotor.h"

#define MODE_CLIENT_SIM 0
#define MODE_SERVER_SIM 1

#define Q3_WORLD_SCALE glm::vec3(0.015f, 0.015f, 0.015f)

struct server_packet {
	unsigned long long time; 
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

static void _add_meshbuffer_to_trimesh(btTriangleMesh *trimesh, irr::video::S3DVertex2TCoords *mb_vertices, u32 vertex_count, u16 *mb_indices, u32 index_count, const glm::vec3 &scale){
	int triCount = 0;
	if(!mb_vertices) return;
	if(mb_indices && index_count > 0){
		for (unsigned int j=0; j < index_count; j += 3){
			video::S3DVertex2TCoords *v0 = &mb_vertices[mb_indices[j]]; 
			video::S3DVertex2TCoords *v1 = &mb_vertices[mb_indices[j+1]]; 
			video::S3DVertex2TCoords *v2 = &mb_vertices[mb_indices[j+2]]; 
			//::printf("tri: %f %f %f, %f %f %f, %f %f %f\n", v0->Pos.X, v0->Pos.Y, v0->Pos.Z, v1->Pos.X, v1->Pos.Y, v1->Pos.Z, v2->Pos.X, v2->Pos.Y, v2->Pos.Z); 
			trimesh->addTriangle(
				btVector3(v0->Pos.X * scale.x, v0->Pos.Y * scale.y, v0->Pos.Z * scale.z), 
				btVector3(v1->Pos.X * scale.x, v1->Pos.Y * scale.y, v1->Pos.Z * scale.z), 
				btVector3(v2->Pos.X * scale.x, v2->Pos.Y * scale.y, v2->Pos.Z * scale.z)
			); 
			triCount++;
		}
	} else {
		// nonindexed buffer
		for (unsigned int j=0; j < vertex_count; j += 3){
			video::S3DVertex2TCoords *v0 = &mb_vertices[j];
			video::S3DVertex2TCoords *v1 = &mb_vertices[j+1];
			video::S3DVertex2TCoords *v2 = &mb_vertices[j+2];
			trimesh->addTriangle(
				btVector3(v0->Pos.X * scale.x, v0->Pos.Y * scale.y, v0->Pos.Z * scale.z),
				btVector3(v1->Pos.X * scale.x, v1->Pos.Y * scale.y, v1->Pos.Z * scale.z),
				btVector3(v2->Pos.X * scale.x, v2->Pos.Y * scale.y, v2->Pos.Z * scale.z)
			); 
			triCount++;
		}
	}
	//printf("processed %d vertices, %d indices\n", vertex_count, index_count);
}

static void _add_trimesh_to_world(btDynamicsWorld *World, btTriangleMesh *trimesh, btVector3 pos = btVector3(0, 0, 0)){
	btCollisionShape *shape = new btBvhTriangleMeshShape(trimesh, false);
	
	float mass = 0.0f; 
	btTransform Transform; 
	Transform.setIdentity(); 
	Transform.setOrigin(pos); 

	btDefaultMotionState *mstate = new btDefaultMotionState(Transform);

	// Add mass
	btVector3 inertia;
	shape->calculateLocalInertia(mass, inertia);

	// Create the rigid body object
	btRigidBody *body = new btRigidBody(mass, mstate, shape, inertia);

	// Add it to the world
	World->addRigidBody(body, COLLIDE_WORLD, COLLIDE_FRAME | COLLIDE_PROP);
}

void Application::add_triangle_mesh(IMesh *mesh, const glm::vec3 &scale){
	if(!mesh) return;

	btTriangleMesh *trimesh = new btTriangleMesh();

	//int triCount = 0;
	for (unsigned int i = 0; i < mesh->getMeshBufferCount(); i++){
		irr::scene::IMeshBuffer* mb = mesh->getMeshBuffer(i);
		printf("adding triangle mesh: %d vertices, %d indices\n", mb->getVertexCount(), mb->getIndexCount());
	  	if(!mb) continue;

		if (mb->getVertexType()==irr::video::EVT_STANDARD){
			::printf("Standard vertex\n"); 
		}
		if (mb->getVertexType()==irr::video::EVT_2TCOORDS){
			irr::video::S3DVertex2TCoords* mb_vertices=(irr::video::S3DVertex2TCoords*)mb->getVertices();
			_add_meshbuffer_to_trimesh(trimesh, mb_vertices, mb->getVertexCount(), mb->getIndices(), mb->getIndexCount(), scale);
		}
	}
	_add_trimesh_to_world(World, trimesh);
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
				/*
                const btVector3& ptA = pt.getPositionWorldOnA();
                const btVector3& ptB = pt.getPositionWorldOnB();
                const btVector3& normalOnB = pt.m_normalWorldOnB;
				*/
				self->onCollision(obA, obB); 	
            }
        }
    }
}
 
static int paused = 0; 
Application::Application(){
	_snd = new SoundSystem();
	mRCThrottle = 0; 
	mRCPitch = 0.5; 
	mRCYaw = 0.5; 
	mRCRoll = 0.5; 
	mRCAux1 = 0.5; 
	mRCAux2 = 0.5; 

	memset(_key_down, 0, sizeof(_key_down)); 

	_camera_mode = CAMERA_THIRD_PERSON; 

	_spin = 0; 
	_angle = 0; 

	if(initSITL() != 0){
		fprintf(stderr, "FATAL: could not initialize sitl module!\n");
		exit(1);
	}

	_dev = createDevice(video::EDT_OPENGL, dimension2d<u32>(640, 480), 16, false, false, false, this);
	irrGUI = _dev->getGUIEnvironment();
	irrTimer = _dev->getTimer();
	_scene = _dev->getSceneManager();
	_drv = _dev->getVideoDriver();
	//_drv->setTextureCreationFlag(ETCF_NO_ALPHA_CHANNEL, false);
	//_drv->setTextureCreationFlag(ETCF_ALWAYS_32_BIT, false);

	_scene->setAmbientLight(video::SColorf(1.0f, 1.0f, 1.0f, 1.0f)); 
	_dev->getCursorControl()->setVisible(0);

	IGUIEnvironment *_env = _dev->getGUIEnvironment();
	_env->addImage(_drv->getTexture("base/logo.png"),
            position2d<int>(10,10));

	initJoystick(); 

	loadMediaArchives(); 
	
	// Initialize bullet
	CollisionConfiguration = new btDefaultCollisionConfiguration();
	BroadPhase = new btAxisSweep3(btVector3(-1000, -1000, -1000), btVector3(1000, 1000, 1000));
	Dispatcher = new btCollisionDispatcher(CollisionConfiguration);
	Solver = new btSequentialImpulseConstraintSolver();
	World = new btDiscreteDynamicsWorld(Dispatcher, BroadPhase, Solver, CollisionConfiguration);
	World->setGravity(btVector3(0, -9.82, 0)); 
	World->setInternalTickCallback(_dynamicsTickCallback); 
	World->setWorldUserInfo(this); 

	// Add camera
	//mCamera = _scene->addCameraSceneNodeFPS(0, 100, 0.01);
	mCamera = _scene->addCameraSceneNode();
	mCamera->setNearValue(0.01); 
	mCamera->setFarValue(1000.0); 
	mCamera->setRotation(vector3df(0, 0, 0)); 
	mCamera->setFOV(glm::radians(90.0f)); 
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
	
	//_aircraft->setPosition(_player_start + glm::vec3(0, 4, 0)); 
	_aircraft->setHomeLocation(glm::vec3(149.165230, 584, -35.363261)); 

	// platform
	//CreateBox(btVector3(_player_start.x, _player_start.y-3, _player_start.z), vector3df(5.0f, 1.5f, 5.0f), 0.0f, "ice0.jpg");
	//CreateBox(btVector3(_player_start.x, _player_start.y-3, _player_start.z), vector3df(500.0f, 1.5f, 500.0f), 0.0f, "ice0.jpg");
	//CreateBox(btVector3(0, 1, 0), vector3df(500.0f, 1.5f, 500.0f), 0.0f, "ice0.jpg");
	//sock.bind("127.0.0.1", 9002); 
	//sock.set_blocking(false); 

	CreateBox(btVector3(0.0f, 2.0f, 4.0f), vector3df(0.5f, 2.0f, 0.5f), 5.0f, "sand.jpg");
	CreateBox(btVector3(2.0f, 2.0f, 4.0f), vector3df(0.5f, 2.0f, 0.5f), 5.0f, "sand.jpg");
	CreateBox(btVector3(1.0f, 3.0f, 4.0f), vector3df(3.0f, 1.0f, 1.0f), 5.0f, "sand.jpg");

	_led_on = _drv->getTexture("led_red_on.png");
	_led_off = _drv->getTexture("led_red_off.png");
	_drv->makeColorKeyTexture(_led_on, core::position2d<s32>(0, 0));
	_drv->makeColorKeyTexture(_led_off, core::position2d<s32>(0, 0));

	::fprintf(stdout, "QuadSim started!\n"); 
}

int Application::loadMap(const char *filename){
	// load a map
	scene::IQ3LevelMesh *mesh = (scene::IQ3LevelMesh*)_scene->getMesh(filename); 

	if(!mesh){
		printf("could not load map mesh!\n"); 
		return -1; 
	}

	_scene->getParameters()->setAttribute(scene::ALLOW_ZWRITE_ON_TRANSPARENT, true); 
	// the additional mesh can be quite huge and is unoptimized
	scene::IMesh * const additional_mesh = mesh->getMesh(quake3::E_Q3_MESH_ITEMS);

	scene::IMesh * const geom = mesh->getMesh(quake3::E_Q3_MESH_GEOMETRY); 
	scene::ISceneNode *node = _scene->addOctreeSceneNode(geom, 0, -1, 4096);
	
	node->setScale(core::vector3df(Q3_WORLD_SCALE.x, Q3_WORLD_SCALE.y, Q3_WORLD_SCALE.z)); 
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

	add_triangle_mesh(additional_mesh, Q3_WORLD_SCALE); 
	add_triangle_mesh(geom, Q3_WORLD_SCALE); 

	// add terrain scene node
    scene::ITerrainSceneNode* terrain = _scene->addTerrainSceneNode(
        "base/terrain-heightmap.bmp",
        0,                  // parent node
        -1,                 // node id
        core::vector3df(-1000.f, -200.f, -1000.f),     // position
        core::vector3df(0.f, 0.f, 0.f),     // rotation
        core::vector3df(10.0f, 5.0f, 10.0f),  // scale
        video::SColor ( 255, 255, 255, 255 ),   // vertexColor
        5,                  // maxLOD
        scene::ETPS_17,             // patchSize
        4                   // smoothFactor
        );

    terrain->setMaterialFlag(video::EMF_LIGHTING, false);

    terrain->setMaterialTexture(0,
            _drv->getTexture("base/terrain-texture.jpg"));
    terrain->setMaterialTexture(1,
            _drv->getTexture("base/terrain-detail.jpg"));
    terrain->setTriangleSelector(sel);  
    terrain->setMaterialType(video::EMT_DETAIL_MAP);

    terrain->scaleTexture(20.0f, 20.0f);

	printf("adding terrain to collision world..\n");
	scene::CDynamicMeshBuffer* buffer = new scene::CDynamicMeshBuffer(video::EVT_2TCOORDS, video::EIT_16BIT);
    terrain->getMeshBufferForLOD(*buffer, 0);
	btTriangleMesh *trimesh = new btTriangleMesh();
	printf("terrain has %d vertices and %d indices\n", buffer->getVertexCount(), buffer->getIndexCount());
	_add_meshbuffer_to_trimesh(trimesh, (video::S3DVertex2TCoords*)buffer->getVertices(), buffer->getVertexCount(), buffer->getIndices(), buffer->getIndexCount(), glm::vec3(10.0f, 5.0f, 10.0f));
	_add_trimesh_to_world(World, trimesh, btVector3(-1000.f, -200.f, -1000.f));

	sel->drop();

	// create sky
	_drv->setTextureCreationFlag(video::ETCF_CREATE_MIP_MAPS, false);
	_scene->addSkyDomeSceneNode(_drv->getTexture("base/skydome_2k.jpg"), 16, 8, 0.95f, 2.0f);
	_drv->setTextureCreationFlag(video::ETCF_CREATE_MIP_MAPS, true);

	//_drv->setFog(video::SColor(0,138,125,81), video::EFT_FOG_LINEAR, 250, 1000, .003f, true, false);

	_player_start = get_player_start(mesh) * Q3_WORLD_SCALE; 

	//_aircraft->setPosition(glm::vec3(10, 10, 10));
	printf("Player start found at: %f %f %f\n", _player_start.x, _player_start.y, _player_start.z);

	_aircraft->setPosition(_player_start + glm::vec3(0, 1, 0));
	mCamera->setPosition(vector3df(_player_start.x, _player_start.y, _player_start.z));

	return 0; 
}

int Application::loadReplay(const char *filename){
	if(_replay.load(filename) < 0){
		return -1; 
	}
	return 0; 
}

void Application::loadMediaArchives(){
	glob_t results; 
	_dev->getFileSystem()->addFileArchive("base");

	glob("base/pak*.pk3", 0, 0, &results); 
	glob("base/map*.pk3", GLOB_APPEND, 0, &results); 
	glob("base/[!p][!a][!k]*.pk3", GLOB_APPEND, 0, &results); 

	for(size_t c = 0; c < results.gl_pathc; c++){
		printf("Loading archive %s\n", results.gl_pathv[c]); 
		_dev->getFileSystem()->addFileArchive(results.gl_pathv[c]);
	}
}

void Application::onCollision(const btCollisionObject *a, const btCollisionObject *b){
	_aircraft->onCollision(a, b); 
}

int Application::initSITL(){
	sitl = SITLInterface::create(SITL_NINJAFLIGHT);
	if(!sitl) return -1;
	return 0;
}

Application::~Application(){
	// TODO: double check that we delete everything..
	ClearObjects();
	delete World;
	delete Solver;
	delete Dispatcher;
	delete BroadPhase;
	delete CollisionConfiguration;
	_dev->drop();
	delete _snd;
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

void Application::handleInputJoystick(double dt){
	float thr = -_joystickState.Axis[SEvent::SJoystickEvent::AXIS_Y] / 32767.0f; 
	float yaw = _joystickState.Axis[SEvent::SJoystickEvent::AXIS_X] / 32767.0f; 
	float pit = -_joystickState.Axis[SEvent::SJoystickEvent::AXIS_R] / 32767.0f; 
	float rll = _joystickState.Axis[SEvent::SJoystickEvent::AXIS_Z] / 32767.0f; 
	//float u = _joystickState.Axis[SEvent::SJoystickEvent::AXIS_U] / 32767.0f; 
	//float v = _joystickState.Axis[SEvent::SJoystickEvent::AXIS_V] / 32767.0f; 
	
	//printf("JOYSTICK: %f %f %f %f %f %f\n", thr, yaw, pit, rll, u, v); 

	mRCThrottle = (1.0f + thr) * 0.5f;
	mRCYaw = (1.0f + yaw) * 0.5f;
	mRCPitch = (1.0f + pit) * 0.5f;
	mRCRoll = (1.0f + rll) * 0.5f;
/*
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
	
	mRCPitch = -thr * 0.5f + 0.5f; 
	mRCRoll = yaw * 0.5f + 0.5f; 
	*/
}

void Application::handleInputKeyboard(double dt){
	// Throttle
	if(_key_down[KEY_KEY_W]){
		mRCThrottle += dt * 0.4; 	
	} else if(_key_down[KEY_KEY_S]){
		mRCThrottle -= dt * 0.4; 
	} else {
		
	}

	// Yaw
	if(_key_down[KEY_KEY_D]){
		if(mRCYaw < 0.5) mRCYaw = 0.5; 
		mRCYaw += dt * 0.8; 	
	} else if(_key_down[KEY_KEY_A]){
		if(mRCYaw > 0.5) mRCYaw = 0.5; 
		mRCYaw -= dt * 0.8; 
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
		mRCAux1 += dt * 0.3; 	
	} else if(_key_down[KEY_KEY_K]){
		mRCAux1 -= dt * 0.3; 
	} 

	if(_key_down[KEY_KEY_O]){
		mRCAux2 += dt * 0.3; 	
	} else if(_key_down[KEY_KEY_L]){
		mRCAux2 -= dt * 0.3; 
	} 
}

void Application::handleInput(double dt){
	if(_joystickEnabled) handleInputJoystick(dt); 
	else handleInputKeyboard(dt); 
	if(mRCThrottle > 1.0f) mRCThrottle = 1.0f; if(mRCThrottle < 0.0f) mRCThrottle = 0.0f; 
	if(mRCYaw > 1.0f) mRCYaw = 1.0f; if(mRCYaw < 0.0f) mRCYaw = 0.0f; 
	if(mRCPitch > 1.0f) mRCPitch = 1.0f; if(mRCPitch < 0.0f) mRCPitch = 0.0f; 
	if(mRCRoll > 1.0f) mRCRoll = 1.0f; if(mRCRoll < 0.0f) mRCRoll = 0.0f; 
	if(mRCAux1 > 1.0f) mRCAux1 = 1.0f; if(mRCAux1 < 0.0f) mRCAux1 = 0.0f; 
	if(mRCAux2 > 1.0f) mRCAux2 = 1.0f; if(mRCAux2 < 0.0f) mRCAux2 = 0.0f; 
}

void Application::initJoystick(){
	core::array<SJoystickInfo> joystickInfo;
	if(_dev->activateJoysticks(joystickInfo))
	{
		printf("Joystick support is enabled and %d joysticks are present!\n", joystickInfo.size());

		for(u32 joystick = 0; joystick < joystickInfo.size(); ++joystick)
		{
			printf("Joystick %d: \n", joystick); 
			printf("\tName: %s\n", joystickInfo[joystick].Name.c_str()); 
			printf("\tAxes: %d\n", joystickInfo[joystick].Axes); 
			printf("\tButtons: %d\n", joystickInfo[joystick].Buttons); 
			printf("\tHat is: "); 

			switch(joystickInfo[joystick].PovHat)
			{
			case SJoystickInfo::POV_HAT_PRESENT:
				printf("present\n"); 
				break;

			case SJoystickInfo::POV_HAT_ABSENT:
				printf("absent\n"); 
				break;

			case SJoystickInfo::POV_HAT_UNKNOWN:
			default:
				printf("unknown\n"); 
				break;
			}
		}
		_joystickEnabled = joystickInfo.size() > 0; 
	}
	else
	{
		printf("*** No joysticks found\n"); 
	}
}

static const glm::vec3 _dir[6] = {
	glm::vec3(0, 0, -1), 
	glm::vec3(0, 0, 1),
	glm::vec3(-1, 0, 0),
	glm::vec3(1, 0, 0),
	glm::vec3(0, -1, 0),
	glm::vec3(0, 1, 0)
}; 

static glm::vec3 dir[6]; 	

void Application::_draw_circle(const glm::vec3 &center, const glm::quat &rot, float r, int num_segments) { 
	float theta = 2 * 3.1415926 / float(num_segments); 
	float tangetial_factor = tanf(theta);//calculate the tangential factor 
	float radial_factor = cosf(theta);//calculate the radial factor 
	float x = r;//we start at angle = 0 
	float y = 0; 

	glm::vec3 v[2];

	for(int ii = 0; ii < num_segments; ii++) 
	{ 
		v[ii % 2] = center + rot * glm::vec3(x, 0, y);//output vertex 

		if(ii != 0 && (ii % 2) == 1){
			SColor color = SColor( 255, 0, 255, 0 ); 
			/*
			int idx = num_segments / 4
			if(_range_scan[2 + idx] < 1.99f)
				color = SColor( 255, 0, 255, 0 ); 
			else
				color = SColor( 255, 255, 0, 0); 
				*/
			_drv->draw3DLine(vector3df(v[0].x, v[0].y, v[0].z),
			vector3df(v[1].x, v[1].y, v[1].z), 
			color);
		}

		//calculate the tangential vector 
		//remember, the radial vector is (x, y) 
		//to get the tangential vector we flip those coordinates and negate one of them 
		float tx = -y; 
		float ty = x; 
		//add the tangential vector 
		x += tx * tangetial_factor; 
		y += ty * tangetial_factor; 
		//correct using the radial factor 
		x *= radial_factor; 
		y *= radial_factor; 
	} 
}

void Application::renderRange(){
	glm::vec3 pos = _aircraft->getPosition(); 
	glm::quat rot = _aircraft->getRotation(); 

	_draw_circle(pos, rot, 2.0, 16);
/*
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
*/
}

#define is_zero(X) (fabsf(X) < FLT_EPSILON)
#include <time.h>

#include <random>
#include <chrono>
unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
std::default_random_engine rng(seed);   
std::normal_distribution<double> norm_dist(0.0,1.0);

void Application::scanRange(){
	glm::vec3 pos = _aircraft->getPosition(); 
	glm::quat rot = _aircraft->getRotation(); 
	
	for(int c = 0; c < 6; c++){
		glm::vec3 end; 
		/*
		dir[c] = _dir[c]; 
		if(is_zero(dir[c].x)) dir[c].x = (static_cast <float> (rand()) / static_cast <float> (RAND_MAX) - 0.5f) * 0.3f;
		if(is_zero(dir[c].y)) dir[c].y = (static_cast <float> (rand()) / static_cast <float> (RAND_MAX) - 0.5f) * 0.3f;
		if(is_zero(dir[c].z)) dir[c].z = (static_cast <float> (rand()) / static_cast <float> (RAND_MAX) - 0.5f) * 0.3f;

		// add noise 
		float noisen = norm_dist(rng); 
		dir[c] += noisen * 0.1f; 
		if(rand() % 10 > 5) dir[c] *= 0.1; 
		*/
		clipRay(pos, pos + rot * _dir[c] * 2.0f, &end); 
		_range_scan[c] = glm::length(end - pos); 	
	}
	for(int c = 0; c < 4; c++){
		float dist = _range_scan[c];
		sitl->write_range(c * 90, (dist > 0)?(dist * 100):-1);
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
			glm::vec3 ct = pos + rot * glm::vec3(0, 0.5, 1.0); 
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

void Application::updateReplay(float dt){
	if(!_replay_mode) return; 
	if(_replay.step(dt)){
		//glm::vec3 pos = _replay.getPosition(); 
		//glm::quat rot = _replay.getRotation(); 

		//printf("keyframe: pos(%f %f %f)\n", pos.x, pos.y, pos.z); 

		//_aircraft->setPosition(pos); 
		//_aircraft->setRotation(rot); 
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
		updateReplay(dt); 

		_aircraft->update(dt);

		// Relay the object's orientation to irrlicht
		for(list<btRigidBody *>::Iterator Iterator = Objects.begin(); Iterator != Objects.end(); ++Iterator) {
			UpdateRender(*Iterator);
		}	

	}

	// run rendering much slower (at less than 30 fps) to give maximum responsiveness to the simulation
	static long long time_to_update = 0;
	if(time_to_update > time) {
		_dev->run();
		return;
	}
	time_to_update = time + 40;

	const core::dimension2du &screen = _drv->getScreenSize();

	_drv->beginScene(true, true, SColor(255, 20, 20, 20));
	
	_scene->drawAll();
	
	// debug
	_aircraft->render(); 
	renderRange(); 
	// render euler angles
	glm::quat q = sitl->get_rotation();
	glm::vec3 pos = _aircraft->getPosition();
	glm::vec3 up = q * glm::vec3(0, 10, 0);
	_drv->draw3DLine(vector3df(pos.x, pos.y, pos.z),
		vector3df(pos.x + up.x, pos.y + up.y, pos.z + up.z), 
		SColor( 255, 100, 255, 100 ));

	// draw sticks
	_drv->draw2DRectangle(SColor(100, 100, 100, 100), core::rect<s32>(10, screen.Height - 50, 50, screen.Height - 10));
	_drv->draw2DRectangle(SColor(100, 100, 100, 100), core::rect<s32>(60, screen.Height - 50, 100, screen.Height - 10));
	// cross lines
	_drv->draw2DLine(core::position2d<s32>(10, screen.Height - 30), core::position2d<s32>(50, screen.Height - 30), SColor(100, 255,255,255));
	_drv->draw2DLine(core::position2d<s32>(30, screen.Height - 50), core::position2d<s32>(30, screen.Height - 10), SColor(100, 255,255,255));
	_drv->draw2DLine(core::position2d<s32>(60, screen.Height - 30), core::position2d<s32>(100, screen.Height - 30), SColor(100, 255,255,255));
	_drv->draw2DLine(core::position2d<s32>(80, screen.Height - 50), core::position2d<s32>(80, screen.Height - 10), SColor(100, 255,255,255));
	
	// sticks themselves
	_drv->draw2DPolygon(core::position2d<s32>(10 + 40 * mRCYaw, screen.Height - 10 - 40 * mRCThrottle), 5, SColor(100, 100, 0, 0), 20);
	_drv->draw2DPolygon(core::position2d<s32>(60 + 40 * mRCRoll, screen.Height - 10 - 40 * mRCPitch), 5, SColor(100, 100, 0, 0), 20);

	// aux channels
	_drv->draw2DLine(core::position2d<s32>(110, screen.Height - 50), core::position2d<s32>(110, screen.Height - 10), SColor(100, 255,255,255));
	_drv->draw2DLine(core::position2d<s32>(125, screen.Height - 50), core::position2d<s32>(125, screen.Height - 10), SColor(100, 255,255,255));
	_drv->draw2DPolygon(core::position2d<s32>(110, screen.Height - 10 - 40 * mRCAux1), 5, SColor(110, 100, 0, 0), 20);
	_drv->draw2DPolygon(core::position2d<s32>(125, screen.Height - 10 - 40 * mRCAux2), 5, SColor(100, 100, 0, 0), 20);


	gui::IGUIFont* font = _dev->getGUIEnvironment()->getBuiltInFont();

	if(_key_down[KEY_KEY_W])
		font->draw(L"THROTTLE UP", 
			core::rect<s32>(10,10,300,50),
			video::SColor(255,255,255,255));
	if(_key_down[KEY_KEY_S])
		font->draw(L"THROTTLE DOWN", 
			core::rect<s32>(10,20,300,50),
			video::SColor(255,255,255,255));
	if(_key_down[KEY_KEY_A])
		font->draw(L"YAW LEFT", 
			core::rect<s32>(10,30,300,50),
			video::SColor(255,255,255,255));
	if(_key_down[KEY_KEY_D])
		font->draw(L"YAW RIGHT", 
			core::rect<s32>(10,40,300,50),
			video::SColor(255,255,255,255));
	if(_key_down[KEY_UP])
		font->draw(L"PITCH FORWARD", 
			core::rect<s32>(10,50,300,50),
			video::SColor(255,255,255,255));
	if(_key_down[KEY_DOWN])
		font->draw(L"PITCH BACKWARD", 
			core::rect<s32>(10,60,300,50),
			video::SColor(255,255,255,255));
	if(_key_down[KEY_LEFT])
		font->draw(L"ROLL LEFT", 
			core::rect<s32>(10,70,300,50),
			video::SColor(255,255,255,255));
	if(_key_down[KEY_RIGHT])
		font->draw(L"ROLL RIGHT", 
			core::rect<s32>(10,80,300,50),
			video::SColor(255,255,255,255));
	
	irrGUI->drawAll();

	/*
	_osd.draw();
	uint16_t x_step = screen.Width / 376;
	uint16_t y_step = screen.Height / 266;
	int tt = 0;
	for(unsigned x = 0; x < screen.Width; x += x_step){
		for(unsigned y = 0; y < screen.Height; y += y_step){
			if(tt % 4 == 3)
			_drv->drawPixel(x, y, SColor(0, 0, 0, 0));
			tt++;
		}
	}
*/
	// draw leds
	if(sitl->read_led(0))
		_drv->draw2DImage(_led_on, position2di(10, screen.Height - 100), rect<s32>(0, 0, 32, 32), 0, SColor(255, 255, 255, 255), true);
	else
		_drv->draw2DImage(_led_off, position2di(10, screen.Height - 100), rect<s32>(0, 0, 32, 32), 0, SColor(255, 255, 255, 255), true);

	// update beeper
	_snd->beep(sitl->read_beeper());

	_drv->endScene();
	_dev->run();
}


bool Application::OnEvent(const SEvent &ev) {
	if(ev.EventType == EET_JOYSTICK_INPUT_EVENT){
		_joystickState = ev.JoystickEvent; 
		return true; ; 
	}

	if(ev.EventType == EET_KEY_INPUT_EVENT){
		_key_down[ev.KeyInput.Key] = ev.KeyInput.PressedDown; 
	}
	
	if(ev.EventType == EET_KEY_INPUT_EVENT && !ev.KeyInput.PressedDown) {
		switch(ev.KeyInput.Key) {
			case KEY_ESCAPE:
				Done = true;
			break;
			case KEY_KEY_R: 
				_replay_mode = true; 
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
				_aircraft->setPosition(glm::vec3(0, 30, 0)); 
				//_aircraft->setRotation(rots[id++] * y); 
				//_aircraft->setAngularVelocity(glm::vec3(0, 0, 0)); 
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
				_aircraft->setAngularVelocity(glm::vec3(0, 0, 0)); 
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
/*
	Node->setMaterialFlag(video::EMF_LIGHTING, true);
	Node->setMaterialFlag(video::EMF_FOG_ENABLE, true);
	int matCount = Node->getMaterialCount();
	for(int i=0; i < matCount; i++)
		Node->getMaterial(i).AmbientColor.set(255,255,0,0);
*/
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
	static glm::vec3 angles(0, 0, 0); 

	//printf("servo(%d %d %d %d) rpy(%f %f %f)\n", pkt.servo[0], pkt.servo[1], pkt.servo[2], pkt.servo[3], roll, pitch, yaw); 
	for(unsigned c = 0; c < FC_SITL_PWM_CHANNELS; c++){
		_aircraft->setOutput(c, (sitl->read_pwm(c) - 1000) / 1000.0f); 
	}

	//glm::vec3 pos = _aircraft->getPosition(); 
	//glm::quat rot = _aircraft->getRotation(); 
	glm::vec3 accel = _aircraft->getAccel(); 
	//glm::ivec3 loc = _aircraft->getLocation(); 
	//glm::vec3 mag = _aircraft->getMagneticField();
	//glm::vec3 vel = _aircraft->getVelocity(); 
	glm::vec3 gyro = _aircraft->getGyro(); 
	//glm::vec3 euler = glm::eulerAngles(rot); 

	sitl->write_gyro(-gyro.z, gyro.x, -gyro.y);
	//sitl->write_accel(accel.z, accel.x, -accel.y);
	sitl->write_accel(accel.z, -accel.x, accel.y);

	//state.pos[0] = pos.z; state.pos[1] = pos.x; state.pos[2] = -pos.y; 
	//state.loc[0] = loc.z; state.loc[1] = loc.x; state.loc[2] = loc.y; 
	//state.mag[0] = mag.z; state.mag[1] = mag.x; state.mag[2] = -mag.y; 
	//state.vel[0] = vel.z; state.vel[1] = vel.x; state.vel[2] = -vel.y; 

	sitl->write_rc(0, 1000 + mRCRoll * 1000);
	sitl->write_rc(1, 1000 + mRCPitch * 1000);
	sitl->write_rc(2, 1000 + mRCYaw * 1000);
	sitl->write_rc(3, 1000 + mRCThrottle * 1000);
	sitl->write_rc(4, 1000 + mRCAux1 * 1000);
	sitl->write_rc(5, 1000 + mRCAux2 * 1000);

	#if 0
	if(pkt.time == 0 || (time(NULL) - pkt.time) > 1) {
		printf("Invalid packet timestamp!"); 
		return;  
	}
	#endif

#if 0
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
#endif
#if 0 
	printf("sending: acc(%f %f %f) gyr(%f %f %f) mag(%f %f %f) vel(%f %f %f) loc(%d %d %d) rc(%f %f %f %f)\n", 
		state.accel[0], state.accel[1], state.accel[2],
		state.gyro[0], state.gyro[1], state.gyro[2],
		state.mag[0], state.mag[1], state.mag[2],
		state.vel[0], state.vel[1], state.vel[2], 
		state.loc[0], state.loc[1], state.loc[2],
		state.rcin[0], state.rcin[1], state.rcin[2], state.rcin[3]); 
#endif
	//memcpy(_shmout, &state, sizeof(state)); 
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
