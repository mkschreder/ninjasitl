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

#include "GenericXRotor.h"

class TiltXRotor : public GenericXRotor {
public: 
	TiltXRotor(Application *app); 

	virtual void setOutput(unsigned int id, float value) override ; 
	virtual void onCollision(const btCollisionObject *a, const btCollisionObject *b) override; 

	virtual void update(float dt) override; 
protected: 
	virtual btRigidBody *createFrameRigidBody() override; 
	virtual irr::scene::ISceneNode *createFrameSceneNode() override; 

private:
	irr::scene::IAnimatedMeshSceneNode *_frame_node; 
	irr::scene::IAnimatedMeshSceneNode *_front_arm, *_back_arm; 

	std::vector<btRigidBody*> _props; 

	//irr::scene::ISceneNode *_cube; 
}; 
