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
#include "GenericXRotor.h"

GenericXRotor::GenericXRotor(Application *app):
	Multirotor(app) {

}

irr::scene::ISceneNode *GenericXRotor::createFrameSceneNode(){
	::printf("GenericX: setup scene node\n"); 
	irr::scene::IAnimatedMeshSceneNode *n = (irr::scene::IAnimatedMeshSceneNode*)Multirotor::createFrameSceneNode(); 

	n->setMesh(_application->getSceneManager()->getMesh("frame_quad_x.obj")); 

	return n; 
}

void GenericXRotor::setupMotors(){
	::printf("GenericX: setup motors\n"); 
	_motors.push_back(new Motor(glm::vec3( 0.15, 0.0, 0.1), Motor::MOTOR_CCW));
	_motors.push_back(new Motor(glm::vec3(-0.15, 0.0,-0.1), Motor::MOTOR_CCW));
	_motors.push_back(new Motor(glm::vec3(-0.15, 0.0, 0.1), Motor::MOTOR_CW));
	_motors.push_back(new Motor(glm::vec3( 0.15, 0.0,-0.1), Motor::MOTOR_CW));
}
