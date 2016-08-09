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

#include "TiltXRotor.h"

TiltXRotor::TiltXRotor(Application *app) : GenericXRotor(app) {

}

void TiltXRotor::setOutput(unsigned int id, float value){
	if(id == 7){
		float pitch = (value - 0.5f) * 45.0f; 
		// tilt all rotors forward or backwards depending on supplied pitch
		for(std::vector<Motor*>::iterator i = _motors.begin(); 
			i != _motors.end(); ++i){
			(*i)->setRotation(glm::quat(cos(glm::radians(pitch / 2.0f)), -sin(glm::radians(pitch / 2.0f)), 0, 0)); 
		}
	} else {
		GenericXRotor::setOutput(id, value); 
	}
}
