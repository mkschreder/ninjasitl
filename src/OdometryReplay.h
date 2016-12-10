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

#include <glm/glm.hpp>
#include <glm/gtx/quaternion.hpp>
#include <vector>

class OdometryReplay {
public:
	OdometryReplay(); 
	int load(const char *file); 
	bool step(float dt); 
	glm::vec3 getPosition(); 
	glm::quat getRotation(); 
private: 
	class KeyFrame {
	public: 
		unsigned int id;
		unsigned long long time; 
		glm::vec3 pos;
		glm::quat rot; 
	}; 
	unsigned long long _time; 
	std::vector<KeyFrame> _frames; 
	std::vector<KeyFrame>::iterator _current_frame; 
	glm::vec3 _current_pos; 
	glm::quat _current_rot; 
}; 
