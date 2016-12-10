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

#include "OdometryReplay.h"
#include <iostream>
#include <fstream>
#include <vector>
#include <sstream>
#include <string.h>
#include <string>

using namespace std; 

static void split(const string &s, char delim, vector<string> &elems) {
    stringstream ss(s);
    string item;
    while (getline(ss, item, delim)) {
        elems.push_back(item);
    }
}

static vector<string> split(const string &s, char delim) {
    vector<string> elems;
    split(s, delim, elems);
    return elems;
}

OdometryReplay::OdometryReplay(){

}

int OdometryReplay::load(const char *filename){
	std::ifstream f(filename); 
	if(!f.is_open()) return -ENOENT; 

	string line; 
	// read field names
	if(!std::getline(f, line)) {
		fprintf(stderr, "Could not get first line from file!\n"); 
		f.close(); 
		return -1; 
	}

	vector<string> headers = split(line, ','); 
	const char *names[] = {
		"timeSeq",
		"No.",
		"position.x", 
		"position.y",
		"position.z",
		"orientation.x",
		"field",
		"orientation.y",
		"orientation.z",
		"orientation.w"
	}; 
	int field_count = sizeof(names) / sizeof(names[0]); 
	int c = 0; 
	for(vector<string>::iterator i = headers.begin(); i != headers.end(); i++){
		if(strcmp((*i).c_str(), names[c++]) != 0){
			fprintf(stderr, "File header does not match expected data!\n"); 
			return -1; 
		}
	}
	
	int line_num = 0; 
	// load the data points
	while(std::getline(f, line)){
		KeyFrame frame; 
		unsigned long long tmp; 
		if(sscanf(line.c_str(), "%llu,%d,%llu,%f,%f,%f,%f,%f,%f,%f", 
			&tmp, 
			&frame.id, 
			&frame.time, 
			&frame.pos.x, 
			&frame.pos.y, 
			&frame.pos.z,
			&frame.rot.x,
			&frame.rot.y, 
			&frame.rot.z, 
			&frame.rot.w
		) != field_count){
			fprintf(stderr, "Could not parse field on line %d\n", line_num); 	
		}
		_frames.push_back(frame); 
		line_num++; 
	}
	printf("loaded %d odometry keyframes\n", line_num); 
	_current_frame = _frames.begin();
	_time = (*_current_frame).time; 
	f.close(); 
	return 0; 
}

bool OdometryReplay::step(float dt){
	if(_current_frame == _frames.end()) return false; 	
	_time += (unsigned long long)(dt * 1e9); 
	KeyFrame &f = (*_current_frame); 
	std::vector<KeyFrame>::iterator next_frame = _current_frame + 1; 
	if(next_frame != _frames.end()){
		KeyFrame &n = (*next_frame); 
		if(_time > n.time){
			_current_frame++; 
		} 
		unsigned long long len = n.time - f.time; 
		unsigned long long completed = _time - f.time; 
		if(len > 0){
			float fraction = (float)completed / (float) len;  
			_current_pos = f.pos + glm::vec3(5.0f, 3.0f, 0) + (n.pos - f.pos) * fraction;  
			_current_rot = glm::slerp(f.rot, n.rot, fraction); 

			glm::vec3 e = glm::eulerAngles(_current_rot); 
			printf("frame len %llu %f, %f %f %f\n", len, fraction, glm::degrees(e.x), glm::degrees(e.y), glm::degrees(e.z)); 
		}
	}
	return true; 
}

glm::vec3 OdometryReplay::getPosition(){
	return _current_pos; 
}

glm::quat OdometryReplay::getRotation(){
	return _current_rot; 
}

