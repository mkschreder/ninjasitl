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
#include "time.h"
#include <unistd.h>

static Application *_app;

// Event receiver
class EventReceiverClass : public IEventReceiver  {

public:
	virtual bool OnEvent(const SEvent &TEvent)  {
		_app->OnEvent(TEvent);
		return false; 
	}
};

int main(int argc, char **argv) {
	int c; 
	const char *map = "q3dmp23.bsp"; 
	const char *replay_file = NULL; 

	while((c = getopt(argc, argv, "m:r:")) != -1){
		switch(c){
			case 'm': 
				map = optarg; 
				break; 
			case 'r':	
				replay_file = optarg; 
				break; 
		}
	}

	_app = new Application();

	_app->loadMap(map); 

	if(replay_file){
		_app->loadReplay(replay_file); 
	}

	// arm 
	while(1) {
		_app->run(); 
	}

	return 0; 
}

