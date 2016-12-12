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

// openal headers
// Note: previously openal used to be installed into OpenAL directory which now seems to be AL.
// you may still encounter the old setup. In that case change the path.
#include <AL/al.h>
#include <AL/alc.h>
#include <alsa/asoundlib.h>

class SoundSystem {
public:
	SoundSystem();
	~SoundSystem();

	void start();

	void beep(bool on);
private:
	void _generate_beep_buffer();
protected:
	ALCdevice *dev;
    ALCcontext *ctx;

    ALuint beep_sound;
	ALuint beep_src;
};

