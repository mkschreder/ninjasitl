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

#include <cstddef>
#include <stdio.h>
#include <string.h>
#include "Sound.h"

static void list_audio_devices(const ALCchar *devices)
{
        const ALCchar *device = devices, *next = devices + 1;
        size_t len = 0;

        fprintf(stdout, "Devices list:\n");
        fprintf(stdout, "----------\n");
        while (device && *device != '\0' && next && *next != '\0') {
                fprintf(stdout, "%s\n", device);
                len = strlen(device);
                device += (len + 1);
                next += (len + 2);
        }
        fprintf(stdout, "----------\n");
}


SoundSystem::SoundSystem(){
	list_audio_devices(alcGetString(NULL, ALC_DEVICE_SPECIFIER));

	dev = alcOpenDevice(alcGetString(NULL, ALC_DEVICE_SPECIFIER));
    ctx = alcCreateContext(dev, NULL);
    alcMakeContextCurrent(ctx);

	_generate_beep_buffer();
}

SoundSystem::~SoundSystem(){
	alcMakeContextCurrent(NULL);
    alcDestroyContext(ctx);
    alcCloseDevice(dev);
	ctx = 0;
	dev = 0;
}

void SoundSystem::_generate_beep_buffer(){
	// generate a very simple beep buffer.
    alGenBuffers(1, &beep_sound);

	float freq = 2500.f;
    int seconds = 4;
    unsigned sample_rate = 22050;
    size_t buf_size = seconds * sample_rate;

    short *samples = new short[buf_size];
	bool high = false;
	unsigned short s_high = sample_rate / freq;
    for(size_t i=0; i<buf_size; ++i) {
		if(i % s_high == (unsigned short)(s_high - 1)) high = !high;
		samples[i] = (high)?32760:-32760;
        //samples[i] = 32760 * sin( (2.f*float(M_PI)*freq)/sample_rate * i );
    }

    alBufferData(beep_sound, AL_FORMAT_MONO16, samples, buf_size, sample_rate);
	alGenSources(1, &beep_src);
    alSourcei(beep_src, AL_BUFFER, beep_sound);

	// beep should be looped
	alSourcei(beep_src, AL_LOOPING, 1);

	// delete the buffer since it should now have been copied onto the sound card
	delete samples;
}

void SoundSystem::beep(bool on){
	if(!dev) return;
	ALint sound_state;
	alGetSourcei(beep_src, AL_SOURCE_STATE, &sound_state);

	if(on && sound_state != AL_PLAYING){
		alSourcePlay(beep_src);
	} else if(!on && sound_state == AL_PLAYING){
		alSourceStop(beep_src);
	}
}
