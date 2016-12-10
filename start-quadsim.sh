#!/bin/sh

set -e

killall quadcopter || echo "No process found!"

if [ ! -d build ]; then 
	mkdir build; 
fi

# download pak0
if [ ! -f "base/pak0.pk3" ]; then
	echo "Downloading pak0..."
	wget http://files.anitalink.com/gamecache/quake3/baseq3/pak0.pk3 -O base/pak0.pk3 || {
		echo "Please download pak0.pk3 file from somewhere. It is part of quake3. It contains standard quake textures that you will need to view maps."
	}
fi

cd build
cmake ../src
cd ..

make -C build -j8 && ./build/quadcopter #-m Q3AnthraxDM1.bsp
