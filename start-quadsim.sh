#!/bin/sh

if [ ! -d build ]; then 
	mkdir build; 
fi

cd build
cmake ..
make -j8 
cd ..

killall quadcopter
./build/quadcopter
