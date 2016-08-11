#!/bin/sh

killall quadcopter

if [ ! -d build ]; then 
	mkdir build; 
fi

cd build
cmake ..
cd ..

make -C build -j8 && ./build/quadcopter
