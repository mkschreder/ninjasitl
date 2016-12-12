NinjaSITL
---------

NinjaSITL aims to be an integrated testing environment for testing flight
controllers (currently predominantly aimed at testing ninjaflight:
https://github.com/mkschreder/ninjaflight, although it was first developed for
ardupilot).

[![IMAGE ALT](https://img.youtube.com/vi/rGc4XmpufJ4/0.jpg)](https://www.youtube.com/watch?v=rGc4XmpufJ4)

Building
--------

You will need a few libraries to build the sitl. Most notably:
	
* glm math library
* openal for sound
* bullet physics 

You can install these on linux using:

	sudo apt-get install libbullet-dev libopenal-dev libglm-dev

It also requires a flight controller to simulate. Current version only supports
ninjaflight flight controller. The most straightforward way is therefore to
build the sitl simulator through ninjaflight. This can be done like this:

	git clone https://github.com/mkschreder/ninjaflight.git
	cd ninjaflight && git checkout devel
	make sitl-start

This will build ninjaflight as a shared library (suitable for sitl), then
checkout and build ninjasitl, then try to download pak0 from the internet (it
is a relatively large file ~500Mb) and then start the simulator.

Documentation
-------------

Code documentation is at https://mkschreder.github.io/ninjasitl

About
-----

The simulator provides realistic sensor inputs to the flight controller and
expects to be given outputs in the form of pwm signal. Other things such as
telemetry and osd support are on the plan list and they will be using already
available telemetry protocols to read telemetry data from flight controller in
order to be as close as possible to real world flying. The sitl is developed
primarily for testing various control strategies and algorithms for navigation,
stabilization and autonomous flight. Do not expect state of the art graphics
from it because it is not it's primary purpose. 

For rendering this project uses the good old irrlicht engine which is also
included in the source code. It is not the most modern choice but it is very
easy to work with and does the job for this simulator. 

For sound the sitl uses openal. You can install openal using "apt-get install
libopenal-dev".

NinjaSITL uses physics simulation implemented through the use of Bullet Physics
engine.

Interface
---------

The simulator expects a flight controller to be built as a shared library that
exports fc\_sitl\_create\_aircraft method that creates a new instance of a
simulated aircraft. Each aircraft has to run as a separate thread in parallel
with the simulation. This approach is not mandatory of course and other
approaches have been tried before. However running the flight controller as a
shared library imposes many good constraints on the flight control software that
are good for quality of the code. In particular, the sitl assumes that multiple
instances of the flight controller can be active in the same address space at
the same time. To satisfy this requirement, the flight controller software
needs to use good object oriented practices with instantiated data and not use
any global state at all (otherwise things will just blow up in a multithreaded
application). Constraints like this are generally good for pushing flight
controller development in a more sane direction. 

If you feel that interfacing with the flight controller over shared memory or a
socket is a better fit for you then go back in the commit tree and ressurect
one of these approaches for yourself. They have been used before. 

Compiling
---------

To compile you need: 
apt-get install libglm-dev libbullet-dev

License
-------

This SITL is distributed under GPLv3.
