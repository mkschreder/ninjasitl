cmake_minimum_required(VERSION 2.8)

#find_package( OpenCV REQUIRED )
#${OpenCV_LIBS} 
set(bullet_LIBS "BulletSoftBody" "BulletDynamics" "BulletCollision" "LinearMath")

add_subdirectory("irrlicht/source/Irrlicht")
# Sources
##file(GLOB Project_SOURCES main.cpp Serial.cpp)
file(GLOB Project_SOURCES *.cpp *.c )
file(GLOB Project_HEADER *.h)

include_directories("/usr/include/bullet" "irrlicht/include" "../martink" "../martink/include")

link_directories("../irrlicht/source/Irrlicht/")

set (CMAKE_CXX_FLAGS "-Wall -std=c++11 -pipe -fexceptions -fno-rtti -fstrict-aliasing -g -DNDEBUG -DGLM_FORCE_RADIANS -I/usr/local/include/bullet/ -I..")

# Executable
#add_library(quadcopter ${Project_SOURCES} ${Project_HEADERS})
add_executable(quadcopter ${Project_SOURCES} ${Project_HEADERS})

target_link_libraries(quadcopter "dl" "irrlicht" "m" "GL" "Xxf86vm" "Xext" "X11" "pthread" "jpeg" "bz2" ${bullet_LIBS} "openal")
