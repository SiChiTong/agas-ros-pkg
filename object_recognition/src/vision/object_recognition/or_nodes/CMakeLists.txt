cmake_minimum_required(VERSION 2.8.3)

project(or_nodes)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

find_package(catkin REQUIRED COMPONENTS message_generation roscpp_serialization roscpp cv_bridge tf or_msgs or_libs roslib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})

file(MAKE_DIRECTORY images)
file(MAKE_DIRECTORY objectProperties)

# boost
set(Boost_USE_MULTITHREADED ON)
#signals regex filesystem thread date_time iostreams
find_package(Boost COMPONENTS system serialization REQUIRED)
set(USEPKG_INCLUDE_DIR ${Boost_INCLUDE_DIR} ${USEPKG_INCLUDE_DIR})
set(USEPKG_LIBRARIES ${Boost_LIBRARIES} ${USEPKG_LIBRARIES})


##############################################################################
# Required 3rd party
##############################################################################
find_package( OpenCV REQUIRED )

# This is necessary as all ui files etc will get dumped in the bottom of the
# binary directory.
include_directories(/usr/local/include ${CMAKE_CURRENT_BINARY_DIR} 
	${catkin_INCLUDE_DIRS} 
        src
        $ENV{HOMER_DIR}/catkin_ws/src/vision/object_recognition
        $ENV{HOMER_DIR}/catkin_ws/src/Libraries/robbie_architecture/src
        )
##############################################################################
# Sections
##############################################################################


##############################################################################
# Sources
##############################################################################

#file(GLOB_RECURSE RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS
#  src/main.cpp
#)
add_subdirectory(src/Modules)

##############################################################################
# Binaries
##############################################################################
add_executable(obj_rec src/obj_rec.cpp)
add_executable(obj_learn src/obj_learn.cpp)

target_link_libraries(obj_rec
  Modules
  ${Boost_LIBRARIES}
  ${catkin_LIBRARIES}
)

target_link_libraries(obj_learn
  Modules
  ${Boost_LIBRARIES}
  ${catkin_LIBRARIES}
)

catkin_package(
	INCLUDE_DIRS src
    CATKIN_DEPENDS message_runtime std_msgs sensor_msgs roscpp_serialization or_libs roslib or_msgs
	LIBRARIES Modules
)
