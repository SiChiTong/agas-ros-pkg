cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

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
include_directories(/usr/local/include ${CMAKE_CURRENT_BINARY_DIR})
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
rosbuild_add_executable(obj_rec src/obj_rec.cpp)
rosbuild_add_executable(obj_learn src/obj_learn.cpp)

target_link_libraries(obj_rec
  Modules
  ${Boost_LIBRARIES}
)

target_link_libraries(obj_learn
  Modules
  ${Boost_LIBRARIES}
)
