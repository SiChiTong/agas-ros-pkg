cmake_minimum_required(VERSION 2.8.3)
project(or_libs)
find_package(catkin REQUIRED COMPONENTS message_generation roscpp_serialization roscpp cv_bridge tf)


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

##############################################################################
# Required 3rd party
##############################################################################
include_directories(
	${catkin_INCLUDE_DIRS}
	src
        $ENV{HOMER_DIR}/catkin_ws/src/Libraries/robbie_architecture/src
)

add_subdirectory(src/ConnectedComponentAnalyzer)
add_subdirectory(src/KeyPointExtraction)
add_subdirectory(src/ObjectRecognition)
add_subdirectory(src/ParallelSurf)


catkin_package(
    INCLUDE_DIRS src
    CATKIN_DEPENDS message_runtime std_msgs sensor_msgs roscpp_serialization 
        LIBRARIES ConnectedComponentAnalyzer KeyPointExtraction ObjectRecognition ParallelSurf
)


