cmake_minimum_required(VERSION 2.8.3)
project(obj_rec_gui)

#include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)
#set(ROS_BUILD_TYPE Debug)

#rosbuild_init()
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs genmsg speech_rec_messages cv_bridge roslib robbie_architecture or_libs or_nodes nav_libs rviz ptu)

#set the default path for built executables to the "bin" directory
#set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
#set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(robotgui src/main.cpp)
#target_link_libraries(example ${PROJECT_NAME})

##############################################################################
# Required 3rd party
##############################################################################
# http://qtnode.net/wiki/Qt4_with_cmake

find_package(Qt4 COMPONENTS QtCore QtGui QtOpenGl QtNetwork REQUIRED)
find_package(OpenGL REQUIRED)
SET(QT_USE_QTXML TRUE)
#SET(QT_USE_QTOPENGL TRUE)
include(${QT_USE_FILE})



# This is necessary as all ui files etc will get dumped in the bottom of the
# binary directory.
include_directories(include ${CMAKE_CURRENT_BINARY_DIR} "/usr/include/qt4" "/usr/include/qt4/QtGui" "/usr/include/qt4/QtCore" ${catkin_INCLUDE_DIRS} ${PROJECT_SOURCE_DIR}
	$ENV{HOMER_DIR}/experimental/Libraries/BaseLib/src/
        $ENV{HOMER_DIR}
        src/Workers
		src
        ${OPENGL_INCLUDE_DIRS}
        ${catkin_INCLUDE_DIRS}
	)
##############################################################################
# Sections
##############################################################################

catkin_package(
#  INCLUDE_DIRS ${PROJECT_SOURCE_DIR}
  CATKIN_DEPENDS speech_rec_messages std_msgs roscpp or_nodes or_libs or_msgs
)

add_subdirectory(src)
file(GLOB QT_FORMS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} src/*.ui)
file(GLOB QT_RESOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} resources/resource.qrc)
file(GLOB_RECURSE QT_MOC RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS include/local/*.hpp  src/main.h src/MainWindow.h src/QtRosNode.h src/rosprocessbutton.h src/talker.h src/ButtonProcessAdministrator.h src/rosout_subscriber.h
)

QT4_ADD_RESOURCES(QT_RESOURCES_CPP ${QT_RESOURCES})
QT4_WRAP_UI(QT_FORMS_HPP ${QT_FORMS})
QT4_WRAP_CPP(QT_MOC_HPP ${QT_MOC})


##############################################################################
# Sources
##############################################################################

file(GLOB_RECURSE QT_SOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS src/gicp-fallback.cpp src/main.cpp src/MainWindow.cpp src/QtRosNode.cpp src/rosprocessbutton.cpp
 src/talker.cpp src/ButtonProcessAdministrator.cpp src/rosout_subscriber.cpp
src/Containers/MapTab.cpp
)

##############################################################################
# Binaries
##############################################################################
add_executable(obj_rec_gui ${QT_SOURCES} ${QT_RESOURCES_CPP} ${QT_FORMS_HPP} ${QT_MOC_HPP})
target_link_libraries(obj_rec_gui ${QT_LIBRARIES} ${catkin_LIBRARIES} ${ROS_LIBRARIES}
#--- GUI - Containers
  ObjectRecognitionTab_or
  ObjectLearningTab_or
  #--- GUI - Widgets ---
  GLImageWidget_or
  ImageMessagesDisplay_or
  ObjectRecognitionWidget_or
  SensorDataDisplay_or
  #--- Workers ---
  CoordinateConverterGUI_or
  ImageSources_or
  $ENV{HOMER_DIR}/catkin_ws/devel/lib/libKeyPointExtraction.so
  $ENV{HOMER_DIR}/catkin_ws/devel/lib/libObjectRecognition.so
  $ENV{HOMER_DIR}/catkin_ws/devel/lib/libParallelSurf.so
  Puma2_or
  $ENV{HOMER_DIR}/catkin_ws/devel/lib/libVectorGraphics.so
  $ENV{HOMER_DIR}/catkin_ws/devel/lib/libMath.so
  #--- Dependencies ---
  ${QT_LIBRARIES}
  ${catkin_LIBRARIES}
  GL
  GLU
  konihcl
  #opencv_imgproc
  ${OPENGL_LIBRARIES}
)


