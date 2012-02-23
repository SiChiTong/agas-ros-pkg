cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

#set the default path for built executables to the "bin" directory
INSTALL(CODE "FILE(MAKE_DIRECTORY \${ENV}\${CMAKE_INSTALL_PREFIX}${mydir})")
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
#target_link_libraries(example ${PROJECT_NAME})

#pkg_check_modules(OGRE OGRE)

# add your source files here
set(RobotFace_SRC
  src/main.cpp
  src/MainWindow.cpp
  src/TalkingHead.cpp
  src/TextOutDisplay.cpp
  src/QtRosNode.cpp
  src/FestivalGenerator.cpp
)

# add headers of widgets that declare Q_OBJECT here
set(RobotFace_INC
  include/MainWindow.h
  include/TalkingHead.h
  include/TextOutDisplay.h
  include/QtRosNode.h
  include/FestivalGenerator.h
)

#rosbuild_find_ros_package(ogre)
#message( "Ogre found at ${OGRE_INCLUDE_DIRS}" )

#set(CMAKE_MODULE_PATH "${ogre_PACKAGE_PATH}/ogre/lib/OGRE/cmake/;${CMAKE_MODULE_PATH}")
set(CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake;${CMAKE_MODULE_PATH};/usr/share/OGRE/cmake/modules/;/opt/ros/electric/stacks/perception_pcl/pcl/share/pcl-1.1/;")
#find_package(OGRE REQUIRED)
include_directories( SYSTEM ${OGRE_INCLUDE_DIRS} )
link_directories( ${OGRE_LIBRARY_DIRS} )

#configure_file(${PROJECT_SOURCE_DIR}/plugins.cfg.in ${PROJECT_SOURCE_DIR}/plugins.cfg)


# QT related
find_package(Qt4 REQUIRED)

set(QT_USE_QTXML 1)
include(${QT_USE_FILE})

QT4_WRAP_CPP(RobotFace_MOC_SRC ${RobotFace_INC})

#add_definitions(${QT_DEFINITIONS})
include_directories( SYSTEM ${QT_INCLUDE_DIR})

rosbuild_add_library(RobotFace ${RobotFace_SRC} ${RobotFace_INC} include/FestivalSynthesizer.h include/Config.h include/Data.h)

rosbuild_add_executable(RobotFace ${RobotFace_SRC} ${RobotFace_MOC_SRC} )
rosbuild_add_executable(FestivalSynthesizer src/FestivalSynthesizer.cpp )

target_link_libraries(RobotFace ${QT_LIBRARIES})

find_package(Festival REQUIRED)
include_directories( SYSTEM ${Festival_INCLUDE_DIR} )
target_link_libraries(RobotFace ${Festival_LIBRARIES} )
target_link_libraries(FestivalSynthesizer ${Festival_LIBRARIES} )

find_package(EST REQUIRED)
include_directories( SYSTEM ${EST_INCLUDE_DIR} )
target_link_libraries(RobotFace ${EST_LIBRARIES} )
target_link_libraries(FestivalSynthesizer ${EST_LIBRARIES} )

#target_link_libraries(RobotFace ${USEPKG_LIBRARIES} pulse-simple)
target_link_libraries(FestivalSynthesizer ${USEPKG_LIBRARIES} pulse-simple)

#
# check code stlye
#

if(EXISTS "/usr/bin/python2")
    set(PYTHON_VER "python2")
else()
    set(PYTHON_VER "python")
endif()

# for all style ignore options, execute:
# python2 scripts/cpplint.py --filter=

#set(STYLE_IGNORE "-whitespace/parens,-whitespace/parens,-whitespace/line_length,-whitespace/braces,-whitespace/blank_line,-whitespace/newline,-whitespace/comments,-whitespace/operators,-whitespace/comma,-whitespace/tab,-whitespace/end_of_line,-readability/function,-readability/streams,-readability/todo,-build/include,-legal/copyright")

#set(STYLE_IGNORE "-whitespace,-readability,-legal,-runtime")
set(STYLE_IGNORE "-whitespace/parens, -whitespace/braces, -build, -legal, -readability, -runtime, -build/header_guard")
set(STYLE_SOURCES
    ${CMAKE_SOURCE_DIR}/src
    ${CMAKE_SOURCE_DIR}/includes
)

foreach(STYLE_SOURCE ${STYLE_SOURCES})
    file(GLOB_RECURSE GLOB_CPP ${STYLE_SOURCE}/*.cpp)
    file(GLOB_RECURSE GLOB_H ${STYLE_SOURCE}/*.h)
    set(GLOB_STYLE ${GLOB_STYLE} ${GLOB_CPP} ${GLOB_H})
endforeach()

add_custom_target(style
    COMMAND ${PYTHON_VER} cpplint.py --filter=${STYLE_IGNORE} --counting=detailed ${GLOB_STYLE}
)

