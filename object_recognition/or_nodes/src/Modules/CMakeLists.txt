#find_package(Unicap REQUIRED)
#find_package(GeoTiff REQUIRED)
#find_package(Glib2 REQUIRED)
#find_package(PocketSphinx REQUIRED)
#find_package(KNI REQUIRED)
#find_package(Gstreamer REQUIRED)
#find_package(XmlTwo REQUIRED)
#find_package(PCL REQUIRED)
#find_package(OpenCV REQUIRED)

# QT related
#find_package(Qt4 COMPONENTS QtCore QtGui REQUIRED)
#include(${QT_USE_FILE})
#add_definitions(${QT_DEFINITIONS})

set(Modules_SRC
#ORAutoLearningModule.cpp
ORControlModule.cpp
ORLearningModule.cpp
ORLoaderModule.cpp
ORMatchingModule.cpp
)

#include_directories(SYSTEM
#  ${Unicap_INCLUDE_DIR}
#  ${Glib2_INCLUDE_DIR}
#  ${PocketSphinx_INCLUDE_DIR}
#  ${QT_INCLUDE_DIR}
#  ${KNI_INCLUDE_DIR}
#  ${PCL_INCLUDE_DIR}
#  ${GeoTiff_INCLUDE_DIR}
#  ${Gstreamer_INCLUDE_DIR}
#  ${XmlTwo_INCLUDE_DIR}
#)

rosbuild_add_library(Modules ${Modules_SRC})

target_link_libraries(Modules
	ObjectRecognition
	Singleton
    ImageHelpers
	${OpenCV_LIBRARIES}
	flann
)

