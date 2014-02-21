#find_package(Unicap REQUIRED)
#find_package(Glib2 REQUIRED)

# add your source files here
set(ObjectRecognitionWidget_SRC
  ObjectLearningControl.cpp
  ObjectRecognitionDisplay.cpp
  ObjectList.cpp
  ObjectImagesControl.cpp
)

# add headers of widgets that declare Q_OBJECT here
set(ObjectRecognitionWidget_MOC_H
  ObjectRecognitionDisplay.h
  ObjectList.h
  ObjectImagesControl.h
  ObjectLearningControl.h
)
qt4_wrap_cpp(ObjectRecognitionWidget_MOC_SRC ${ObjectRecognitionWidget_MOC_H})

include_directories(
  ${Unicap_INCLUDE_DIR}
  ${Glib2_INCLUDE_DIR}
)

add_library(ObjectRecognitionWidget_or
  ${ObjectRecognitionWidget_SRC}
  ${ObjectRecognitionWidget_MOC_SRC}
)
target_link_libraries(ObjectRecognitionWidget_or GLImageWidget_or
$ENV{HOMER_DIR}/catkin_ws/devel/lib/libVectorGraphics.so
$ENV{HOMER_DIR}/catkin_ws/devel/lib/libKeyPointExtraction.so
Puma2_or
ImageMessagesDisplay_or)
