#find_package(Unicap REQUIRED)
#find_package(Glib2 REQUIRED)

# add your source files here
set(ObjectRecognitionTab_SRC
  ObjectRecognitionTab.cpp
)

# add headers of widgets that declare Q_OBJECT here
set(ObjectRecognitionTab_MOC_H
  ObjectRecognitionTab.h
)
qt4_wrap_cpp(ObjectRecognitionTab_MOC_SRC ${ObjectRecognitionTab_MOC_H})

include_directories(
  ${Unicap_INCLUDE_DIR}
  ${Glib2_INCLUDE_DIR}
)

add_library(ObjectRecognitionTab_or
  ${ObjectRecognitionTab_SRC}
  ${ObjectRecognitionTab_MOC_SRC}
)
