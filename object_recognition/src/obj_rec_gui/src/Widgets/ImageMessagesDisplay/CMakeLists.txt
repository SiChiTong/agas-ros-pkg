#find_package(Unicap REQUIRED)
#find_package(Glib2 REQUIRED)

# add your source files here
set(ImageMessagesDisplay_SRC
  ImageMessagesDisplay.cpp
  ImageSourceSelector.cpp
)

# add headers of widgets that declare Q_OBJECT here
set(ImageMessagesDisplay_MOC_H
  ImageMessagesDisplay.h
  ImageSourceSelector.h
)
qt4_wrap_cpp(ImageMessagesDisplay_MOC_SRC ${ImageMessagesDisplay_MOC_H})

include_directories(
  ${Unicap_INCLUDE_DIR}
  ${Glib2_INCLUDE_DIR}
)

add_library(ImageMessagesDisplay_or
  ${ImageMessagesDisplay_SRC}
  ${ImageMessagesDisplay_MOC_SRC}
)
