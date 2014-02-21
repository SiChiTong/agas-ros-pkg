#find_package(Unicap REQUIRED)
#find_package(Glib2 REQUIRED)

# add your source files here
set(GLImageWidget_SRC
  GLImageWidget.cpp
  GLImagePainter.cpp
)

# add headers of widgets that declare Q_OBJECT here
set(GLImageWidget_MOC_H
  GLImageWidget.h
)
qt4_wrap_cpp(GLImageWidget_MOC_SRC ${GLImageWidget_MOC_H})

include_directories(
  ${Unicap_INCLUDE_DIR}
  ${Glib2_INCLUDE_DIR}
)

add_library(GLImageWidget_or
  ${GLImageWidget_SRC}
  ${GLImageWidget_MOC_SRC}
)
