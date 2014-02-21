#find_package(Unicap REQUIRED)
#find_package(Glib2 REQUIRED)

# add your source files here
set(ObjectLearningTab_SRC
  ObjectLearningTab.cpp
)

# add headers of widgets that declare Q_OBJECT here
set(ObjectLearningTab_MOC_H
  ObjectLearningTab.h
)
qt4_wrap_cpp(ObjectLearningTab_MOC_SRC ${ObjectLearningTab_MOC_H})

include_directories(
  ${Unicap_INCLUDE_DIR}
  ${Glib2_INCLUDE_DIR}
)

add_library(ObjectLearningTab_or
  ${ObjectLearningTab_SRC}
  ${ObjectLearningTab_MOC_SRC}
)
