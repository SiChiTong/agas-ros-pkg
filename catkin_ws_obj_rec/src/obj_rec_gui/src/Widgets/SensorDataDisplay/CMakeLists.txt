#find_package(Unicap REQUIRED)
#find_package(Glib2 REQUIRED)

# add your source files here
set(SensorDataDisplay_SRC
#  ParticleGridPainter.cpp
#  POIPainter.cpp
#  RobotPosesPainter.cpp
#  TofPainter.cpp
#  SonarPainter.cpp
  PainterCheckBox.cpp
  SensorDataDisplay.cpp
#  ImagePainter.cpp
  PainterPlugin.cpp
#  PersonPainter.cpp
#  SkeletonPainter.cpp
#  RobotPainter.cpp
#  SLAM3DPainter.cpp
#  SceneGraphPainter.cpp
#  LaserScan3DPainter.cpp
#  PathPainter.cpp
  NewLaserDataPainter.cpp
#  RGBDepthPainter.cpp
  SensorDataGLWidget.cpp
#  MapPainter.cpp
#  ParticlePainter.cpp
#  PersonParticlePainter.cpp
#  SilhouettePainter.cpp
)

# add headers of widgets that declare Q_OBJECT here
set(SensorDataDisplay_MOC_H
#  PersonPainter.h
#  SkeletonPainter.h
#  PersonParticlePainter.h
  NewLaserDataPainter.h
#  SonarPainter.h
#  SLAM3DPainter.h
#  SceneGraphPainter.h
#  ParticlePainter.h
#  LaserScan3DPainter.h
  PainterCheckBox.h
#  POIPainter.h
#  ImagePainter.h
#  RobotPainter.h
#  ParticleGridPainter.h
#  RGBDepthPainter.h
  SensorDataGLWidget.h
#  PathPainter.h
  SensorDataDisplay.h
#  RobotPosesPainter.h
#  MapPainter.h
  PainterPlugin.h
#  TofPainter.h
#  SilhouettePainter.h
)
qt4_wrap_cpp(SensorDataDisplay_MOC_SRC ${SensorDataDisplay_MOC_H})

include_directories(
  ${Unicap_INCLUDE_DIR}
  ${Glib2_INCLUDE_DIR}
)

add_library(SensorDataDisplay_or
  ${SensorDataDisplay_SRC}
  ${SensorDataDisplay_MOC_SRC}
)
