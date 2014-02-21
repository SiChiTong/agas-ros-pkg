#find_package(Unicap REQUIRED)
#find_package(Glib2 REQUIRED)

set(ImageGrabber_SRC
#  GrabberUtils.cpp
#  Camera.cpp
  ImageGrabber.cpp
#  GrabbingDevice.cpp
#  GrabbingDeviceManager.cpp
)

include_directories(
  ${Unicap_INCLUDE_DIR}
  ${Glib2_INCLUDE_DIR}
)

add_library(ImageGrabber_or ${ImageGrabber_SRC})
