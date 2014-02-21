set(ObjectRecognition_SRC
CvFundamentalMat.cpp
CvHomography.cpp
FLANNMatcher.cpp
HoughAccumulator.cpp
HoughClusterer.cpp
HoughIndexCalculator.cpp
#ImageProperties.cpp
ImagePropertiesCV.cpp
MatchHelper.cpp
NNRMatcher.cpp
ObjectProperties.cpp
SimpleHoughClusterer.cpp
)

add_library(ObjectRecognition ${ObjectRecognition_SRC})

target_link_libraries(ObjectRecognition
  $ENV{HOMER_DIR}/catkin_ws/devel/lib/libMath.so # TODO find better solution for this
  $ENV{HOMER_DIR}/catkin_ws/devel/lib/libConfig.so
  KeyPointExtraction
  ${OpenCV_LIBS}
  ${catkin_LIBRARIES}
)
