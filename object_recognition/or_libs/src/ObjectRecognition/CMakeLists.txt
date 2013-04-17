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

rosbuild_add_library(ObjectRecognition ${ObjectRecognition_SRC})

target_link_libraries(ObjectRecognition
  Config
  Math
  KeyPointExtraction
  ${OpenCV_LIBS}
)
