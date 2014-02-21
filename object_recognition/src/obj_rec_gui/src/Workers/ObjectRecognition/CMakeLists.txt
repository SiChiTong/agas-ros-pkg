set(ObjectRecognition_SRC
CvFundamentalMat.cpp
CvHomography.cpp
FLANNMatcher.cpp
HoughAccumulator.cpp
HoughClusterer.cpp
HoughIndexCalculator.cpp
ImageProperties.cpp
MatchHelper.cpp
NNRMatcher.cpp
ObjectProperties.cpp
SimpleHoughClusterer.cpp
)

add_library(ObjectRecognition ${ObjectRecognition_SRC})
