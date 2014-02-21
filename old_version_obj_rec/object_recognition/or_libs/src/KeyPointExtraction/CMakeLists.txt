set(KeyPointExtraction_SRC
DefaultExtractor.cpp
KeyPoint.cpp
KeyPointHelper.cpp
SurfExtractorBase.cpp
ParallelSurfExtractor.cpp
#OrigSurfExtractor.cpp
)

rosbuild_add_library(KeyPointExtraction ${KeyPointExtraction_SRC})


include_directories(
../ThreadPool
)


target_link_libraries(KeyPointExtraction
  ParallelSurf 
  ImageHelpers
  ${OpenCV_LIBS}
)
