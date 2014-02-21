set(KeyPointExtraction_SRC
DefaultExtractor.cpp
KeyPoint.cpp
KeyPointHelper.cpp
SurfExtractorBase.cpp
ParallelSurfExtractor.cpp
#OrigSurfExtractor.cpp
ImageMaskCV.cpp
)

add_library(KeyPointExtraction ${KeyPointExtraction_SRC})


include_directories(
../ThreadPool
)


target_link_libraries(KeyPointExtraction
  ParallelSurf
  #ImageHelpers
        $ENV{HOMER_DIR}/catkin_ws/devel/lib/libConfig.so
        $ENV{HOMER_DIR}/catkin_ws/devel/lib/libVectorGraphics.so
  ${OpenCV_LIBS}
  ${catkin_LIBRARIES}
)
