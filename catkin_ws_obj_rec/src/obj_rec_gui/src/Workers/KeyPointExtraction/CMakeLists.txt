set(KeyPointExtraction_SRC
DefaultExtractor.cpp
KeyPoint.cpp
KeyPointHelper.cpp
SurfExtractorBase.cpp
ParallelSurfExtractor.cpp
#OrigSurfExtractor.cpp
)

add_library(KeyPointExtraction ${KeyPointExtraction_SRC})

target_link_libraries(KeyPointExtraction
  Puma2
  ParallelSurf
)


include_directories(
../ThreadPool
)
