set(ParallelSurf_SRC
Image.cpp
KeyPointDescriptor.cpp
KeyPointDetector.cpp
MathStuff.cpp
)

include_directories(
../ThreadPool
)

set(CMAKE_CXX_FLAGS
"${CMAKE_CXX_FLAGS_RELEASE} -O3"
#"${CMAKE_CXX_FLAGS_DEBUG}"
)

add_library(ParallelSurf ${ParallelSurf_SRC})

