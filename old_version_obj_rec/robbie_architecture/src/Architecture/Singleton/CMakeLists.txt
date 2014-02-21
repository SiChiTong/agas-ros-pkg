set(Singleton_SRC
  Clock.cpp
  SequenceNumber.cpp
)

rosbuild_add_library(Singleton ${Singleton_SRC})

target_link_libraries(Singleton Thread)
