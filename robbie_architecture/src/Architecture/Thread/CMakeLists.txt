set(Thread_SRC
  Mutex.cpp
  Thread.cpp
  Condition.cpp
)

rosbuild_add_library(Thread ${Thread_SRC})
