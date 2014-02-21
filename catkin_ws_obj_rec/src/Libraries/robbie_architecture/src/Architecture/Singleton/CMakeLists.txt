set(Singleton_SRC
  Clock.cpp
  SequenceNumber.cpp
)

add_library(Singleton ${Singleton_SRC})

target_link_libraries(Singleton Thread)


