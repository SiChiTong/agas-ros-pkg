set(StateMachine_SRC
  StateMachine.cpp
)

rosbuild_add_library(StateMachine ${StateMachine_SRC})

target_link_libraries(StateMachine Singleton)

