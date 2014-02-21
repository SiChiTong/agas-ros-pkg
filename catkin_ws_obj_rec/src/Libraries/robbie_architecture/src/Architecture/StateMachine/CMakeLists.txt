set(StateMachine_SRC
  StateMachine.cpp
)

add_library(StateMachine ${StateMachine_SRC})

target_link_libraries(StateMachine Singleton)

