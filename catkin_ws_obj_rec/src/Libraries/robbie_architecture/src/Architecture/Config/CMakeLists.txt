set(Config_SRC
  Config.cpp
)

add_library(Config ${Config_SRC})

target_link_libraries(Config TinyXML Thread)

