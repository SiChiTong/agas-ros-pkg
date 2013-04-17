set(Config_SRC
  Config.cpp
)

rosbuild_add_library(Config ${Config_SRC})

target_link_libraries(Config TinyXML Thread)

