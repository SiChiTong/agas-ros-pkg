set(TinyXML_SRC
  tinystr.cpp
  tinyxml.cpp
  tinyxmlerror.cpp
  tinyxmlparser.cpp
)

rosbuild_add_library(TinyXML ${TinyXML_SRC})

