set(VectorGraphics_SRC
  VectorObject2D.cpp
)

find_package(OpenGL REQUIRED)

include_directories( ${OPENGL_INCLUDE_DIRS} )

add_library(VectorGraphics ${VectorGraphics_SRC})

target_link_libraries(VectorGraphics
  Math
  ${OPENGL_LIBRARIES} 
)
