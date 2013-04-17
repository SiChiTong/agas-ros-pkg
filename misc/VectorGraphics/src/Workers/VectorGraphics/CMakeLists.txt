set(VectorGraphics_SRC
  VectorObject2D.cpp
)

rosbuild_add_library(VectorGraphics ${VectorGraphics_SRC})

target_link_libraries(VectorGraphics
  Math
  GL
)
