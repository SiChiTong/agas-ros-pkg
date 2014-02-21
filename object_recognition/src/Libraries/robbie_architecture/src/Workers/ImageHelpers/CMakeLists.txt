set(ImageHelpers_SRC
#ImageMask.cpp
ImageMaskCV.cpp # conversion to OpenCV
)

add_library(ImageHelpers ${ImageHelpers_SRC})
target_link_libraries(ImageHelpers Math)
