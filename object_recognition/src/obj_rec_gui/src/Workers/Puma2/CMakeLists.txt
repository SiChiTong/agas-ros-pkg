set(Explorer_SRC
ColorImageRGB8.cpp
ColorImageRGBa8.cpp
ColorImageUV8.cpp
GrayLevelImage16.cpp
GrayLevelImage8.cpp
GrayLevelImageFloat.cpp
HistogramUV.cpp
#ImageMaskCV.cpp
ImageMask.cpp
ImageOperator8.cpp
ImageOperator.cpp
#ImageReader.cpp
ImageSuperClasses.cpp
ImageToImageOperator8.cpp
#ImageWriter.cpp
openCvAdaptor.cpp
PumaException.cpp
RGB8ToY8UV8Operator.cpp
ThermalToColorOperator.cpp
Y8UV8ToRGB8Operator.cpp
#jmemsrc.c
)

add_library(Puma2_or ${Explorer_SRC})
target_link_libraries(Puma2_or $ENV{HOMER_DIR}/catkin_ws/devel/lib/libMath.so ImageSources_or konihcl ${Jpeg_LIBRARIES})
