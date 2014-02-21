#include "ColorImageRGBa8.h"
//#include "ImageReader.h" // TODO
// #include "ImageWriter.h" // TODO

using namespace puma2;

ColorImageRGBa8::ColorImageRGBa8(int x, int y) : SingleElementImage<RGBa8>(x,y) ,alt(x,y, (MultiElementImage<byte,4>*) (void*) (SingleElementImage<RGBa8>*) this, 0,0){
  setupImageBaseVariables();
};

ColorImageRGBa8::ColorImageRGBa8(int x, int y, ColorImageRGBa8 * m, int xo, int yo) : SingleElementImage<RGBa8>(x,y,m,xo,yo),
                                                                                      alt(x,y,reinterpret_cast<MultiElementImage<byte,4>*>(this),0,0){
  setupImageBaseVariables();
};

void ColorImageRGBa8::assign(int i, int j, int n, int v){
  c0[i][j].assign(v,n);  // Correct
  //alt[i][j][n] = v;        // Test
};


MultiElementImage<byte,4>& ColorImageRGBa8::asMultiElementImage() {
  return alt;
};

int ColorImageRGBa8::numberOfChannels() {
  return 4;
};

void ColorImageRGBa8::readFromFile(const char * fileName)
{
  //ImageReader::readImage(*this, fileName); // TODO
}

void ColorImageRGBa8::writeToFile(const char * fileName) const
{
  // ImageWriter::writeImage(*this, fileName); // TODO
}
