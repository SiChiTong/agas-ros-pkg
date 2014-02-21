#include <string>
#include "ColorImageRGB8.h"
//#include "ImageReader.h" // TODO
// #include "ImageWriter.h"// TODO
#include "MultiElementImage.h"

using namespace std;
using namespace puma2;

ColorImageRGB8::ColorImageRGB8(int x, int y)
	: MultiElementImage<byte,3>(x,y)
{
  setupImageBaseVariables();
}

ColorImageRGB8::ColorImageRGB8(int x, int y, ColorImageRGB8 * m, int xo, int yo)
	: MultiElementImage<byte,3>(x,y,m,xo,yo)
{
  setupImageBaseVariables();
}

void ColorImageRGB8::assign(int i, int j, int n, int v)
{
  c0[i][j][n] = v;
}

void ColorImageRGB8::assign(int x, int y, int r, int g, int b)
{
  c0[y][x][0] = r;
  c0[y][x][1] = g;
  c0[y][x][2] = b;
}

void ColorImageRGB8::operator= (const ColorImageRGB8 & o)
{
  MultiElementImage<byte,3>::operator=(o);
}

void ColorImageRGB8::readFromFile(const char * fileName)
{
  //ImageReader::readImage(*this, fileName); // TODO
}

void ColorImageRGB8::writeToFile(const char * fileName) const
{
  // ImageWriter::writeImage(*this, fileName); // TODO
}

void ColorImageRGB8::reset()
{
}

