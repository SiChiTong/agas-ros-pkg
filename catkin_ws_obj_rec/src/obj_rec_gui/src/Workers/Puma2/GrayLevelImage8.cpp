#include "GrayLevelImage8.h"
// #include "ImageReader.h" // TODO
// #include "ImageWriter.h" // TODO

using namespace puma2;

GrayLevelImage8::GrayLevelImage8(int width, int height)
	: SingleElementImage<byte>(width, height)
{
  setupImageBaseVariables();
  alt = new MultiElementImage<byte,1>(width, height,
  	(MultiElementImage<byte,1>*) (void*) (SingleElementImage<byte>*) this, 0,0);
};

GrayLevelImage8::GrayLevelImage8(int width, int height, GrayLevelImage8 * m, int xo, int yo)
	: SingleElementImage<byte>(width, height,m,xo,yo)
{
  setupImageBaseVariables();
  alt = new MultiElementImage<byte,1>(width, height,
  	(MultiElementImage<byte,1>*) (void*) (SingleElementImage<byte>*) this, xo,yo);
};

GrayLevelImage8::GrayLevelImage8(const GrayLevelImage8& f)
  : SingleElementImage<byte>(f)
{
  using namespace std;
  // cerr << "Copy Constructor Gray-Level Image" << endl;
  setupImageBaseVariables();
  // alt = new MultiElementImage<byte,1>(f.alt);
  alt = new MultiElementImage<byte,1>(f.getWidth(), f.getHeight(),
  	(MultiElementImage<byte,1>*) (void*) (SingleElementImage<byte>*) this, 0,0);
};

GrayLevelImage8::~GrayLevelImage8()
{
  using namespace std;
  // cerr << "Delete Gray-Level Image" << endl;
  delete alt;
};

MultiElementImage<byte,1> & GrayLevelImage8::asMultiElementImage()
{
  return *alt;
};

void GrayLevelImage8::readFromFile(const char * fileName)
{
  // ImageReader::readImage(*this, fileName); // TODO
}

void GrayLevelImage8::writeToFile(const char * fileName) const
{
  // ImageWriter::writeImage(*this, fileName); // TODO
}

void GrayLevelImage8::reset()
{
}



