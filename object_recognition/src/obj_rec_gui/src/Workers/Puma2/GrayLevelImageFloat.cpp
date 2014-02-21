#include "GrayLevelImageFloat.h"
// #include "ImageReader.h" // TODO
// #include "ImageWriter.h" // TODO

#include "GrayLevelImage8.h"

using namespace puma2;

GrayLevelImageFloat::GrayLevelImageFloat(int width, int height)
	: SingleElementImage<float>(width, height)
{
  setupImageBaseVariables();
  alt = new MultiElementImage<float,1>(width, height,
  	(MultiElementImage<float,1>*) (void*) (SingleElementImage<float>*) this, 0,0);
};

GrayLevelImageFloat::GrayLevelImageFloat(int width, int height, GrayLevelImageFloat * m, int xo, int yo)
	: SingleElementImage<float>(width, height,m,xo,yo)
{
  setupImageBaseVariables();
  alt = new MultiElementImage<float,1>(width, height,
  	(MultiElementImage<float,1>*) (void*) (SingleElementImage<float>*) this, xo,yo);
};

GrayLevelImageFloat::GrayLevelImageFloat(const GrayLevelImageFloat& f)
  : SingleElementImage<float>(f)
{
  using namespace std;
  // cerr << "Copy Constructor Gray-Level Image" << endl;
  setupImageBaseVariables();
  // alt = new MultiElementImage<float,1>(f.alt);
  alt = new MultiElementImage<float,1>(f.getWidth(), f.getHeight(),
  	(MultiElementImage<float,1>*) (void*) (SingleElementImage<float>*) this, 0,0);
};

GrayLevelImageFloat::~GrayLevelImageFloat()
{
  using namespace std;
  // cerr << "Delete Gray-Level Image" << endl;
  delete alt;
};

MultiElementImage<float,1> & GrayLevelImageFloat::asMultiElementImage()
{
  return *alt;
};

void GrayLevelImageFloat::readFromFile(const char * fileName)
{
  GrayLevelImage8 byteImg;
  // ImageReader::readImage( byteImg, fileName); // TODO

  resize( byteImg.getWidth(), byteImg.getHeight() );
  for ( unsigned y=0; y<getHeight(); y++ )
  {
    for ( unsigned x=0; x<getWidth(); x++ )
    {
      (*this)[y][x]= byteImg[y][x] * (1.0/255.0);
    }
  }
}

void GrayLevelImageFloat::writeToFile(const char * fileName) const
{
  GrayLevelImage8 byteImg;
  byteImg.resize( getWidth(), getHeight() );

  for ( unsigned y=0; y<getHeight(); y++ )
  {
    for ( unsigned x=0; x<getWidth(); x++ )
    {
      float value = (*this)[y][x] * 255.0;
      if ( value < 0 )
      {
        value = 0;
      }
      if ( value > 255 )
      {
        value = 255;
      }
      byteImg[y][x] = value;
    }
  }

  // ImageWriter::writeImage(byteImg, fileName); // TODO
}

void GrayLevelImageFloat::reset()
{
}



