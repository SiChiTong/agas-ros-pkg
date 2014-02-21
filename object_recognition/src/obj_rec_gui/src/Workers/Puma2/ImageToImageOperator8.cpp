#include "ImageToImageOperator8.h"

using namespace puma2;

ImageToImageOperator8::ImageToImageOperator8() 
{
}

ImageToImageOperator8::~ImageToImageOperator8() 
{ 
};

void ImageToImageOperator8::apply(const GrayLevelImage8 & iImg, GrayLevelImage8 & oImg)
{
	throw "should be implemented in derived function";
}

void ImageToImageOperator8::operator() 
	(const GrayLevelImage8 & iImg, GrayLevelImage8 & oImg)
{
	checkSize(iImg,oImg);   // virtual - can be overwritten
	apply(iImg,oImg);
}

void ImageToImageOperator8::checkSize
	(const GrayLevelImage8 & iImg, GrayLevelImage8 & oImg)
{
	if (oImg.getWidth() != 0) {
	  if ((oImg.getWidth() != iImg.getWidth() )  ||
	      (oImg.getHeight() != iImg.getHeight() ))
		  		throw "Image size missmatch";
	} else {
		oImg.resize(iImg.getWidth(),iImg.getHeight());
	}
}

