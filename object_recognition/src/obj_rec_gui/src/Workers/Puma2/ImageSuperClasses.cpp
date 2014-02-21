#include <string>
#include <typeinfo>
#include <iostream>
#include "ImageSuperClasses.h"
#include "PumaException.h"

#include <stdio.h>

using namespace puma2;

void Image::readFromFile(const char * fileName)
{
  char buffer [1024];
  sprintf (buffer,"Try to read from %s -- not implemeted for current class", fileName);
  throw buffer;
}

void Image::writeToFile(const char * fileName) const
{
  char buffer [1024];
  sprintf (buffer,"Try to write to %s -- not implemeted for current class", fileName);
  throw buffer;
}

Image::Image ()
{
}

Image::~Image() {}

void Image::setupImageBaseVariables ()
{
  // std::cout << std::endl << "Image::setupImageBaseVariables() called for class >" << typeid(*this).name() << "<" << std::endl;
  mValueRangeMinimum = this->getElementTypeMinimum();
  mValueRangeMaximum = this->getElementTypeMaximum();
  // std::cout << "Image::setupImageBaseVariables() max is " << mValueRangeMaximum << std::endl;
}




