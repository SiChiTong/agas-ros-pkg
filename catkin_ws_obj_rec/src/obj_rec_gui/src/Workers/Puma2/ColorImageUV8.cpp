/*******************************************************************************
 *  ColorImageUV8.cpp
 *
 *  (C) 2006 AG Aktives Sehen <agas@uni-koblenz.de>
 *           Universitaet Koblenz-Landau
 *
 *  Additional information:
 *  $Id: ColorImageUV8.cpp 44313 2011-04-06 22:46:28Z agas $
 *******************************************************************************/

#include "ColorImageUV8.h"

#include <string>
// #include "ImageReader.h" // TODO
// #include "ImageWriter.h" // TODO
#include "MultiElementImage.h"

using namespace std;
using namespace puma2;

ColorImageUV8::ColorImageUV8(int x, int y)
	: MultiElementImage<byte,2>(x,y)
{
  setupImageBaseVariables();
}

ColorImageUV8::ColorImageUV8(int x, int y, ColorImageUV8 * m, int xo, int yo)
	: MultiElementImage<byte,2>(x,y,m,xo,yo)
{
  setupImageBaseVariables();
}

void ColorImageUV8::assign(int x, int y, int u, int v)
{
  c0[y][x][0] = u;
  c0[y][x][1] = v;
}

/*
void ColorImageUV8::operator= (const ColorImageUV8 & o)
{
  ColorImage::operator=(o);
  MultiElementImage<byte,2>::operator=(o);
}
*/

void ColorImageUV8::reset()
{
}
