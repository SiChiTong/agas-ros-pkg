/*******************************************************************************
 *  ThermalToColorOperator.cpp
 *
 *  (C) 2007 AG Aktives Sehen <agas@uni-koblenz.de>
 *           Universitaet Koblenz-Landau
 *
 *  Additional information:
 *  $Id: ThermalToColorOperator.cpp 44313 2011-04-06 22:46:28Z agas $ 
 *******************************************************************************/

#include "ThermalToColorOperator.h"
#include <iostream>
using namespace std;

namespace puma2 {

ThermalToColorOperator::ThermalToColorOperator(const GrayLevelImage8& constThermalImage, ColorImageRGB8& imageRGB, int thermalMin, int thermalMax)
{
  m_ThermalMin=thermalMin;
  m_ThermalMax=thermalMax;
  checkArgument(constThermalImage, imageRGB);
  apply(constThermalImage, imageRGB);
}
    
void ThermalToColorOperator::apply(const GrayLevelImage8& constThermalImage, ColorImageRGB8& imageRGB)
{
  
  GrayLevelImage8& thermalImage=const_cast<GrayLevelImage8&>(constThermalImage);

  int width=imageRGB.getWidth();
  int height=imageRGB.getHeight();

  puma2::byte** thermalRow = thermalImage.unsafeRowPointerArray();
  ColorImageRGB8::PixelType** rgbRow = imageRGB.unsafeRowPointerArray();
  
  //ptr. to current row in orig. image
  puma2::byte* currentThermalRow;
  ColorImageRGB8::PixelType* currentRgbRow;
  
  unsigned char rPalette[256];
  unsigned char gPalette[256];
  unsigned char bPalette[256];

  //create palette (blue-green-red)
  int t, t2;
  
  for (int i=0; i<256; i++) {
    t=(i-m_ThermalMin)*255/(m_ThermalMax-m_ThermalMin);
    if (t < 0) t=0;
    if (t > 255) t=255;
    if (t<128) {
      t2=t*2;
      rPalette[i]=0;
      gPalette[i]=t2;
      bPalette[i]=255-t2;
    } else {
      t2=(t-128)*2;
      rPalette[i]=t2;
      gPalette[i]=255-t2;
      bPalette[i]=0;
    }
  }
  
  //convert image
  unsigned int currentThermalSample;
  int x,y;
  for( y = 0; y < height ; y++ ) {
    currentThermalRow=thermalRow[y];
    currentRgbRow=rgbRow[y];
    for( x = 0; x < width ; x++ ) {
      currentThermalSample=currentThermalRow[x];
      currentRgbRow[x][0]=rPalette[ currentThermalSample ];
      currentRgbRow[x][1]=gPalette[ currentThermalSample ];
      currentRgbRow[x][2]=bPalette[ currentThermalSample ];
    }
  }
  
}

}

