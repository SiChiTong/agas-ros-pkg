/*******************************************************************************
 *  RGB8ToY8UV8Operator.cpp
 *
 *  (C) 2007 AG Aktives Sehen <agas@uni-koblenz.de>
 *           Universitaet Koblenz-Landau
 *
 *  Additional information:
 *  $Id: RGB8ToY8UV8Operator.cpp 25610 2008-06-20 08:30:53Z dgossow $
 *******************************************************************************/

#include "RGB8ToY8UV8Operator.h"
#include <iostream>


using namespace std;

namespace puma2
{

  void RGB8ToY8UV8Operator::apply ( const ColorImageRGB8& constRgbImage, GrayLevelImage8& imageY, ColorImageUV8& imageUV )
  {
    ColorImageRGB8& imageRGB=const_cast<ColorImageRGB8&> ( constRgbImage );

    unsigned width=imageRGB.getWidth();
    unsigned height=imageRGB.getHeight();

    unsigned char* yData = ( unsigned char* ) ( imageY.unsafeRowPointerArray() [0] );
    unsigned char* uvData = ( unsigned char* ) ( imageUV.unsafeRowPointerArray() [0][0] );
    unsigned char* rgbData = imageRGB.unsafeRowPointerArray() [0][0];

    float cY,cU,cV;
    float cR,cG,cB;

    for ( unsigned y = 0; y < height ; y++ )
    {
      for ( unsigned x = 0; x < width ; x++ )
      {
        cR = rgbData[0];
        cG = rgbData[1];
        cB = rgbData[2];

        cY = 0.2989*cR+ 0.5866*cG + 0.1145*cB;
        if ( cY > 255 ) { cY=255; }
        if ( cY < 0 ) { cY=0; }

        cU = -0.1688*cR - 0.3312*cG + 0.5000*cB +128;
        if ( cU > 255 ) { cU=255; }
        if ( cU < 0 ) { cU=0; }

        cV = 0.5000*cR - 0.4183*cG - 0.0817*cB +128;
        if ( cV > 255 ) { cV=255; }
        if ( cV < 0 ) { cV=0; }

        *yData=cY;
        uvData[0]=cU;
        uvData[1]=cV;

        rgbData+=3;
        uvData+=2;
        yData++;
      }

    }

  }

}

