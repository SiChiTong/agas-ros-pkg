/*******************************************************************************
 *  Y8UV8ToRGB8Operator.cpp
 *
 *  (C) 2007 AG Aktives Sehen <agas@uni-koblenz.de>
 *           Universitaet Koblenz-Landau
 *
 *  Additional information:
 *  $Id: Y8UV8ToRGB8Operator.cpp 44313 2011-04-06 22:46:28Z agas $
 *******************************************************************************/

#include "Y8UV8ToRGB8Operator.h"
#include <iostream>

using namespace std;

namespace puma2 {


  bool Y8UV8ToRGB8Operator::initialized=false;

  int Y8UV8ToRGB8Operator::yPalette[256];
  int Y8UV8ToRGB8Operator::uPaletteG[256];
  int Y8UV8ToRGB8Operator::uPaletteB[256];
  int Y8UV8ToRGB8Operator::vPaletteR[256];
  int Y8UV8ToRGB8Operator::vPaletteG[256];
  Mutex Y8UV8ToRGB8Operator::mutex;

  void Y8UV8ToRGB8Operator::initialize()
  {
    /**
     * @note Conversion formulae:
     *
     * Formula for digital YCrCb (YUV) taken from the Sony X700 technical manual.
     * R = Y           + 1.4022V
     * G = Y - 0.3458U - 0.7144V
     * B = Y + 1.7710U
     *
     * Taken from "intersil YCbCr to RGB considerations"
     * R´ = 1.164(Y - 16)                   + 1.596(Cr - 128)
     * G´ = 1.164(Y - 16) - 0.392(Cb - 128) - 0.813(Cr - 128)
     * B´ = 1.164(Y - 16) + 2.017(Cb - 128)
    */

    mutex.lock();
      if (!initialized) {
        // TRACE_INFO("Creating look up tables."); // TODO use ros
        //create palettes
        for (int i=0;i<256;i++)
        {
          yPalette[i]  = i;
          vPaletteR[i] = int(  1.4022*float(i-128) );
          uPaletteG[i] = int( -0.3458*float(i-128) );
          vPaletteG[i] = int( -0.7144*float(i-128) );
          uPaletteB[i] = int(  1.7710*float(i-128) );
/*          yPalette[i]  = int(  1.164*float(i-16) );
          vPaletteR[i] = int(  1.569*float(i-128) );
          uPaletteG[i] = int( -0.392*float(i-128) );
          vPaletteG[i] = int( -0.813*float(i-128) );
          uPaletteB[i] = int(  2.017*float(i-128) );*/
        }
        initialized=true;
      }
    mutex.unlock();
  }


  Y8UV8ToRGB8Operator::Y8UV8ToRGB8Operator() { }

  Y8UV8ToRGB8Operator::~Y8UV8ToRGB8Operator() { }

  void Y8UV8ToRGB8Operator::apply(const GrayLevelImage8& constYImage, const ColorImageUV8& constUvImage, ColorImageRGB8& imageRGB)
  {

    initialize();

    unsigned width=imageRGB.getWidth();
    unsigned height=imageRGB.getHeight();

    GrayLevelImage8& imageY=const_cast<GrayLevelImage8&>(constYImage);
    ColorImageUV8& imageUV=const_cast<ColorImageUV8&>(constUvImage);

    unsigned char* yData = (unsigned char*) ( imageY.unsafeRowPointerArray()[0] );
    unsigned char* uvData = (unsigned char*) (imageUV[0][0]);
    ColorImageRGB8::PixelType* rgbData = imageRGB.unsafeRowPointerArray()[0];

    int r,g,b;
    int cY;
    unsigned u,v;

    for( unsigned y = 0; y < height ; y++ ) {
      for( unsigned x = 0; x < width ; x++ )
      {
        cY=yPalette[ *(yData++) ];
        u=*(uvData++);
        v=*(uvData++);

        r=cY + vPaletteR[ v ];
        if (r>255) r=255;
        if (r<0) r=0;

        g=cY + uPaletteG[ u ] + vPaletteG[ v ];
        if (g>255) g=255;
        if (g<0) g=0;

        b=cY + uPaletteB[ u ];
        if (b>255) b=255;
        if (b<0) b=0;

        (*rgbData)[0]=r;
        (*rgbData)[1]=g;
        (*rgbData)[2]=b;

        rgbData++;
      }

    }

  }

  }


/*

  //create palettes
  for (int i=0;i<256;i++)
  {
    uPaletteG[i] = int( -0.3458*float(i-128) );
    uPaletteB[i] = int(  1.7710*float(i-128) );
    vPaletteR[i] = int(  1.4022*float(i-128) );
    vPaletteG[i] = int( -0.7144*float(i-128) );
  }

  unsigned char* yData = (unsigned char*) ( imageY.unsafeRowPointerArray()[0] );
  char* uvData = (char*) (imageUV.unsafeRowPointerArray()[0]);
  ColorImageRGB8::PixelType* rgbData = imageRGB.unsafeRowPointerArray()[0];

  int r,g,b;
  unsigned cY,cU,cV;

  for( unsigned y = 0; y < height ; y++ ) {
    for( unsigned x = 0; x < width ; x++ )
    {
      cY=*yData;
      cU=unsigned(int( (* uvData   ) ));
      cV=unsigned(int( (*(uvData+1)) ));

      r=y                  + vPaletteR[ cV ];
      if (r>255) r=255;
      if (r<0) r=0;

      g=y + uPaletteG[ cU ] + vPaletteG[ cV ];
      if (g>255) g=255;
      if (g<0) g=0;

      b=y + uPaletteB[ cU ];
      if (b>255) b=255;
      if (b<0) b=0;

      (*rgbData)[0]=r;
      (*rgbData)[1]=g;
      (*rgbData)[2]=b;

      yData++;
      uvData+=2;
      rgbData++;
    }

  }

*/
