/*******************************************************************************
 *  ColorImageYUV8.cpp
 *
 *  (C) 2006 AG Aktives Sehen <agas@uni-koblenz.de>
 *           Universitaet Koblenz-Landau
 *
 *  Information on Code Review state:
 *  Author: SB; DevelTest: ; Reviewer: ; Review: ; State: NOK
 *
 *  Additional information:  
 *  $Id: ColorImageYUV8.h 44313 2011-04-06 22:46:28Z agas $ 
 *******************************************************************************/

#ifndef ColorImageYUV8_H
#define ColorImageYUV8_H

#include "../../Workers/Puma2/ColorImageRGB8.h"

namespace puma2 {

  /**
  * @class ColorImageYUV8
  * @brief Wrapper around the ColorImageRGB8 class to represent YUV Images
  */
  class ColorImageYUV8 : public ColorImageRGB8 
  {
    
    public:
  
      /**
      * Default constructor.
      * @param x,y width and height of the image
      */
      ColorImageYUV8(int x = 0, int y = 0): ColorImageRGB8(x, y) {};
  
      /**
      * Undocumented
      * TODO: ask for documentation
      */
      ColorImageYUV8(int x, int y, ColorImageYUV8 * m, int xo, int yo): ColorImageRGB8(x, y, m, xo, yo) {};
  
  };

}
#endif
