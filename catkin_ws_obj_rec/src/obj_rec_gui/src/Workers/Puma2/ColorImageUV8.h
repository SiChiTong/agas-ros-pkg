/*******************************************************************************
 *  ColorImageUV8.h
 *
 *  (C) 2006 AG Aktives Sehen <agas@uni-koblenz.de>
 *           Universitaet Koblenz-Landau
 *
 *  Additional information:
 *  $Id: ColorImageUV8.h 44313 2011-04-06 22:46:28Z agas $
 *******************************************************************************/

#ifndef ColorImageUV8_H
#define ColorImageUV8_H

#include <limits.h>

#include "ImageSuperClasses.h"
#include "MultiElementImage.h"

namespace puma2 {

typedef unsigned char byte;

/**
 * @class ColorImageUV8
 * @brief Represents the U and V channels of a YUV image
 */
class ColorImageUV8 :
  public MultiElementImage<byte,2>
{
  public:

    /**
     * Default constructor.
     */
    ColorImageUV8(int x = 0, int y = 0);

    ColorImageUV8(int x, int y, ColorImageUV8 * m, int xo, int yo);

    /**
     * Set all values
     * @param[in] x Row
     * @param[in] y Line
     * @param[in] u U-value
     * @param[in] v V-value
     */
    void assign(int x, int y, int u, int v);

    /*
void operator= (const ColorImageUV8 & o);
*/

    /** see Image::getElementTypeMinimum() */
    double getElementTypeMinimum() const { return 0.0; };

    /** see Image::getElementTypeMaximum() */
    double getElementTypeMaximum() const { return UCHAR_MAX; };

protected:
    void reset();
};

}
#endif
