#ifndef GrayLevelImage16_H
#define GrayLevelImage16_H

#include "SingleElementImage.h"
#include "ImageSuperClasses.h"

namespace puma2 {

/**
 * @class GrayLevelImage16
 * @brief Imageclass for an 16-bit gray level image.
 */
class GrayLevelImage16 :
  public SingleElementImage<unsigned short>   //< unsigned short == 16 bit? --> ask configure!
{
  public:

    /**
     * Default constructor.
     */
    GrayLevelImage16(int x = 0, int y = 0);

    GrayLevelImage16(int x, int y, GrayLevelImage16 * m, int xo, int yo);


    /** see Image::getElementTypeMinimum() */
    double getElementTypeMinimum() const { return 0; };

    /** see Image::getElementTypeMaximum() */
    double getElementTypeMaximum() const { return 255*255; };
};

}

#endif
