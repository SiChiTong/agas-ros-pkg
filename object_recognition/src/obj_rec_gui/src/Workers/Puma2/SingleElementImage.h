/**
 * @file    SingleElementImage.h
 * @brief   Basic template for image data storage.
 *
 * (C) 2007 AG Aktives Sehen <agas@uni-koblenz.de>, Universitaet Koblenz-Landau
 * $Id: $
 */

#ifndef SingleElementImage_H
#define SingleElementImage_H

#include "BaseImageTemplate.h"

namespace puma2 {

/**
 * @class   SingleElementImage
 * @brief   Single channel image template
 *
 * (detailed descripton forthcomming)
 *
 * @see     TBaseImg
 * @author  Dietrich Paulus <paulus@uni-koblenz.de>
 * @date    January 2007
 */

template <class T>
  class SingleElementImage : public TBaseImg<T> {
  public:
    typedef T ElementType;	//< element type - here alias for pixel
    typedef T PixelType;	//< pixel type -- same as ElementType for SingleElements
    static int numberOfChannels() { return 1; }
    SingleElementImage(int width=0, int height = 0) : TBaseImg<T>(width, height) {}
    /// subimage constructor
    SingleElementImage(int width, int height, SingleElementImage* master, int xOffset, int yOffset)
        : TBaseImg<T>(width, height, master, xOffset, yOffset) {}
    const ElementType sample(int x, int y, int n) const {
          assert( n == 0 );
          return TBaseImg<T>::c0[y][x];
        }
    ElementType& sample(int x, int y, int n) {
          assert( n == 0 );
          return TBaseImg<T>::c0[y][x];
        }
};

}

#endif
