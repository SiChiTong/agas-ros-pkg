/*******************************************************************************
 *  RGB8ToY8UV8Operator.h
 *
 *  (C) 2006 AG Aktives Sehen <agas@uni-koblenz.de>
 *           Universitaet Koblenz-Landau
 *
 *  Additional information:
 *  $Id: RGB8ToY8UV8Operator.h 25610 2008-06-20 08:30:53Z dgossow $
 *******************************************************************************/

#ifndef RGB8ToY8UV8Operator_H
#define RGB8ToY8UV8Operator_H

#include "ImageToImagePairOperator.h"
#include "ColorImageUV8.h"
#include "../../Workers/Puma2/GrayLevelImage8.h"
#include "../../Workers/Puma2/ColorImageRGB8.h"
 #include "../ImageSources/Mutex.h" // TODO

namespace puma2 {

/**
 * @class RGB8ToY8UV8Operator
 * @brief Converts an RGB image to separate Y and UV images
 */
class RGB8ToY8UV8Operator :
    public ImageToImagePairOperator<ColorImageRGB8,GrayLevelImage8,ColorImageUV8>
{
  public:

    /**
     * Default constructor.
     */
    RGB8ToY8UV8Operator() {};

    /**
     * Create, apply, delete
     */
    RGB8ToY8UV8Operator( const ColorImageRGB8& constRgbImage, GrayLevelImage8& imageY, ColorImageUV8& imageUV )
    {
      checkArgument( constRgbImage, imageY, imageUV );
      apply( constRgbImage, imageY, imageUV );
    }

    /**
     * Destructor
     */
    virtual ~RGB8ToY8UV8Operator() {};

    /// Do the real work of the operator
  	virtual void apply( const ColorImageRGB8& constRgbImage, GrayLevelImage8& imageY, ColorImageUV8& imageUV );

  private:
      
};

}

#endif
