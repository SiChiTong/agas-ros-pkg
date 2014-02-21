/*******************************************************************************
 *  Y8UV8ToRGB8Operator.h
 *
 *  (C) 2006 AG Aktives Sehen <agas@uni-koblenz.de>
 *           Universitaet Koblenz-Landau
 *
 *  Additional information:
 *  $Id: Y8UV8ToRGB8Operator.h 44313 2011-04-06 22:46:28Z agas $ 
 *******************************************************************************/

#ifndef Y8UV8ToRGB8Operator_H
#define Y8UV8ToRGB8Operator_H

#include "ImagePairToImageOperator.h"
#include "ColorImageUV8.h"
#include "../../Workers/Puma2/GrayLevelImage8.h"
#include "../../Workers/Puma2/ColorImageRGB8.h"
#include "../../Workers/ImageSources/Mutex.h"

namespace puma2 {

/**
 * @class Y8UV8ToRGB8Operator
 * @brief Converts a GrayLevelImage (the Y channel) and a ColorImageUV8 to RGB
 */
class Y8UV8ToRGB8Operator :
	public ImagePairToImageOperator<GrayLevelImage8,ColorImageUV8,ColorImageRGB8>
{
  public:

    /**
     * Default constructor.
     */
    Y8UV8ToRGB8Operator();

    /**
     * Create, apply, delete
     */
    Y8UV8ToRGB8Operator(const GrayLevelImage8& constYImage, const ColorImageUV8& constUvImage, ColorImageRGB8& imageRGB)
    {
      checkArgument(constYImage, constUvImage, imageRGB);
      apply(constYImage, constUvImage, imageRGB);
    }

    /**
     * Destructor
     */
    virtual ~Y8UV8ToRGB8Operator();

    /// Do the real work of the operator
  	virtual void apply(const GrayLevelImage8& imageY, const ColorImageUV8& imageUV, ColorImageRGB8& imageRGB);

  private:
    
    static void initialize();

    static int yPalette[256];
    static int uPaletteG[256];
    static int uPaletteB[256];
    static int vPaletteR[256];
    static int vPaletteG[256];
    static bool initialized;
    static Mutex mutex;
      
};

}

#endif
