#ifndef ImageToImageOperator8_H
#define ImageToImageOperator8_H

#include "ImageOperator8.h"

namespace puma2 {

/**
 * @class ImageOperator
 * @brief Image operator for an 8-bit gray level image.
 */

class ImageToImageOperator8 : public ImageOperator8
{
public:

    /**
     * Default constructor.
     */
    ImageToImageOperator8();

    /**
     * Destructor
     */
    virtual ~ImageToImageOperator8();


	/// interface that will call apply -- can be overwritten in derived class
	virtual void operator() (const GrayLevelImage8 & iImg, GrayLevelImage8 & oImg);
protected:
    /// will be pure virtual later
	virtual void apply(const GrayLevelImage8 & iImg, GrayLevelImage8 & oImg);

    /** 
	  * test and adjust size of output image
	  *
	  * Default: sizes must be equal for input and output
	  *
	  * if output image has zero size, it will be allocated to have
	  * the same size as the input image.
	  *
	  * other behaviour can be defined for derived operators which only
	  * have to redefine this method
	  */
	virtual void checkSize(const GrayLevelImage8 & iImg, GrayLevelImage8 & oImg);
};

}

#endif
