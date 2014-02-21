/*******************************************************************************
 *  ImagePairToImageOperator.h
 *
 *  (C) 2007 AG Aktives Sehen <agas@uni-koblenz.de>
 *           Universitaet Koblenz-Landau
 *
 *  Additional information:
 *  $Id: ImagePairToImageOperator.h 44313 2011-04-06 22:46:28Z agas $
 *******************************************************************************/

#ifndef ImagePairToImageOperator_H
#define ImagePairToImageOperator_H

#include <iostream>
#include "ImageOperator.h"

namespace puma2 {

/**
 * @class ImageOperator
 * @brief Image operator for any triple of image classes ((in1,in2), out)
 * @author David Gossow (RX)
 */

template <class I1, class I2, class O > class ImagePairToImageOperator
	: public ImageOperator
{
public:

    /**
     * Default constructor.
     */
    ImagePairToImageOperator();

    /**
     * Destructor
     */
    virtual ~ImagePairToImageOperator();

	/// interface that will call apply -- can be overwritten in derived class
	virtual void operator() (const I1 & iImg1, const I2 & iImg2, O & oImg);

protected:
    /// must be defined in derived class
	virtual void apply(const I1 & iImg1, const I2 & iImg2, O & oImg) = 0;

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
	  *
	  * The method will throw an exception if input and output image are the
	  * same. This can be overwritten in a derived class,
	  * if inplace operation is possible.
	  */
	virtual void checkArgument(const I1 & iImg1, const I2 & iImg2, O & oImg);
public:
	static  void checkImageArgument(const I1 & iImg1, const I2 & iImg2, O & oImg);
};


template <class I1, class I2, class O>
   ImagePairToImageOperator<I1,I2,O>::ImagePairToImageOperator()
{
}

template <class I1, class I2, class O>
   ImagePairToImageOperator<I1,I2,O>::~ImagePairToImageOperator()
{
};

template <class I1, class I2, class O>
   void ImagePairToImageOperator<I1,I2,O>::operator()
	(const I1 & iImg1, const I2 & iImg2, O & oImg)
{
	checkArgument(iImg1, iImg2, oImg);   // virtual - can be overwritten
	apply(iImg1, iImg2, oImg);       // virtual - must be overwritten
}


template <class I1, class I2, class O>
   void ImagePairToImageOperator<I1,I2,O>::checkArgument
	(const I1 & iImg1, const I2 & iImg2, O & oImg)
{
	checkImageArgument(iImg1, iImg2, oImg);
}

template <class I1, class I2, class O>
   void ImagePairToImageOperator<I1,I2,O>::checkImageArgument
	(const I1 & iImg1, const I2 & iImg2, O & oImg)
{
	if (oImg.getWidth() != 0) {
	  if (    (oImg.getWidth() != iImg1.getWidth() )  || (oImg.getHeight() != iImg1.getHeight() ) ||
            (oImg.getWidth() != iImg2.getWidth() )  || (oImg.getHeight() != iImg2.getHeight() ) )
    {
		  		throw "Image size missmatch";
    }
	} else {
    if ( (reinterpret_cast<const void*>(&iImg1) == reinterpret_cast<const void*>(&oImg)) ||
         (reinterpret_cast<const void*>(&iImg2) == reinterpret_cast<const void*>(&oImg)) )
    {
				throw "Inplace operation of image operator not possible";
    }
		oImg.resize(iImg1.getWidth(),iImg1.getHeight());
	}
}

} // Namespace

#endif
