#ifndef ImageToImageOperator_H
#define ImageToImageOperator_H

#include <iostream>
#include "ImageOperator.h"

namespace puma2 {

/**
 * @class ImageOperator
 * @brief Image operator for any pair of image classes (in, out)
 */

template <class I, class O > class ImageToImageOperator 
	: public ImageOperator
{
public:

    /**
     * Default constructor.
     */
    ImageToImageOperator();

    /**
     * Destructor
     */
    virtual ~ImageToImageOperator();

	/// interface that will call apply -- can be overwritten in derived class
	virtual void operator() (const I & iImg, O & oImg);

protected:
    /// must be defined in derived class
	virtual void apply(const I & iImg, O & oImg) = 0;

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
	virtual void checkArgument(const I & iImg, O & oImg);
public:
	static  void checkImageArgument(const I & iImg, O & oImg);
};


template <class I, class O> 
   ImageToImageOperator<I,O>::ImageToImageOperator() 
{
}

template <class I, class O> 
   ImageToImageOperator<I,O>::~ImageToImageOperator() 
{ 
};

template <class I, class O> 
   void ImageToImageOperator<I,O>::operator() 
	(const I & iImg, O & oImg)
{
	checkArgument(iImg,oImg);   // virtual - can be overwritten
	apply(iImg,oImg);       // virtual - must be overwritten
}


template <class I, class O> 
   void ImageToImageOperator<I,O>::checkArgument
	(const I & iImg, O & oImg)
{
	checkImageArgument(iImg,oImg);
}

template <class I, class O> 
   void ImageToImageOperator<I,O>::checkImageArgument
	(const I & iImg, O & oImg)
{
	if (oImg.getWidth() != 0) {
	  if ((oImg.getWidth() != iImg.getWidth() )  ||
	      (oImg.getHeight() != iImg.getHeight() ))
		  		throw "Image size missmatch";
	} else {
	    if (reinterpret_cast<const void*>(&iImg) == reinterpret_cast<const void*>(&oImg)) 
				throw "Inplace operation of image operator not possible";
		oImg.resize(iImg.getWidth(),iImg.getHeight());
	}
}

} // Namespace

#endif
