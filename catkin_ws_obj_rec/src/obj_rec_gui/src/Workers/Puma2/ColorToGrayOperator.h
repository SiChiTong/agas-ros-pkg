#ifndef Col2Gvi_H
#define Col2Gvi_H

#include "ImageToImageOperator.h"
#include "ColorImageRGB8.h"
#include "GrayLevelImage8.h"

namespace puma2 {

/**
 * @class DPOperator
 * @brief Demonstration of image operator for an 8-bit gray level image.
 */
template <class C, class G>
class ColorToGrayOperator  : 
	public ImageToImageOperator<C,G>
{
  public:

    /**
     * Default constructor.
     */
    ColorToGrayOperator();

    /**
     * Create, apply, delete
     */
	ColorToGrayOperator(const C & iImg, G & oImg)
	{
	      // NB: we can call this method only here, after
		  // complete construction of the base class
		  // i.e., we may not call this function in a base class
		  // constructor as virtual function tables are not 
		  // set up properly there!
	      this->operator()(iImg,oImg); // will call virtual functions
	}

    /**
     * Destructor
     */
    virtual ~ColorToGrayOperator();

    /// Do the real work of the operator
	virtual void apply(const C & iImg, G & oImg);
};

template <class C, class G>
ColorToGrayOperator<C,G>::ColorToGrayOperator() 
{
}

template <class C, class G>
ColorToGrayOperator<C,G>::~ColorToGrayOperator() 
{ 
  
};

template <class C, class G>
void ColorToGrayOperator<C,G>::apply(const C & iImg, G & oImg)
{
	for (int i = iImg.getHeight() - 1 ; i >= 0; --i) {
	  const typename C::PixelType *rowPtr = iImg[i];
	  for (int j = iImg.getWidth() - 1 ; j >= 0; --j) {
	  	oImg[i][j] = 
		   ((3 * int(rowPtr[j][0]) +
	  	     6 * int(rowPtr[j][1]) +
		         int(rowPtr[j][2])) / 10);
	  }
	}
}

}

#endif
