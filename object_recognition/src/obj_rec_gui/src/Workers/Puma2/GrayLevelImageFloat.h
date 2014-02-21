#ifndef GrayLevelImageFloat_H
#define GrayLevelImageFloat_H

#include "SingleElementImage.h"
#include "ImageSuperClasses.h"
#include "MultiElementImage.h"

namespace puma2 {

/**
 * @class GrayLevelImageFloat
 * @brief Imageclass for an 8-bit gray level image.
 */
class GrayLevelImageFloat :
    public SingleElementImage<float>
{
  public:

    /**
     * Default constructor.
     */
    GrayLevelImageFloat(int xwidth = 0, int height = 0);

    GrayLevelImageFloat(const GrayLevelImageFloat &);

    /**
     * Subimage constructor:
     *   @param width   horizontal size
     *   @param height  vertical size
     *   @param m       Pointer to master image of which the image becomes a subimage
     *   @param xo      offset for horizontal position of subimage in master image
     *   @param yo      offset for vertical position of subimage in master image
     *
     * The constructor will do all required checks on the sizes of the images.
     */
    GrayLevelImageFloat(int width, int height, GrayLevelImageFloat * m, int xo, int yo);

    /**
     * Destructor
     */
    ~GrayLevelImageFloat();

    /** see Image::getElementTypeMinimum() */
    double getElementTypeMinimum() const { return 0; };

    /** see Image::getElementTypeMaximum() */
    double getElementTypeMaximum() const { return 1; };

    /** see Image::readFromFile() */
    void readFromFile(const char * fileName);
    /** see Image::writeToFile() */
    void writeToFile(const char * fileName) const;

    /**
     * Alternative access method to the image data as MultiElementImage.
     */
    MultiElementImage<float,1> & asMultiElementImage();
        // we could initialize alt only, when we need it
        // { if (alt == NULL) alt = new ...; return *alt; }
        // but we would have to store xo, yo, -- another
        // DESIGN OPTION

    /** automatic conversion
      *
      * Advantage: a GrayLevelImageFloat can be passed to a function
      * such as Test3<float,1> directly
      *
      * Disadvantage: may be confusing as for the overloaded
      * function
      *  void test4(SingleElementImage<float>& g);    // automatic choice
      *  void test4(MultiElementImage<float,1>& g); // needs asMultiElementImage()
      *
      */
    operator MultiElementImage<float,1>& () { return asMultiElementImage(); }

    void operator= (const GrayLevelImageFloat & g) {
       SingleElementImage<float>::operator=(g);
	}

  protected:
    virtual void reset(); /// reset internal IPL structures after resize
  private:
    MultiElementImage<float,1> * alt; /// alternative
};

}

#endif
