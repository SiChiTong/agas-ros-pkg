#ifndef GrayLevelImage8_H
#define GrayLevelImage8_H

#include "SingleElementImage.h"
#include "ImageSuperClasses.h"
#include "MultiElementImage.h"

namespace puma2 {

typedef unsigned char byte;

/**
 * @class GrayLevelImage8
 * @brief Imageclass for an 8-bit gray level image.
 */
class GrayLevelImage8 :
    public SingleElementImage<byte>
{
  public:

    /**
     * Default constructor.
     */
    GrayLevelImage8(int xwidth = 0, int height = 0);

    GrayLevelImage8(const GrayLevelImage8 &);

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
    GrayLevelImage8(int width, int height, GrayLevelImage8 * m, int xo, int yo);

    /**
     * Destructor
     */
    ~GrayLevelImage8();

    /** see Image::getElementTypeMinimum() */
    double getElementTypeMinimum() const { return 0; };

    /** see Image::getElementTypeMaximum() */
    double getElementTypeMaximum() const { return 255; };

    /** see Image::readFromFile() */
    void readFromFile(const char * fileName);
    /** see Image::writeToFile() */
    void writeToFile(const char * fileName) const;

    /**
     * Alternative access method to the image data as MultiElementImage.
     */
    MultiElementImage<byte,1> & asMultiElementImage();
        // we could initialize alt only, when we need it
        // { if (alt == NULL) alt = new ...; return *alt; }
        // but we would have to store xo, yo, -- another
        // DESIGN OPTION

    /** automatic conversion
      *
      * Advantage: a GrayLevelImage8 can be passed to a function
      * such as Test3<byte,1> directly
      *
      * Disadvantage: may be confusing as for the overloaded
      * function
      *  void test4(SingleElementImage<byte>& g);    // automatic choice
      *  void test4(MultiElementImage<byte,1>& g); // needs asMultiElementImage()
      *
      */
    operator MultiElementImage<byte,1>& () { return asMultiElementImage(); }

    void operator= (const GrayLevelImage8 & g) {
       SingleElementImage<byte>::operator=(g);
	}

  protected:
    virtual void reset(); /// reset internal IPL structures after resize
  private:
    MultiElementImage<byte,1> * alt; /// alternative
};

}

#endif
