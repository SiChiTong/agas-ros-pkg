#ifndef ColorImageRGB8_H
#define ColorImageRGB8_H

#include "ImageSuperClasses.h"
#include "MultiElementImage.h"

namespace puma2 {

typedef unsigned char byte;

/**
 * @class ColorImageRGB8
 * @brief Imageclass for an 8-bit, 3 channel RGB color image.
 */
class ColorImageRGB8 :
  public MultiElementImage<byte,3>
{
  public:

    /**
     * Default constructor.
     */
    ColorImageRGB8(int x = 0, int y = 0);

    /**
     * Subimage constructor:
     *   @param x: horizontal size
     *   @param y: vertical size
     *   @param m: Pointer to master image of which the image becomes a subimage
     *   @param xo: offset for horizontal position of subimage in master image
     *   @param yo: offset for vertical position of subimage in master image
     *
     * The constructor will do all required checks on the sizes of the images.
     */
    ColorImageRGB8(int x, int y, ColorImageRGB8 * m, int xo, int yo);

    /**
     * Set a single value
     * @param[in] i Line
     * @param[in] j Row
     * @param[in] n Channel
     * @param[in] v Value
     */
    void assign(int i, int j, int n, int v);

    /**
     * Set all values
     * @param[in] x Row
     * @param[in] y Line
     * @param[in] r R-value
     * @param[in] g G-value
     * @param[in] b B-value
     */
    void assign(int x, int y, int r, int g, int b);

    void operator= (const ColorImageRGB8 & o);


    /** see Image::getElementTypeMinimum() */
    double getElementTypeMinimum() const { return 0; };

    /** see Image::getElementTypeMaximum() */
    double getElementTypeMaximum() const { return 255; };

    /** see Image::readFromFile() */
    void readFromFile(const char * fileName);
    /** see Image::writeToFile() */
    void writeToFile(const char * fileName) const;
	typedef byte B3[3];
protected:
    void reset();
};
template <> inline void Vector<ColorImageRGB8::B3>::copy(const ColorImageRGB8::B3& f, ColorImageRGB8::B3& t)
{
	  t[0] = f[0];
	  t[1] = f[1];
	  t[2] = f[2];

}

}
#endif

