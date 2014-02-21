#ifndef ColorImageRGB8a_H
#define ColorImageRGB8a_H

#include "ImageSuperClasses.h"
#include "SingleElementImage.h"
#include "MultiElementImage.h"

namespace puma2 {

typedef unsigned char byte;

struct RGBa8 {
  union {
     struct {                   //< order needs to be checked
      unsigned int r:8;
      unsigned int g:8;
      unsigned int b:8;
      unsigned int a:8;
     };
     unsigned int u;  //< should be 32 bit
     byte w[4];       //< provides alternative access method
   };
   RGBa8 & operator=(const RGBa8 o) { u = o.u; return *this; }
   void operator= (int v) { u = v; }
   byte & operator[] (int i) { return w[i]; }
   byte operator[] (int i) const { return w[i]; }
   void assign(int v, int n) { w[n] = v; }
   typedef byte B4[4];
};

template <> inline void Vector<RGBa8::B4>::copy(const RGBa8::B4& f, RGBa8::B4& t)
	{ t[0] = f[0];
	  t[1] = f[1];
	  t[2] = f[2];
	  t[3] = f[3]; }
template <> inline void Vector<RGBa8>::copy(const RGBa8& f, RGBa8& t) { t.u = f.u; }

/**
 * @class ColorImageRGBa8
 * @brief Imageclass for an 8-bit, 4 channel RGBA color image.
 *
 * This class superseeds ColorImageRGB in two aspects: It internally uses
 * a data structure to store all samples of a pixel, which allows for
 * the efficient assignment of entire pixels in a single statement,
 * and it adds an alpha channel, containing transparency information
 * for each pixel.
 * In fact, the alpha value stores not the 'transparency' but the
 * 'opacity' of a pixel (which is just the inverse). That is,
 * alpha = 0 (better: alpha = getValueRangeMinimum()) means
 * least opacity (== full transparency), while alpha = getValueRangeMaximum()
 * denotes maximum opacity (== totally intransparent, as it would
 * be considered for an image without alpha channel).
 * Alpha values are considered to be in the range of 0.0 .. 1.0
 * (represented as 0 .. 255 in ColorImageRGBa8).
 * \f$\alpha = 0.5\f$ then represents a 50% opaqe pixel.
 * For any pixel p in the image with some alpha value \f$\alpha\f$, which is
 * to be rendered above some background color b, the resulting
 * color c is computed as
 *   \f$c = \alpha\cdot p + (1-\alpha)\cdot b\f$
 * Alpha channels can be stored with some common image file formats,
 * notably PNG and TIFF, but <b>not</b> with e.g. PPM/PGM or JPEG.
 */
class ColorImageRGBa8 :
    public SingleElementImage<RGBa8>
{
  public:
    ColorImageRGBa8(int x = 0, int y = 0);

    ColorImageRGBa8(int x, int y, ColorImageRGBa8 * m, int xo, int yo);

    /**
     * Set a single value
     * @param[in] i Line
     * @param[in] j Row
     * @param[in] n Channel
     * @param[in] v Value
     */
    void assign(int i, int j, int n, int v);

    // ~ColorImageRGBa8() { delete alt; }
    MultiElementImage<byte,4> & asMultiElementImage();

    static int numberOfChannels();

    /** see Image::getElementTypeMinimum() */
    double getElementTypeMinimum() const { return 0; };

    /** see Image::getElementTypeMaximum() */
    double getElementTypeMaximum() const { return 255; };

    /** see Image::readFromFile() */
    void readFromFile(const char * fileName);
    /** see Image::writeToFile() */
    void writeToFile(const char * fileName) const;

  private:
        /** TEST - ONE DESIGN OPTION:
        The image pointer alt will be
        initialized using the sub-image constructor.
        Using a hard type conversion cast we will
        cheat and this image will not allocate memory itself.
        Instead, it will reuse the memory allocated for the
        matrix in the base class SingleElementImage.

        (validity of casts could be checked by a singleton
        upon initialization of the program)
      */
    MultiElementImage<byte,4> alt; // alternative access
};

}

#endif
