/**
 * @file    ImageReader.h
 * @brief   Contains ImageReader.
 *
 * (c) 2007 AG Aktives Sehen <agas@uni-koblenz.de>, Universitaet Koblenz-Landau
 * $Id: $ 
 */

#ifndef IMAGE_READER_H
#define IMAGE_READER_H

#include "GrayLevelImage8.h"
#include "GrayLevelImage16.h"
#include "ColorImageRGB8.h"
#include "ColorImageRGBa8.h"

#include <cstdio>

namespace puma2 {

/**
 * @class   ImageReader
 * @brief   Read common image formats from file or other input channels.
 * @author  Detlev Droege
 * @date    Februar 2007
 */

class ImageReader {
 private:	// should never be instantiated
  ImageReader () {};	///< default constructor
  ~ImageReader () {};	///< default destructor

 public:
  /** Read an image from file filename into the given instance of an Image.
   * The image data will be auto-converted to an 8-bit gray level image if 
   * it isn't such a thing already.
   * @param[in,out] img		the GrayLevelImage8 object where the image data should go
   * @param[in] filename	the filename of the image 
   * @return returns true for success, false otherwise
   */
  static bool readImage(GrayLevelImage8 &img, std::string filename);

  static bool readImage(GrayLevelImage16 &img, std::string filename);

  /** Read an image from file filename into the given instance of an Image.
   * The image data will be auto-converted to a 3 times 8-bit RGB image if 
   * it isn't such a thing already.
   * @param[in,out] img		the ColorImageRGB8 object where the image data should go
   * @param[in] filename	the filename of the image 
   * @return returns true for success, false otherwise
   */
  static bool readImage(ColorImageRGB8  &img, std::string filename);

  /** Load a JPEG image from some memory area and decompress into the given instance of an Image.
   * The image data will be converted to a 3 times 8-bit RGB image.
   * @param[in,out] img		the ColorImageRGB8 object where the image data should go
   * @param[in] ptr		location in mamory where the data starts 
   * @param[in] numBytes	size of the memory area pointed to by ptr.
   * @return returns true for success, false otherwise
   */
  static bool loadJPEGImage(ColorImageRGB8  &img, byte* ptr, size_t numBytes);

  /** Read an image from file filename into the given instance of an Image.
   * The image data will be auto-converted to a 4 times 8-bit RGBA image if 
   * it isn't such a thing already.
   * If no alpha information is present in the file, it will be set to
   * valueRangeMaximum, denoting 'opaque' pixels.
   * @param[inout] img		the ColorImageRGBa8 object where the image data should go
   * @param[in] filename	the filename of the image 
   * @return returns true for success, false otherwise
   */
  static bool readImage(ColorImageRGBa8  &img, std::string filename);
};

}

#endif /* IMAGE_READER_H */
