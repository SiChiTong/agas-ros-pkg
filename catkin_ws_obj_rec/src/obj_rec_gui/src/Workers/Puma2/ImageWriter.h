/**
 * @file    ImageWriter.h
 * @brief   Contains ImageWriter.
 *
 * (c) 2007 AG Aktives Sehen <agas@uni-koblenz.de>, Universitaet Koblenz-Landau
 * $Id: $ 
 */

#ifndef IMAGE_WRITER_H
#define IMAGE_WRITER_H

#include "GrayLevelImage8.h"
#include "GrayLevelImage16.h"
#include "ColorImageRGB8.h"
#include "ColorImageRGBa8.h"
#include "FeatureImage.h"

#include <cstdio>

namespace puma2 {

/**
 * @class   ImageWriter
 * @brief   Write common image formats to file.
 * @author  Detlev Droege
 * @date    Februar 2007
 */

class ImageWriter {
 private:	// should never be instantiated
  ImageWriter () {};	///< default constructor
  ~ImageWriter () {};	///< default destructor

 public:
  /** Write an image to file filename.
   * The output file format wil be determined from the filename suffix.
   * @param[in] filename	the filename of the image 
   * @param[in] img	the image object where the image data should go
   * @return returns true for success, false otherwise
   */
  static bool writeImage(const GrayLevelImage8  &img, std::string filename);

  static bool writeImage(const GrayLevelImage16 &img, std::string filename);

  static bool writeImage(const ColorImageRGB8   &img, std::string filename);

  static bool writeImage(const ColorImageRGBa8  &img, std::string filename);

  /**
   * @brief Writes a FeatureImage to a file.
   * 
   * Writes a FeatureImage to a file. Since the class FeatureImage is a template
   * class, the implementation of this method has to be in the header file.
   */
  template <class T> static bool writeImage(FeatureImage<T> &img,
                                            std::string filename);
};

template <class T> bool ImageWriter::writeImage(FeatureImage<T> &img,
        std::string filename)
{
  GrayLevelImage8 g = img.getGrayLevelImageRepresentation();
  return writeImage(g, filename);
}

}

#endif /* IMAGE_WRITER_H */
