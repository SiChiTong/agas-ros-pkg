/*******************************************************************************
 *  ImageMaskCV.h
 *
 *  (C) 2007 AG Aktives Sehen <agas@uni-koblenz.de>
 *           Universitaet Koblenz-Landau
 *
 *  Additional information:
 *  $Id: $
 *******************************************************************************/

#ifndef ImageMaskCV_H
#define ImageMaskCV_H

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "Workers/Math/Box2D.h"


/**
 * @class  ImageMaskCV
 * @brief  Binary image mask implementation
 * @author David Gossow (RX), Viktor Seib (R20)
 * @note   Each pixel of the binary mask is represented a byte (0:masked, 255:unmasked)
 */
class ImageMaskCV
{
  public:

    enum MaskValues {
      MASKED = 0,
      VISIBLE = 255
    };

    /** @brief The constructor */
    ImageMaskCV( );

    /** @brief Copy / assignment */
    ImageMaskCV( const ImageMaskCV& other );
    ImageMaskCV& operator=( const ImageMaskCV& other );

    /** @brief Takes ownership of the given mask data. If no data is given, the mask is filled with VOID */
    ImageMaskCV( unsigned width, unsigned height, unsigned char* data=0 );

    /** @brief Creates a mask, values between maskMin and maskMax are considered as void (0) */
    ImageMaskCV( unsigned width, unsigned height, unsigned char* data, char maskedMin, char maskedMax );

    /** @brief Creates a mask by masking pixels above the given value */
    ImageMaskCV( cv::Mat& image, unsigned char minVal, unsigned char maxVal=255 );

    /** @brief Creates a difference mask */
    ImageMaskCV(cv::Mat& foregroundY, cv::Mat& foregroundUv,
              cv::Mat& backgroundY, cv::Mat& backgroundUv,
              int threshold );

    /** @brief The destructor */
    ~ImageMaskCV();

    void apply( cv::Mat& image, unsigned char fillValue=0 );
    void apply( cv::Mat& image, unsigned char fillR, unsigned char fillG, unsigned char fillB );

    /** @brief replace masked areas by gray values */
    void grayOut( cv::Mat& colorImage, cv::Mat& graimageY );
    void grayOut( cv::Mat& colorImage );

    /** @return true if the given value could be found within the given radius around (x,y)  */
    bool findValue( int x, int y, unsigned char value, float radius );

   /**
    * @brief Generates a circle with the given radius around each unmasked pixel
    * @note  Affects only areas that can fully be included within the circular mask (no border treatment)
    */
    void erode( float radius=1.0 );

   /**
    * @brief Generates a circle with the given radius around each masked pixel
    * @note  Affects only areas that can fully be included within the circular mask (no border treatment)
    */
    void dilate( float radius=1.0 );

    /** @brief Overwrite whole mask */
    void fill( unsigned char value );

    /** @brief Bitwise OR operation. Enlarges the unmasked areas by those of the other histogram. */
    void expand( const ImageMaskCV& other );

    /** @brief Leave only borders between masked and unmasked areas as VOID */
    void findBorders( );

    /** @return center of gravity of all unmasked pixels */
    Point2D getGravCenter( );

    Box2D<int> getBoundingBox();

    /** @return new ImageMaskCV covering only the given subarea */
    ImageMaskCV* subMask( Box2D<int> area );

   /**
    * @return  Pointer to the raw mask data
    * @warning Unsafe access to internal data. Avoid use if not neccesary.
    * @warning This pointer gets invalid when applying filters etc.
    */
    unsigned char* getData() { return m_Data; }

    inline unsigned getWidth() { return m_Width; }
    inline unsigned getHeight() { return m_Height; }

  private:

    enum maskOperationT{
      dilateOperation,
      erodeOperation
    };

    void maskOperation( maskOperationT operation, float radius );

    /** @brief creates a circular filter kernel and stores the offsets off all occupied pixels in an array */
    void createCircularKernel( float radius, int*& maskOffset, int& halfMaskSize, unsigned& maskLength );

    unsigned char* m_Data;

    unsigned m_Width;
    unsigned m_Height;

};

#endif
