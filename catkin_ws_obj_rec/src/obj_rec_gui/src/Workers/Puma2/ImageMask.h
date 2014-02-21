/*******************************************************************************
 *  ImageMask.h
 *
 *  (C) 2007 AG Aktives Sehen <agas@uni-koblenz.de>
 *           Universitaet Koblenz-Landau
 *
 *  Additional information:
 *  $Id: $
 *******************************************************************************/

#ifndef ImageMask_H
#define ImageMask_H

#include "GrayLevelImage8.h"
#include "ColorImageRGB8.h"
#include "ColorImageUV8.h"

#include "Workers/Math/Box2D.h"

/**
 * @class  ImageMask
 * @brief  Binary image mask implementation
 * @author David Gossow (RX)
 * @note   Each pixel of the binary mask is represented a byte (0:masked, 255:unmasked)
 */
class ImageMask
{
  public:

    enum MaskValues {
      MASKED = 0,
      VISIBLE = 255
    };

    /** @brief The constructor */
    ImageMask( );

    /** @brief Copy / assignment */
    ImageMask( const ImageMask& other );
    ImageMask& operator=( const ImageMask& other );

    /** @brief Takes ownership of the given mask data. If no data is given, the mask is filled with VOID */
    ImageMask( unsigned width, unsigned height, unsigned char* data=0 );

    /** @brief Creates a mask, values between maskMin and maskMax are considered as void (0) */
    ImageMask( unsigned width, unsigned height, unsigned char* data, char maskedMin, char maskedMax );

    /** @brief Creates a mask by masking pixels above the given value */
    ImageMask( puma2::GrayLevelImage8& image, unsigned char minVal, unsigned char maxVal=255 );
    ImageMask( puma2::ColorImageUV8& image, unsigned char minValU, unsigned char minValV );

    /** @brief Creates a difference mask */
    ImageMask(puma2::GrayLevelImage8& foregroundY, puma2::ColorImageUV8& foregroundUv,
              puma2::GrayLevelImage8& backgroundY, puma2::ColorImageUV8& backgroundUv,
              int threshold );

    /** @brief The destructor */
    ~ImageMask();

    void apply( puma2::GrayLevelImage8& image, unsigned char fillValue=0 );
    void apply( puma2::ColorImageUV8& image, unsigned char fillU=0, unsigned char fillV=0 );
    void apply( puma2::ColorImageRGB8& image, unsigned char fillR, unsigned char fillG, unsigned char fillB );

    /** @brief replace masked areas by gray values */
    void grayOut( puma2::ColorImageRGB8& colorImage, puma2::GrayLevelImage8& graimageY );
    void grayOut( puma2::ColorImageRGB8& colorImage );

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
    void expand( const ImageMask& other );

    /** @brief Leave only borders between masked and unmasked areas as VOID */
    void findBorders( );

    /** @return center of gravity of all unmasked pixels */
    Point2D getGravCenter( );

    Box2D<int> getBoundingBox();

    /** @return new ImageMask covering only the given subarea */
    ImageMask* subMask( Box2D<int> area );

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
