/*******************************************************************************
 *  ImageMask.cpp
 *
 *  (C) 2007 AG Aktives Sehen <agas@uni-koblenz.de>
 *           Universitaet Koblenz-Landau
 *
 *  Additional information:
 *  $Id: $
 *******************************************************************************/

#include <cstring>
#include "ImageMask.h"
// #include "Architecture/Tracer/Tracer.h" // TODO

#include <math.h>
#include <iostream>

#define THIS ImageMask

using namespace puma2;


THIS::THIS()
{
  m_Width = 0;
  m_Height = 0;
  m_Data = 0;
}

THIS::THIS ( unsigned width, unsigned height, unsigned char* data )
{
  m_Width = width;
  m_Height = height;
  if ( !data )
  {
    m_Data = new unsigned char[width*height];
    fill ( MASKED );
  }
  else
  {
    m_Data = data;
  }
}


THIS::THIS ( unsigned width, unsigned height, unsigned char* data, char voidMin, char voidMax )
{
  m_Width = width;
  m_Height = height;
  unsigned dataSize = width * height;
  m_Data = new unsigned char[dataSize];
  if ( !data )
  {
    // TRACE_ERROR ( "Mask data is 0-pointer!" ); // TODO use ros
    fill ( MASKED );
  }
  else
  {
    for ( unsigned i = 0; i < dataSize; i++ )
    {
      if ( ( data[i] >= voidMin ) && ( data[i] <= voidMax ) )
      {
        m_Data[i] = MASKED;
      }
      else
      {
        m_Data[i] = VISIBLE;
      }
    }
  }
}


THIS::THIS ( const ImageMask& other )
{
  m_Data = 0;
  operator= ( other );
}

THIS& THIS::operator= ( const ImageMask & other )
{
  if ( m_Data ) { delete m_Data; }
  m_Width = other.m_Width;
  m_Height = other.m_Height;
  m_Data = new unsigned char[m_Width*m_Height];
  memcpy ( m_Data, other.m_Data, m_Width*m_Height );
  return *this;
}

THIS::~THIS()
{
  if ( m_Data )
  {
      delete[] m_Data;
  }
}

THIS::THIS ( GrayLevelImage8& image, unsigned char minVal, unsigned char maxVal )
{
  m_Width = image.getWidth();
  m_Height = image.getHeight();
  m_Data = new unsigned char[m_Width*m_Height];
  fill ( 255 );
  //mask all pixels with value maskVal
  unsigned char* imageRow;
  unsigned offsetMask = 0;
  for ( unsigned y = 0; y < m_Height; y++ )
  {
    imageRow = image.unsafeRowPointerArray() [y];
    for ( unsigned x = 0; x < m_Width; x++ )
    {
      if ( ( imageRow[x] >= minVal ) && ( imageRow[x] <= maxVal ) )
      {
        m_Data[offsetMask] = MASKED;
      }
      offsetMask++;
    }
  }
}

THIS::THIS ( puma2::ColorImageUV8& image, unsigned char minValU, unsigned char minValV )
{
  m_Width = image.getWidth();
  m_Height = image.getHeight();
  m_Data = new unsigned char[m_Width*m_Height];
  fill ( 255 );
  //mask all pixels with value maskVal
  ColorImageUV8::PixelType* imageRow;
  unsigned offsetMask = 0;
  for ( unsigned y = 0; y < m_Height; y++ )
  {
    imageRow = image.unsafeRowPointerArray() [y];
    for ( unsigned x = 0; x < m_Width; x++ )
    {
      if ( ( imageRow[x][0] >= minValU ) )
      {
        m_Data[offsetMask] = MASKED;
      }
      offsetMask++;
    }
  }
}

THIS::THIS ( puma2::GrayLevelImage8& foregroundY, puma2::ColorImageUV8& foregroundUv,
             puma2::GrayLevelImage8& backgroundY, puma2::ColorImageUV8& backgroundUv,
             int threshold )
{
  if ( ( foregroundUv.getWidth() != foregroundY.getWidth() ) ||
       ( foregroundUv.getHeight() != foregroundY.getHeight() ) ||
       ( foregroundUv.getWidth() != backgroundY.getWidth() ) ||
       ( foregroundUv.getHeight() != backgroundY.getHeight() ) ||
       ( foregroundUv.getWidth() != backgroundUv.getWidth() ) ||
       ( foregroundUv.getHeight() != backgroundUv.getHeight() ) )
  {
    // TRACE_ERROR ( "Image size mismatch in difference mask constructor." ); // TODO use ros
    m_Data = 0;
    return;
  }

  m_Width = foregroundY.getWidth();
  m_Height = foregroundY.getHeight();
  int threshold2 = threshold * threshold;

  m_Data = new unsigned char[m_Width*m_Height];
  fill ( 255 );

  ColorImageUV8::PixelType** foregroundUvRow = foregroundUv.unsafeRowPointerArray();
  ColorImageUV8::PixelType** backgroundUvRow = backgroundUv.unsafeRowPointerArray();
  puma2::byte** foregroundYRow = foregroundY.unsafeRowPointerArray();
  puma2::byte** backgroundYRow = backgroundY.unsafeRowPointerArray();

  ColorImageUV8::PixelType* foregroundUvPointer;
  ColorImageUV8::PixelType* backgroundUvPointer;
  puma2::byte* foregroundYPointer;
  puma2::byte* backgroundYPointer;

  unsigned char* currentMaskPixel = m_Data;

  for ( unsigned int y = 0; y < m_Height ; y++ )
  {
    foregroundUvPointer = foregroundUvRow[y];
    backgroundUvPointer = backgroundUvRow[y];
    foregroundYPointer = foregroundYRow[y];
    backgroundYPointer = backgroundYRow[y];
    for ( unsigned int x = 0; x < m_Width ; x++ )
    {
      int grayDiff = foregroundYPointer[x] - backgroundYPointer[x];
      int uDiff = foregroundUvPointer[x][0] - backgroundUvPointer[x][0];
      int vDiff = foregroundUvPointer[x][1] - backgroundUvPointer[x][1];

//       uDiff *= 5;
//       vDiff *= 5;

      int distance = grayDiff * grayDiff + uDiff * uDiff + vDiff * vDiff;

      if ( distance < threshold2 )
      {
        *currentMaskPixel = MASKED;
      }
      currentMaskPixel++;
    }
  }

}

void THIS::fill ( unsigned char value )
{
  if ( !m_Data )
  {
   // TRACE_ERROR ( "This mask is empty." );// TODO use ros
    return;
  }
  memset ( m_Data, value, m_Width*m_Height );
}


void THIS::expand ( const ImageMask& other )
{
  if ( !m_Data )
  {
    // TRACE_ERROR ( "This mask is empty." );// TODO use ros
    return;
  }
  if ( !other.m_Data )
  {
    // TRACE_ERROR ( "Other mask is empty." );// TODO use ros
    return;
  }
  if ( ( m_Width != other.m_Width ) || ( m_Height != other.m_Height ) )
  {
   // TRACE_ERROR ( "Wrong mask size." );// TODO use ros
    return;
  }
  for ( unsigned i = 0; i < m_Width*m_Height; i++ )
  {
    m_Data[i] |= other.m_Data[i];
  }
}


void THIS::apply ( GrayLevelImage8& image, unsigned char fillValue )
{
  if ( !m_Data )
  {
   // TRACE_ERROR ( "This mask is empty." );// TODO use ros
    return;
  }
  if ( ( unsigned ( image.getWidth() ) != m_Width ) != ( unsigned ( image.getHeight() ) != m_Height ) )
  {
   //  TRACE_ERROR ( "Wrong image size." );// TODO use ros
    return;
  }

  unsigned char* currentYPixel = image.unsafeRowPointerArray() [0];
  unsigned char* currentMaskPixel = m_Data;

  for ( unsigned y = 0; y < m_Height; y++ )
    for ( unsigned x = 0; x < m_Width; x++ )
    {
      if ( ! ( *currentMaskPixel ) )
      {
        ( *currentYPixel ) = fillValue;
      }
      currentYPixel++;
      currentMaskPixel++;
    }
}


void THIS::apply ( puma2::ColorImageUV8& image, unsigned char fillU, unsigned char fillV )
{
  if ( !m_Data )
  {
   // TRACE_ERROR ( "This mask is empty." );// TODO use ros
    return;
  }
  if ( ( unsigned ( image.getWidth() ) != m_Width ) != ( unsigned ( image.getHeight() ) != m_Height ) )
  {
   // TRACE_ERROR ( "Wrong image size." );// TODO use ros
    return;
  }

  ColorImageUV8::PixelType* currentUvPixel = image.unsafeRowPointerArray() [0];
  unsigned char* currentMaskPixel = m_Data;

  for ( unsigned y = 0; y < m_Height; y++ )
    for ( unsigned x = 0; x < m_Width; x++ )
    {
      if ( ! ( *currentMaskPixel ) )
      {
        ( *currentUvPixel ) [0] = fillU;
        ( *currentUvPixel ) [1] = fillV;
      }
      currentUvPixel++;
      currentMaskPixel++;
    }
}


void THIS::apply ( puma2::ColorImageRGB8& image, unsigned char fillR, unsigned char fillG, unsigned char fillB )
{
  if ( !m_Data )
  {
    //TRACE_ERROR ( "This mask is empty." );// TODO use ros
    return;
  }
  if ( ( unsigned ( image.getWidth() ) != m_Width ) != ( unsigned ( image.getHeight() ) != m_Height ) )
  {
    //TRACE_ERROR ( "Wrong image size." );// TODO use ros
    return;
  }

  ColorImageRGB8::PixelType* currentRgbPixel = image.unsafeRowPointerArray() [0];
  unsigned char* currentMaskPixel = m_Data;

  for ( unsigned y = 0; y < m_Height; y++ )
    for ( unsigned x = 0; x < m_Width; x++ )
    {
      if ( ! ( *currentMaskPixel ) )
      {
        ( *currentRgbPixel ) [0] = fillR;
        ( *currentRgbPixel ) [1] = fillG;
        ( *currentRgbPixel ) [2] = fillB;
      }
      currentRgbPixel++;
      currentMaskPixel++;
    }
}


void THIS::grayOut ( puma2::ColorImageRGB8& colorImage, puma2::GrayLevelImage8& graimageY )
{
  if ( !m_Data )
  {
    //TRACE_ERROR ( "This mask is empty." );// TODO use ros
    return;
  }
  if ( ( ( unsigned ( colorImage.getWidth() ) != m_Width ) != ( unsigned ( colorImage.getHeight() ) != m_Height ) ) ||
       ( ( unsigned ( graimageY.getWidth() ) != m_Width ) != ( unsigned ( graimageY.getHeight() ) != m_Height ) ) )
  {
    //TRACE_ERROR ( "Wrong image size." );// TODO use ros
    return;
  }

  ColorImageRGB8::PixelType* currentRgbPixel = colorImage.unsafeRowPointerArray() [0];
  unsigned char* currentYPixel = graimageY.unsafeRowPointerArray() [0];
  unsigned char* currentMaskPixel = m_Data;
  unsigned val;

  for ( unsigned y = 0; y < m_Height; y++ )
    for ( unsigned x = 0; x < m_Width; x++ )
    {
      if ( ! ( *currentMaskPixel ) )
      {
        val = *currentYPixel / 2 + 64;
        ( *currentRgbPixel ) [0] = val;
        ( *currentRgbPixel ) [1] = val;
        ( *currentRgbPixel ) [2] = val;
      }
      currentRgbPixel++;
      currentYPixel++;
      currentMaskPixel++;
    }
}


void THIS::grayOut ( puma2::ColorImageRGB8& colorImage )
{
  if ( !m_Data )
  {
   // TRACE_ERROR ( "This mask is empty." );// TODO use ros
    return;
  }

  ColorImageRGB8::PixelType* currentRgbPixel = colorImage.unsafeRowPointerArray() [0];
  unsigned char* currentMaskPixel = m_Data;
  unsigned val;

  for ( unsigned y = 0; y < m_Height; y++ )
    for ( unsigned x = 0; x < m_Width; x++ )
    {
      if ( ! ( *currentMaskPixel ) )
      {
        val = ( ( *currentRgbPixel ) [0] + ( *currentRgbPixel ) [1] + ( *currentRgbPixel ) [2] ) / 6 + 64;
        ( *currentRgbPixel ) [0] = val;
        ( *currentRgbPixel ) [1] = val;
        ( *currentRgbPixel ) [2] = val;
      }
      currentRgbPixel++;
      currentMaskPixel++;
    }
}


bool THIS::findValue ( int centerX, int centerY, unsigned char value, float radius )
{
  if ( !m_Data )
  {
    //TRACE_ERROR ( "This mask is empty." );// TODO use ros
    return false;
  }

  int startX = int ( centerX - radius );
  int startY = int ( centerY - radius );
  int endX = int ( centerX + radius );
  int endY = int ( centerY + radius );

  if ( startX < 0 ) { startX = 0; }
  if ( startY < 0 ) { startY = 0; }
  if ( endX >= int ( m_Width ) ) { endX = m_Width - 1; }
  if ( endY >= int ( m_Height ) ) { endY = m_Height - 1; }

  float sqrRadius = radius*radius;

  for ( int y = startY; y <= endY; y++ )
    for ( int x = startX; x <= endX; x++ )
    {
//       std::cout << int(m_Data[x+m_Width*y]) << " ";
      if ( m_Data[x+m_Width*y] == value )
      {
        float sqrDist = float ( x - centerX ) * float ( x - centerX ) + float ( y - centerY ) * float ( y - centerY );
        if ( sqrDist <= sqrRadius ) { return true; }
      }
    }

  return false;
}


void THIS::erode ( float radius )
{
  maskOperation ( erodeOperation, radius );
}


void THIS::dilate ( float radius )
{
  maskOperation ( dilateOperation, radius );
}


void THIS::maskOperation ( maskOperationT operation, float radius )
{
  if ( !m_Data )
  {
   // TRACE_ERROR ( "This mask is empty." );// TODO use ros
    return;
  }

  if ( radius < 1.0 ) { return; }

  int* offsetMask;
  int halfMaskSize;
  unsigned maskLength;

  //create circular filter mask
  createCircularKernel ( radius, offsetMask, halfMaskSize, maskLength );

  //copy mask data
  unsigned char* data = new unsigned char[m_Width*m_Height];
  memcpy ( data, m_Data, m_Width*m_Height );

  //apply filter mask
  unsigned i = halfMaskSize + m_Width * halfMaskSize;
  unsigned char fillValue;

  switch ( operation )
  {
    case erodeOperation:
      fillValue = MASKED;
      break;
    case dilateOperation:
      fillValue = VISIBLE;
      break;
  }

  for ( unsigned y = halfMaskSize; y < m_Height - halfMaskSize; y++ )
  {
    for ( unsigned x = halfMaskSize; x < m_Width - halfMaskSize; x++ )
    {
      if ( m_Data[i] && ! ( m_Data[i-1] && m_Data[i+1] && m_Data[i-m_Width] && m_Data[i+m_Width] ) )
      {
        for ( unsigned j = 0; j < maskLength; j++ )
        {
          data[ i + offsetMask[j] ] = fillValue;
        }
      }
      i++;
    }
    i += halfMaskSize * 2;
  }

  //switch to new mask
  delete[] m_Data;
  m_Data = data;

  delete[] offsetMask;
}


void THIS::findBorders( )
{
  if ( !m_Data )
  {
   // TRACE_ERROR ( "This mask is empty." );// TODO use ros
    return;
  }

  //copy mask data
  unsigned char* data = new unsigned char[m_Width*m_Height];
  memset ( data, VISIBLE, m_Width*m_Height );

  //apply filter mask
  unsigned i = m_Width + 1;

  for ( unsigned y = 1; y < m_Height - 1; y++ )
  {
    for ( unsigned x = 1; x < m_Width - 1; x++ )
    {
      if ( m_Data[i] && ! ( m_Data[i-1] && m_Data[i+1] && m_Data[i-m_Width] && m_Data[i+m_Width] ) )
      {
        data[i] = MASKED;
      }
      i++;
    }
    i += 2;
  }

  //switch to new mask
  delete[] m_Data;
  m_Data = data;
}


void THIS::createCircularKernel ( float radius, int*& offsetMask, int& halfMaskSize, unsigned& maskLength )
{
  unsigned maskSize = unsigned ( radius ) * 2 + 1;
  halfMaskSize = maskSize / 2;
  offsetMask = new int[maskSize*maskSize-1];
  unsigned i = 0;
  float dist;
  for ( int y = -halfMaskSize; y <= halfMaskSize; y++ )
    for ( int x = -halfMaskSize; x <= halfMaskSize; x++ )
    {
      dist = sqrt ( float ( x ) * float ( x ) + float ( y ) * float ( y ) );
      if ( dist <= radius )
      {
        offsetMask[i] = x + m_Width * y;
        i++;
      }
    }
  maskLength = i;
}

Point2D THIS::getGravCenter ( )
{
  double centerX = 0;
  double centerY = 0;
  int numPixels = 0;
  int i = 0;

  for ( unsigned y = 0; y < m_Height; y++ )
  {
    for ( unsigned x = 0; x < m_Width; x++ )
    {
      if ( m_Data[i] == VISIBLE )
      {
        centerX += x;
        centerY += y;
        numPixels ++;
      }
      i++;
    }
  }
  centerX /= numPixels;
  centerY /= numPixels;

  return Point2D( centerX, centerY );
}

Box2D<int> THIS::getBoundingBox()
{
  //initialize "negative" bounding box (min/max swapped)
  Box2D<int> bBox( m_Width, m_Height, 0, 0 );
  int i = 0;

  for ( unsigned y = 0; y < m_Height; y++ )
  {
    for ( unsigned x = 0; x < m_Width; x++ )
    {
      if ( m_Data[i] == VISIBLE )
      {
        bBox.enclose( x, y );
      }
      i++;
    }
  }
  bBox.enclose( bBox.maxX()+1, bBox.maxY()+1 );

  return bBox;
}

THIS* THIS::subMask( Box2D<int> area )
{
  int newWidth = area.width();
  int newHeight= area.height();
  THIS* result = new THIS( newWidth, newHeight );
  unsigned char* newData = result->getData();

  int minX = area.minX();
  int minY = area.minY();

  int i = 0;

  for ( int y = 0; y < newHeight; y++ )
  {
    for ( int x = 0; x < newWidth; x++ )
    {
      newData[i] = m_Data[ minX+x + ( minY+y )*m_Width ];
      i++;
    }
  }

  return result;
}

#undef DEBUG_OUTPUT
#undef THIS
