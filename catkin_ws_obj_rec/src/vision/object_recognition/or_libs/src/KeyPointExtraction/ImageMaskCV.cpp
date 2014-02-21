/*******************************************************************************
 *  ImageMaskCV.cpp
 *
 *  (C) 2007 AG Aktives Sehen <agas@uni-koblenz.de>
 *           Universitaet Koblenz-Landau
 *
 *  Additional information:
 *  $Id: $
 *******************************************************************************/

#include <cstring>
#include "ImageMaskCV.h"
// #include "Architecture/Tracer/Tracer.h" // TODO

#include <math.h>
#include <iostream>

#define THIS ImageMaskCV


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


THIS::THIS ( const ImageMaskCV& other )
{
  m_Data = 0;
  operator= ( other );
}

THIS& THIS::operator= ( const ImageMaskCV & other )
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

THIS::THIS ( cv::Mat& image, unsigned char minVal, unsigned char maxVal )
{
  m_Width = image.cols;
  m_Height = image.rows;
  m_Data = new unsigned char[m_Width*m_Height];
  fill ( 255 );
  //mask all pixels with value maskVal
  unsigned offsetMask = 0;
  for ( unsigned y = 0; y < m_Height; y++ )
  {
    for ( unsigned x = 0; x < m_Width; x++ )
    {
        // handle gray level images
        if(image.type() == CV_8UC1)
        {
          if ( ( image.at<unsigned char>(y,x) >= minVal ) && ( image.at<unsigned char>(y,x) <= maxVal ) )
          {
            m_Data[offsetMask] = MASKED;
          }
        }
        // handle color images
        else if(image.type() == CV_8UC3)
        {
            if ( ( image.at<cv::Vec3b>(y,x)[0] >= minVal ) )
            {
              m_Data[offsetMask] = MASKED;
            }
        }
      offsetMask++;
    }
  }
}


THIS::THIS (cv::Mat& foregroundY, cv::Mat& foregroundUv,
             cv::Mat& backgroundY, cv::Mat& backgroundUv,
             int threshold )
{
  if ( ( foregroundUv.cols != foregroundY.cols ) ||
       ( foregroundUv.rows != foregroundY.rows ) ||
       ( foregroundUv.cols != backgroundY.cols ) ||
       ( foregroundUv.rows != backgroundY.rows ) ||
       ( foregroundUv.cols != backgroundUv.cols ) ||
       ( foregroundUv.rows != backgroundUv.rows ) )
  {
    // TRACE_ERROR ( "Image size mismatch in difference mask constructor." ); // TODO use ros
    m_Data = 0;
    return;
  }

  m_Width = foregroundY.cols;
  m_Height = foregroundY.rows;
  int threshold2 = threshold * threshold;

  m_Data = new unsigned char[m_Width*m_Height];
  fill ( 255 );

  unsigned char* currentMaskPixel = m_Data;

  for ( unsigned int y = 0; y < m_Height ; y++ )
  {
    for ( unsigned int x = 0; x < m_Width ; x++ )
    {
        // TODO check whether this is working correctly

        int grayDiff = foregroundY.at<unsigned char>(y,x) - backgroundY.at<unsigned char>(y,x);

        int bDiff = foregroundUv.at<cv::Vec3b>(y,x)[0] - backgroundUv.at<cv::Vec3b>(y,x)[0];
        int gDiff = foregroundUv.at<cv::Vec3b>(y,x)[1] - backgroundUv.at<cv::Vec3b>(y,x)[1];
        int rDiff = foregroundUv.at<cv::Vec3b>(y,x)[2] - backgroundUv.at<cv::Vec3b>(y,x)[2];

      int distance = grayDiff * grayDiff + bDiff * bDiff + gDiff * gDiff + rDiff * rDiff;

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


void THIS::expand ( const ImageMaskCV& other )
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


void THIS::apply ( cv::Mat& image, unsigned char fillValue )
{
  if ( !m_Data )
  {
   // TRACE_ERROR ( "This mask is empty." );// TODO use ros
    return;
  }
  if ( ( unsigned ( image.cols ) != m_Width ) != ( unsigned ( image.rows ) != m_Height ) )
  {
   //  TRACE_ERROR ( "Wrong image size." );// TODO use ros
    return;
  }

  unsigned char* currentMaskPixel = m_Data;

  for ( unsigned y = 0; y < m_Height; y++ )
    for ( unsigned x = 0; x < m_Width; x++ )
    {
      if ( ! ( *currentMaskPixel ) )
      {
          image.at<unsigned char>(y,x) = fillValue;
      }
      currentMaskPixel++;
    }
}


void THIS::apply ( cv::Mat& image, unsigned char fillR, unsigned char fillG, unsigned char fillB )
{
  if ( !m_Data )
  {
    //TRACE_ERROR ( "This mask is empty." );// TODO use ros
    return;
  }
  if ( ((unsigned) image.cols != m_Width)  != ( (unsigned) image.rows != m_Height ) )
  {
    //TRACE_ERROR ( "Wrong image size." );// TODO use ros
    return;
  }

  unsigned char* currentMaskPixel = m_Data;

  for ( unsigned y = 0; y < m_Height; y++ )
    for ( unsigned x = 0; x < m_Width; x++ )
    {
      if ( ! ( *currentMaskPixel ) )
      {
       image.at<cv::Vec3b>(y,x) = cv::Vec3b(fillG, fillR, fillB);
      }
      currentMaskPixel++;
    }
}


void THIS::grayOut ( cv::Mat& colorImage, cv::Mat& graimageY )
{
  if ( !m_Data )
  {
    //TRACE_ERROR ( "This mask is empty." );// TODO use ros
    return;
  }

  if( colorImage.type() != CV_8UC3)
  {
      // TRACE_ERROR ( "Wrong data type: CV_8UC3 is needed." );// TODO use ros
       return;
  }

  if( graimageY.type() != CV_8UC1)
  {
      // TRACE_ERROR ( "Wrong data type: CV_8UC1 is needed." );// TODO use ros
       return;
  }

  if ( ( ( unsigned ( colorImage.cols ) != m_Width ) != ( unsigned ( colorImage.rows ) != m_Height ) ) ||
       ( ( unsigned ( graimageY.cols ) != m_Width ) != ( unsigned ( graimageY.rows ) != m_Height ) ) )
  {
    //TRACE_ERROR ( "Wrong image size." );// TODO use ros
    return;
  }

  unsigned char* currentMaskPixel = m_Data;
  unsigned val;

  for ( unsigned y = 0; y < m_Height; y++ )
    for ( unsigned x = 0; x < m_Width; x++ )
    {
      if ( ! ( *currentMaskPixel ) )
      {
          val = graimageY.at<unsigned char>(y,x) / 2 + 64;
          colorImage.at<cv::Vec3b>(y,x) = cv::Vec3b(val,val,val);
      }
      currentMaskPixel++;
    }
}


void THIS::grayOut ( cv::Mat& colorImage )
{
  if ( !m_Data )
  {
   // TRACE_ERROR ( "This mask is empty." );// TODO use ros
    return;
  }

  if( colorImage.type() != CV_8UC3)
  {
      // TRACE_ERROR ( "Wrong data type: CV_8UC3 is needed." );// TODO use ros
       return;
  }

  unsigned char* currentMaskPixel = m_Data;
  unsigned val;

  for ( unsigned y = 0; y < m_Height; y++ )
    for ( unsigned x = 0; x < m_Width; x++ )
    {
      if ( ! ( *currentMaskPixel ) )
      {
        cv::Vec3b currentRgbPixel = colorImage.at<cv::Vec3b>(y,x);
        val = ( currentRgbPixel[0] + currentRgbPixel[1] + currentRgbPixel[2] ) / 6 + 64;
        colorImage.at<cv::Vec3b>(y,x) = cv::Vec3b(val,val,val);
      }
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
  default:
      fillValue = MASKED;
      //TRACE_WARNING ( "No mask operation provided, using \"erode operation\"!" );// TODO use ros
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
