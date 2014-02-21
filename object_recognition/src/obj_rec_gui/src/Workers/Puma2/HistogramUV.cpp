/*******************************************************************************
 *  HistogramUV.cpp
 *
 *  (C) 2007 AG Aktives Sehen <agas@uni-koblenz.de>
 *           Universitaet Koblenz-Landau
 *
 *  Additional information:
 *  $Id: HistogramUV.cpp 24234 2008-04-12 17:32:23Z dgossow $
 *
 *******************************************************************************/

#include <stdlib.h>

#include "HistogramUV.h"
#include "../../Workers/Puma2/Y8UV8ToRGB8Operator.h"

#define THIS HistogramUV

#include <iostream>
#include <limits>
#include <math.h>
#include <sstream>

using namespace puma2;
using namespace std;

THIS::THIS( unsigned binSize )
{
  m_BinSize = binSize;
  if ( 256%m_BinSize == 0 ) {
    m_NumBins=256/m_BinSize;
  } else {
    m_NumBins=256/binSize+1;
  }
  m_Matrix.resize( m_NumBins, m_NumBins );
  m_Data = ((EntryT**)(m_Matrix)) [0];
  m_DataLength = m_NumBins*m_NumBins;
  clear();
}

THIS::THIS( )
{
  m_BinSize = 0;
}


THIS::~THIS() {}

THIS::THIS( const HistogramUV& other )
{
  m_BinSize=0;
  operator=(other);
}

HistogramUV& THIS::operator=( const HistogramUV& other )
{
  if (other.m_BinSize == 0) {
    m_BinSize=0;
    return *this;
  }
  if (m_BinSize != 0 && m_BinSize != other.m_BinSize) {
    std::ostringstream stream;
    stream << "Bin sizes do not match ( " << m_BinSize << " != " << other.m_BinSize << " )";
    // TRACE_ERROR(stream.str()); // TODO use ros
    return *this;
  }
  m_BinSize=other.m_BinSize;
  m_NumBins=other.m_NumBins;
  m_Matrix.resize( m_NumBins, m_NumBins );
  m_Data = ((EntryT**)(m_Matrix)) [0];
  m_DataLength = m_NumBins*m_NumBins;
  memcpy( m_Data, other.m_Data, m_DataLength*sizeof(EntryT) );
  return *this;
}

// TODO
//THIS::THIS( ExtendedInStream& extStrm )
//{
//  short version;
//  extStrm >> version;
//  extStrm >> m_BinSize;
//  extStrm >> m_NumBins;
//  extStrm >> m_Matrix;
//  m_Data = ((EntryT**)(m_Matrix)) [0];
//  m_DataLength = m_NumBins*m_NumBins;
//}

//void THIS::storer( ExtendedOutStream& extStrm )
//{
//  short version=10;
//  extStrm << version;
//  extStrm << m_BinSize;
//  extStrm << m_NumBins;
//  extStrm << m_Matrix;
//}

void THIS::printOn( ostream& strm )
{
  strm << "Bin size: " << m_BinSize << endl;
  strm << "Histogram size: " << m_NumBins << " x " << m_NumBins << endl;
  strm << "Mean value: " << getMeanValue() << endl;
  strm << "Max value: " << getMaxValue() << endl;
  strm << "Deviation: " << getDeviation();
}

bool THIS::checkInit() const {
  if (m_BinSize == 0) {
    //TRACE_ERROR("This histogram has not been initialized (bin size=0)."); // TODO use ros
    return false;
  }
  return true;
}

void THIS::clear()
{
  if (!checkInit()) { return; }
  memset( m_Data, 0, m_DataLength*sizeof(EntryT) );
}


int THIS::correct( int val, int y ) const
{
  int result= ( val - 128 ) * 64 / y + 128;
  if (result<0) { result=0; }
  if (result>255) { result=255; }
  result/=m_BinSize;
  return result;
}

void THIS::addImage( ColorImageUV8 &imageUV, GrayLevelImage8 &imageY, unsigned minY, unsigned maxY )
{
  Box2D<int> bBox(0, 0, imageUV.getWidth()-1, imageUV.getHeight()-1);
  addImage(imageUV, imageY, bBox, minY, maxY);
}

void THIS::addImage( ColorImageUV8 &imageUV, GrayLevelImage8 &imageY, Box2D<int> bBox, unsigned minY, unsigned maxY )
{
  if (!checkInit()) { return; }

  if (minY<1) { minY=1; }
  if (maxY>254) { maxY=254; }

  ColorImageUV8::PixelType* currentUvPixel;
  unsigned char* currentYPixel;

  for(int y = bBox.minY(); y <= bBox.maxY(); y++)
  for(int x = bBox.minX(); x <= bBox.maxX(); x++)
  {
    currentUvPixel=&(imageUV[y][x]);
    currentYPixel = &(imageY[y][x]);
    if( ( (*currentUvPixel)[0] != 255) && (*currentYPixel >= minY) && (*currentYPixel <= maxY) )
    {
      //center around 0, divide by y, uncenter, divide by bin size
      m_Matrix[ correct( (*currentUvPixel)[0], (*currentYPixel) ) ][ correct( (*currentUvPixel)[1], (*currentYPixel) ) ] ++;
    }
    currentYPixel++;
    currentUvPixel++;
  }

//  m_Matrix[m_NumBins-1][m_NumBins-1]=0;
}


void THIS::substract( float value )
{
  for (unsigned i=0;i<m_DataLength;i++)
  {
    m_Data[i] -= value;
    if ( m_Data[i] < 0) { m_Data[i]=0; }
  }
}


void THIS::divideBy( const HistogramUV& divident )
{
  if (!checkInit()) { return; }

  if (m_BinSize != divident.getBinSize() ) {
    // TRACE_ERROR("Bin sizes do not match."); // TODO use ros
    return;
  }

  const EntryT* dividentData=divident.getData();

  for (unsigned i=0;i<m_DataLength;i++)
  {
    m_Data[i] /= dividentData[i]+1;
  }
}

void THIS::add( const HistogramUV& other )
{
  if (!checkInit()) { return; }

  if (m_BinSize != other.getBinSize() ) {
    // TRACE_ERROR("Bin sizes do not match."); // TODO use ros
    return;
  }

  const EntryT* dividentData=other.getData();

  for (unsigned i=0;i<m_DataLength;i++)
  {
    m_Data[i] += dividentData[i];
  }
}


void THIS::applyThreshold( float thresholdFactor, float dilationRadius )
{
  float threshold = thresholdFactor * float( getMeanValue() );
  ostringstream stream;
  stream << "Binarizing histogram. Threshold: " << threshold << " Dilation Radius: " << dilationRadius;
  // TRACE_SYSTEMINFO( stream.str() ) // TODO use ros
  ImageMask dataCopyMask( m_NumBins, m_NumBins );
  unsigned char* dataCopy=dataCopyMask.getData();
  for (unsigned i=0;i<m_DataLength;i++)
  {
    if (m_Data[i] < threshold) { dataCopy[i]=0; }
    else { dataCopy[i]=255; }
  }
  dataCopyMask.dilate( dilationRadius );
  dataCopy=dataCopyMask.getData();
  for (unsigned i=0;i<m_DataLength;i++)
  {
    m_Data[i]=dataCopy[i];
  }
  //note: the ImageMask deletes dataCopy
}


void THIS::normalizeMax()
{
  EntryT maxValue = getMaxValue();

  if(maxValue == 0)
  {
    // TRACE_SYSTEMINFO( "Histogram is zero. Nothing to do." ) // TODO use ros
    return;
  }

  for(unsigned i=0; i < m_DataLength; i++)
  {
    m_Data[i] /= maxValue;
  }
}

void THIS::clearCenter(unsigned int range)
{
  unsigned realRange = range/m_BinSize;
  if (realRange > m_NumBins/2)
  {
   realRange = m_NumBins/2;
  }
  for (unsigned y = m_NumBins/2 - range; y < m_NumBins/2 + range; y++)
    for (unsigned x = m_NumBins/2 - range; x < m_NumBins/2 + range; x++)
    {
      m_Matrix[y][x] = 0;
    }
}

float THIS::getDeviation()
{
  float weightedDistanceSum = 0;
  float totalSum = 0;

  for(int y = 1; y < int(m_NumBins); y++ )
    for(int x = 1; x < int(m_NumBins); x++ )
    {
      if(m_Matrix[y][x] != 0)
      {
        int squareDistance = ( abs(y-128) * abs(y-128) ) + ( abs(x-128) * abs(x-128) );
        float distance = sqrt(squareDistance);
        weightedDistanceSum += float(m_Matrix[y][x]) * distance;
        totalSum+=m_Matrix[y][x];
      }
    }

  return float(weightedDistanceSum) / float(totalSum) / sqrt(2.0);
}

ImageMask* THIS::getMask( ColorImageUV8 &imageUV, GrayLevelImage8 &imageY, unsigned minY, unsigned maxY ) const
{
  if (!checkInit()) { return 0; }

  if (minY<1) { minY=1; }
  if (maxY>254) { maxY=254; }

  unsigned width = imageUV.getWidth();
  unsigned height = imageUV.getHeight();

  unsigned char* mask = new unsigned char[ width*height ];
  memset( mask, 0, width*height );

  ColorImageUV8::PixelType* currentUvPixel=imageUV.unsafeRowPointerArray()[0];
  unsigned char* currentYPixel=imageY.unsafeRowPointerArray()[0];
  unsigned char* currentMaskPixel=mask;
  unsigned u,v;

  for(unsigned y=0; y < height; y++)
  {
    for(unsigned x=0; x < width; x++)
    {
      u=(*currentUvPixel)[0]/m_BinSize;
      v=(*currentUvPixel)[1]/m_BinSize;
      if (  ( ( *currentYPixel < maxY ) && ( *currentYPixel > minY ) ) &&
            ( m_Matrix[ correct( (*currentUvPixel)[0], (*currentYPixel) ) ][ correct( (*currentUvPixel)[1], (*currentYPixel) ) ] ) )
      {
        *currentMaskPixel = 255;
      }

      currentUvPixel++;
      currentYPixel++;
      currentMaskPixel++;
    }
  }

  ImageMask* maskObj=new ImageMask( width, height, mask );

  return maskObj;
}


void THIS::getImage( ColorImageRGB8& imageRGB, float exponent ) const
{
  if (!checkInit()) { return; }

  if ( (imageRGB.getWidth() == 0) && (imageRGB.getHeight() == 0) )
  {
    imageRGB.resize(m_NumBins,m_NumBins);
  }

  if ( (unsigned(imageRGB.getWidth()) != m_NumBins) != (unsigned(imageRGB.getHeight()) != m_NumBins) )
  {
    // TRACE_ERROR("Wrong image size."); // TODO use ros
    return;
  }

  //Calculate normalized values
  EntryT maxValue=getMaxValue();
  EntryT* normalizedData=new EntryT[m_DataLength];

  if ( exponent == 1.0 )
  {
    for (unsigned i=0;i<m_DataLength;i++) {
      normalizedData[i] = m_Data[i] / maxValue;
    }
  } else
  {
    for (unsigned i=0;i<m_DataLength;i++) {
      normalizedData[i] = pow( m_Data[i] / maxValue, exponent );
    }
  }

  //Create UV image map
  ColorImageUV8 imageUV(m_NumBins,m_NumBins);
  GrayLevelImage8 imageY(m_NumBins,m_NumBins);

  ColorImageUV8::PixelType* currentUvPixel=imageUV.unsafeRowPointerArray()[0];

  memset( imageY.unsafeRowPointerArray()[0], 196, m_DataLength );

  for (unsigned u=0; u < m_NumBins; u++)
  for (unsigned v=0; v < m_NumBins; v++)
  {
    (*currentUvPixel)[0]=u*m_BinSize;
    (*currentUvPixel)[1]=v*m_BinSize;
    currentUvPixel++;
  }
  Y8UV8ToRGB8Operator( imageY, imageUV, imageRGB );

  //Multiply by histogram values
  ColorImageRGB8::PixelType* currentRgbPixel=imageRGB.unsafeRowPointerArray()[0];
  EntryT* currentHistogramEntry=normalizedData;
  EntryT histGamma;

  for (unsigned u=0; u < m_NumBins; u++)
   for (unsigned v=0; v < m_NumBins; v++)
  {
    histGamma=*currentHistogramEntry;

    (*currentRgbPixel)[0] *= histGamma;
    (*currentRgbPixel)[1] *= histGamma;
    (*currentRgbPixel)[2] *= histGamma;

/*    if ( histGamma != 0.0 )
    {
      white=unsigned( (1.0-histGamma)*255.0 );
      (*currentRgbPixel)[0]= white + unsigned( float( (*currentRgbPixel)[0] ) * histGamma );
      (*currentRgbPixel)[1]= white + unsigned( float( (*currentRgbPixel)[1] ) * histGamma );
      (*currentRgbPixel)[2]= white + unsigned( float( (*currentRgbPixel)[2] ) * histGamma );
    }
    else
    {
      (*currentRgbPixel)[0]= 0;
      (*currentRgbPixel)[1]= 0;
      (*currentRgbPixel)[2]= 0;
    }*/
    currentRgbPixel++;
    currentHistogramEntry++;
  }

  delete[] normalizedData;

}


EntryT THIS::distance(HistogramUV& other) const
{
  if(m_NumBins != other.m_NumBins)
  {
    // TRACE_ERROR( "Size mismatch."); // TODO use ros
    return 0;
  }

  EntryT sumDiff=0;
  EntryT sum=0;
  EntryT thisMean=getMeanValue();
  EntryT otherMean=other.getMeanValue();

  //calculate standard deviation of normalized histograms
  for(unsigned i = 0; i < m_DataLength; i++)
  {
    EntryT thisNorm=m_Data[i] / thisMean;
    EntryT otherNorm=other.m_Data[i] / otherMean;

    EntryT diff= thisNorm-otherNorm;

    sum+= thisNorm+otherNorm;
    sumDiff+= diff*diff;
  }

  return sumDiff/sum;
}

EntryT THIS::getMeanValue() const
{
  if (!checkInit()) { return 0; }

  EntryT sum=0;
  for (unsigned i=0;i<m_DataLength;i++)
  {
    sum+=m_Data[i];
  }
  EntryT mean = sum / m_DataLength;
  return mean;
}


EntryT THIS::getMaxValue() const
{
  if (!checkInit()) { return 0; }

  EntryT max=0;
  for (unsigned i=0;i<m_DataLength;i++)
  {
    if (m_Data[i] > max) { max=m_Data[i]; }
  }
  ostringstream stream;
  stream << "Maximum " << max;
  // TRACE_SYSTEMINFO( stream.str() ) //// TODO use ros
  return max;
}

#undef THIS
