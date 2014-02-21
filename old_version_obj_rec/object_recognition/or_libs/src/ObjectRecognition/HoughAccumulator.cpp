/*******************************************************************************
 *  HoughAccumulator.cpp
 *
 *  (C) 2007 AG Aktives Sehen <agas@uni-koblenz.de>
 *           Universitaet Koblenz-Landau
 *******************************************************************************/

#include "HoughAccumulator.h"
#include "Architecture/Config/Config.h"

#include <sstream>
#include <algorithm>    // for max_element

#define THIS HoughAccumulator

THIS::THIS()
{
  m_ScaleBins = Config::getInt( "ObjectRecognition.HoughClustering.iScaleBins" );
  m_OrientationBins = Config::getInt( "ObjectRecognition.HoughClustering.iOrientationBins" );
  m_XLocationBins = Config::getInt( "ObjectRecognition.HoughClustering.iXLocationBins" );
  m_YLocationBins = Config::getInt( "ObjectRecognition.HoughClustering.iYLocationBins" );

  m_AccumulatorSize = m_ScaleBins*m_OrientationBins*m_XLocationBins*m_YLocationBins;
  m_AccumulatorArray = new std::list< KeyPointMatch >[m_AccumulatorSize];

  //Initialize accumulator
  for (unsigned i = 0; i < m_AccumulatorSize; i++)
  {
    m_AccumulatorArray[i].clear();
  }
}

std::vector< std::list< KeyPointMatch > > THIS::getClusteredMatches()
{
  unsigned int threshold = Config::getInt( "ObjectRecognition.HoughClustering.iMinMatchNumber" );

  std::vector< std::list< KeyPointMatch> > newMatches;

  for(unsigned int i=0;i<m_AccumulatorSize;++i)
  {
    std::list< KeyPointMatch > entry = m_AccumulatorArray[i];
    if(entry.size()>=threshold)
    {
      m_Log << "(+";
      newMatches.push_back(entry);
    }
    else
    {
      m_Log << "(-";
      m_AccumulatorArray[i].clear();
    }
    m_Log << "->" << i << "/" << entry.size() << ")";
  }

  //sort matches in descending order
  std::sort(newMatches.begin(), newMatches.end(), compareMatchList());

  return newMatches;
}

std::vector< std::list< KeyPointMatch > > THIS::getMaximumMatches()
{
    unsigned int threshold = Config::getInt( "ObjectRecognition.HoughClustering.iMinMatchNumber" );

    std::vector< std::list< KeyPointMatch> > newMatches;

    for(int s=0;s<m_ScaleBins;++s)
    {
        for(int r=0;r<m_OrientationBins;++r)
        {
            for(int x=0;x<m_XLocationBins;++x)
            {
                for(int y=0;y<m_YLocationBins;++y)
                {
                    std::list< KeyPointMatch > entry = m_AccumulatorArray[getIndex(s,r,x,y)];
                    bool isMax = true;
                    if(entry.size()>=threshold)
                    {
                        //Check if neighbors have less entries
                        for(int sN=s-1;sN>=0 && sN<s+1 && sN<m_ScaleBins;++sN)
                        {
                            for(int rN=r-1;rN>=0 && rN<r+1 && rN<m_OrientationBins && isMax;++rN)
                            {
                                for(int xN=x-1;xN>=0 && xN<x+1 && xN<m_XLocationBins && isMax;++xN)
                                {
                                    for(int yN=y-1;yN>=0 && yN<y+1 && yN<m_YLocationBins && isMax;++yN)
                                    {
                                        //Don't check against itself
                                        if(sN!=s && rN!=r && xN !=x && yN!=y)
                                        {
                                            if(m_AccumulatorArray[getIndex(sN,rN,xN,yN)].size()>=entry.size())
                                            {
                                                isMax=false;
                                            }
                                        }
                                    }
                                }
                            }
                        }
                        if( isMax)
                        {
                            m_Log << "(+";
                            newMatches.push_back(entry);
                        }
                        else
                        {
                            m_Log << "(-";
                            m_AccumulatorArray[getIndex(s,r,x,y)].clear();
                        }
                    }
                    else
                    {
                        m_Log << "(-";
                        m_AccumulatorArray[getIndex(s,r,x,y)].clear();
                    }
                    m_Log << "->" << getIndex(s,r,x,y) << "/" << entry.size() << ")";
                }
            }
        }
    }
    //sort matches in descending order
    std::sort(newMatches.begin(), newMatches.end(), compareMatchList());
    return newMatches;
}

unsigned int THIS::getIndex(int scaleIndex, int orientationIndex, int xIndex, int yIndex)
{
    return scaleIndex
          + orientationIndex*m_ScaleBins
          + xIndex*(m_ScaleBins*m_OrientationBins)
          + yIndex*(m_ScaleBins*m_OrientationBins*m_XLocationBins);
}

bool THIS::incrAccumulatorValue(int scaleIndex, int orientationIndex, int xIndex, int yIndex, KeyPointMatch match)
{
  if(verifyAccumulatorIndex(scaleIndex,orientationIndex,xIndex,yIndex))
  {
    m_AccumulatorArray[getIndex(scaleIndex,orientationIndex,xIndex,yIndex)].push_back(match);
    return true;
  }
  else
  {
    return false;
  }
}

bool THIS::getAccumulatorValue(int scaleIndex, int orientationIndex, int xIndex, int yIndex, unsigned int& value)
{
  if(verifyAccumulatorIndex(scaleIndex,orientationIndex,xIndex,yIndex))
  {
    value = m_AccumulatorArray[getIndex(scaleIndex,orientationIndex,xIndex,yIndex)].size();
    return true;
  }
  else
  {
    value = 0;
    return false;
  }
}

void THIS::resetAccumulator()
{
  for (unsigned i = 0; i < m_AccumulatorSize; i++)
  {
    m_AccumulatorArray[i].clear();
  }
}

bool THIS::verifyAccumulatorIndex(int scale, int orientation, int xLocation, int yLocation)
{

  if(scale<m_ScaleBins && orientation<m_OrientationBins && xLocation<m_XLocationBins && yLocation<m_YLocationBins &&
     scale>=0 && orientation>=0 && xLocation>=0 && yLocation>=0)
  {
    return true;
  }
  else
  {
    return false;
  }
}

float THIS::getVariance()
{
  float sum = 0.0;
  float sumSquare = 0.0;
  int n =  m_AccumulatorArray->size();

  for(int i=0;i<n;++i)
  {
  int features =  m_AccumulatorArray[i].size();
  sumSquare+=(features*features);
  sum+=features;
  }

  return (sumSquare/n-((sum/n)*(sum/n)));
}

unsigned int THIS::getMaxAccumulatorValue()
{
  float max=0;
  for (unsigned i = 0; i < m_AccumulatorSize; i++)
  {
    int value = m_AccumulatorArray[i].size();
    if ( value > max )
    {
      max = value;
    }
  }

  return max;
}

void THIS::getImage( cv::Mat& target )
{
  /*
    large bins: x and y-position
    small bins: y -> orientation, x -> scale
    pixel grey value: number of entries in bin depending on maximum value
   */

  float norm = 255.0 / getMaxAccumulatorValue();

  int w = (m_ScaleBins+1)*m_XLocationBins;
  int h = (m_OrientationBins+1)*m_YLocationBins;
  target.create( h, w, CV_8UC3 );

  for ( int y=0; y<h; y++ )
  {
    for ( int x=0; x<w; x++ )
    {
      int scaleIndex = x % (m_ScaleBins+1);
      int orientationIndex = y % (m_OrientationBins+1);
      if ( ( scaleIndex >= m_ScaleBins ) || ( orientationIndex >= m_OrientationBins ) )
      {
        target.at<cv::Vec3b>(y,x) = cv::Vec3b(0,0,255);
        continue;
      }
      int xIndex = x / (m_ScaleBins+1);
      int yIndex = y / (m_OrientationBins+1);

      unsigned int histValue;

      getAccumulatorValue(scaleIndex, orientationIndex, xIndex, yIndex, histValue);

      if ( histValue == 0 )
      {
        target.at<cv::Vec3b>(y,x) = cv::Vec3b(0,0,0);
      }
      else
      {
          target.at<cv::Vec3b>(y,x) = cv::Vec3b(histValue * norm,histValue * norm,histValue * norm);
      }
    }
  }
}

THIS::~THIS()
{
  delete[] m_AccumulatorArray;
}

#undef THIS
