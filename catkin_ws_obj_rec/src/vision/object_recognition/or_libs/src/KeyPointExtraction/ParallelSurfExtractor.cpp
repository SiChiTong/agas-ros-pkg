/*******************************************************************************
 *  SurfExtractor.cpp
 *
 *  (C) 2008 AG Aktives Sehen <agas@uni-koblenz.de>
 *           Universitaet Koblenz-Landau
 *
 *  $Id: $
 *
 *******************************************************************************/

#include "ParallelSurfExtractor.h"


#include <cmath>
#include <sstream>
#include <float.h>

#include "../ParallelSurf/Image.h"
#include "../ParallelSurf/KeyPointDetector.h"
#include "../ParallelSurf/KeyPointDescriptor.h"
#include "../ParallelSurf/KeyPoint.h"

#define THIS ParallelSurfExtractor

#define FULL_DEBUG

using namespace std;


THIS::THIS( int numThreads )
{
  m_IntegralImage = 0;
  
  if ( numThreads != 0 )
  {
    m_ThreadPool = new boost::threadpool::pool( numThreads );
  }  
  else
  {
    m_ThreadPool = new boost::threadpool::pool( boost::thread::hardware_concurrency() );
  }
}


THIS::~THIS()
{
  ROS_DEBUG_STREAM ( "Deleting member variables.." );
  delete m_IntegralImage;
//  delete m_ThreadPool;
}



THIS::THIS ( const THIS& other ) : SurfExtractorBase ( other )
{
}


THIS& THIS::operator= ( const THIS & other )
{
  SurfExtractorBase::operator=(other);
  return *this;
}



std::string THIS::getName()
{
  std::ostringstream s;
  s << "ParallelSURF (" << m_ThreadPool->size() << " threads)";
  return s.str();
}


void THIS::setImage ( const cv::Mat &image )
{
    cv::Mat dest;
    // if it is a color image convert it to grayscale
    if(image.type() == CV_8UC3)
    {
        cvCvtColor( &image, &dest, CV_RGB2GRAY );
    }
    else
    {
        dest = image;
    }

    unsigned char *data[dest.rows];
    for(int i = 0; i < dest.rows; i++)
    {
        data[i] =  dest.row(i).data;
    }

  delete m_IntegralImage;
  m_IntegralImage = new parallelsurf::Image( (const unsigned char**) data, dest.cols, dest.rows );
}

void THIS::getKeyPoints ( std::vector< KeyPoint >& keyPoints )
{
  
  std::vector<parallelsurf::KeyPoint> panoKeyPoints;

  parallelsurf::KeyPointDetector detector( *m_IntegralImage, *m_ThreadPool );
  parallelsurf::KeyPointDescriptor descriptor( *m_IntegralImage, *m_ThreadPool, m_Extended );
  
  //insertor inserts into panoKeyPoints
  KeyPointVectInsertor insertor( panoKeyPoints );
  
  //Detect
  detector.setMaxOctaves( m_Octaves );
  detector.setScoreThreshold( m_BlobResponseThreshold );
  
  detector.detectKeyPoints( insertor );
  
  //Compute orientation & descriptor
  
  if ( m_RotationInvariance)
  {
    descriptor.assignOrientations( panoKeyPoints.begin(), panoKeyPoints.end() );
  }
  else
  {
    for ( unsigned i=0; i<panoKeyPoints.size(); i++ )
    {
      panoKeyPoints[i]._ori=0;
    }
  }
  
  descriptor.makeDescriptors( panoKeyPoints.begin(), panoKeyPoints.end() );
  
  //Copy results
  keyPoints.resize( panoKeyPoints.size() );
  
  for ( unsigned i=0; i<panoKeyPoints.size(); i++ )
  {
    keyPoints[i].x = panoKeyPoints[i]._x;
    keyPoints[i].y = panoKeyPoints[i]._y;
    keyPoints[i].scale = panoKeyPoints[i]._scale;
    keyPoints[i].orientation = panoKeyPoints[i]._ori;
    keyPoints[i].strength = panoKeyPoints[i]._score;
    keyPoints[i].sign = panoKeyPoints[i]._trace;
    keyPoints[i].featureVector = panoKeyPoints[i]._vec;
  }
  
}

