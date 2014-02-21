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

// #include "Architecture/Tracer/Tracer.h" // TODO

#include "../../Workers/Puma2/ColorToGrayOperator.h"

#include "../../Workers/ParallelSurf/Image.h"
#include "../../Workers/ParallelSurf/KeyPointDetector.h"
#include "../../Workers/ParallelSurf/KeyPointDescriptor.h"
#include "../../Workers/ParallelSurf/KeyPoint.h"

#define THIS ParallelSurfExtractor

// #define FULL_DEBUG // TODO kann das weg oder ist das kunst?


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
  // TRACE_SYSTEMINFO ( "Deleting member variables.." ); // TODO use ros
  delete m_IntegralImage;
//  delete m_ThreadPool; // TODO maybe this should be executed ?? Thread bug ??
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


void THIS::setImage ( const puma2::GrayLevelImage8 &pumaImage )
{
  delete m_IntegralImage;
  m_IntegralImage = new parallelsurf::Image( pumaImage.unsafeRowPointerArray(), pumaImage.getWidth(), pumaImage.getHeight() );
}


void THIS::setImage ( const puma2::ColorImageRGB8 &pumaImage )
{
  puma2::GrayLevelImage8 pumaImageY;
  puma2::ColorToGrayOperator<puma2::ColorImageRGB8,puma2::GrayLevelImage8>( pumaImage, pumaImageY );

  delete m_IntegralImage;
  m_IntegralImage = new parallelsurf::Image( (const unsigned char**)pumaImageY.unsafeRowPointerArray(), pumaImage.getWidth(), pumaImage.getHeight() );
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

