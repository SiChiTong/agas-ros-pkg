/*******************************************************************************
 *  OrigSurfExtractor.cpp
 *
 *  (C) 2008 AG Aktives Sehen <agas@uni-koblenz.de>
 *           Universitaet Koblenz-Landau
 *
 *  $Id: $
 *
 *******************************************************************************/

#include "OrigSurfExtractor.h"


#include <cmath>
#include <sstream>
#include <float.h>

#include "Architecture/Config/Config.h"
#include "Architecture/Tracer/Tracer.h"
#include "Architecture/Singleton/Clock.h"

#include "Workers/Puma2/ColorToGrayOperator.h"
#include "Workers/SimpleTimer/SimpleTimer.h"

#include <surf/surflib.h>


#define THIS OrigSurfExtractor

#define FULL_DEBUG

using namespace std;


THIS::THIS( )
{
  m_IntegralImage = 0;
}


THIS::~THIS()
{
  TRACE_SYSTEMINFO ( "Deleting member variables.." )
  
  //TODO
  if(m_IntegralImage)
  {
    delete m_IntegralImage;
  }
}



THIS::THIS ( const THIS& other ) : SurfExtractorBase ( other )
{
}


THIS& THIS::operator= ( const THIS & other )
{
  SurfExtractorBase::operator= ( other );
  return *this;
}


std::string THIS::getName() {
  if ( m_Extended )
  {
    return "SURF-128";
  }
  else
  {
    return "SURF-64";
  }
}


void THIS::setImage ( const puma2::GrayLevelImage8 &pumaImage )
{
  int width = pumaImage.getWidth();
  int height = pumaImage.getHeight();
  
  surf::Image* surfImage = new surf::Image ( width, height );
  
  double** surfRows = surfImage->getPixels();
  unsigned char** pumaRows = const_cast<puma2::GrayLevelImage8&> ( pumaImage ).unsafeRowPointerArray();
  
  for ( int y = 0; y < height; y++ )
  {
    unsigned char* pumaRow = pumaRows[y];
    double* surfRow = surfRows[y];
    for ( int x = 0; x < width; x++ )
    {
      surfRow[x] = double ( pumaRow[x] ) / 255.0;
    }
  }
  
  delete m_IntegralImage;
  m_IntegralImage = new surf::Image ( surfImage, false );
  
  //TODO
  delete surfImage;
}


void THIS::setImage ( const puma2::ColorImageRGB8 &pumaImage )
{
  puma2::GrayLevelImage8 pumaImageY;
  puma2::ColorToGrayOperator<puma2::ColorImageRGB8,puma2::GrayLevelImage8>( pumaImage, pumaImageY );
  setImage( pumaImageY );
}


void THIS::getKeyPoints ( std::vector< KeyPoint >& keyPoints )
{
  vector<surf::Ipoint> ipoints;
  
  if ( !m_IntegralImage )
  {
    TRACE_ERROR ( "No image was set yet." );
    return;
  }
  
  ipoints.reserve ( 1000 );
  
  // Extract keypoints with Fast Hessian
  surf::FastHessian fh ( m_IntegralImage, ipoints, m_BlobResponseThreshold, false, m_InitLobeSize*3, m_SamplingStep, m_Octaves );
  fh.getInterestPoints();
  
  surf::Surf surf ( m_IntegralImage, false, !m_RotationInvariance, m_Extended, m_IndexSize );
 
  keyPoints.resize( ipoints.size() );
  
  int descLength = surf.getVectLength();
  
  for ( unsigned i = 0; i < ipoints.size(); i++ )
  {
    surf.setIpoint ( &ipoints[i] );
    surf.assignOrientation();
    surf.makeDescriptor();
    
    keyPoints[i].x = ipoints[i].x;
    keyPoints[i].y = ipoints[i].y;
    keyPoints[i].scale = ipoints[i].scale;
    keyPoints[i].orientation = ipoints[i].ori;
    keyPoints[i].strength = ipoints[i].strength;
    keyPoints[i].sign = ipoints[i].laplace;
    keyPoints[i].copyDescriptor( ipoints[i].ivec, descLength );
  }
  
  
  
}

