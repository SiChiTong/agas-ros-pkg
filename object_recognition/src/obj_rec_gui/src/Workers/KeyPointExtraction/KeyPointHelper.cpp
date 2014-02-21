/*******************************************************************************
 *  KeyPointHelper.cpp
 *
 *  (C) 2007 AG Aktives Sehen <agas@uni-koblenz.de>
 *           Universitaet Koblenz-Landau
 *******************************************************************************/

#include "KeyPointHelper.h"

#include <algorithm>

#define THIS KeyPointHelper

void THIS::bBoxFilter( std::vector< KeyPoint > keyPointsIn, std::vector< KeyPoint >& keyPointsOut,
                              Box2D<> boundingBox )
{
  keyPointsOut.clear();
  keyPointsOut.reserve ( keyPointsIn.size() );
  for ( unsigned i = 0; i < keyPointsIn.size(); i++ )
  {
    if ( boundingBox.contains ( keyPointsIn[i].x, keyPointsIn[i].y ) )
    {
      keyPointsOut.push_back ( keyPointsIn[i] );
    }
  }
}

void THIS::imageBorderFilter( std::vector< KeyPoint > keyPointsIn, std::vector< KeyPoint >& keyPointsOut, int imgWidth, int imgHeight )
{
  const double border = 17;
  keyPointsOut.clear();
  for ( unsigned i = 0; i < keyPointsIn.size(); i++ )
  {
    // ignore points that are too close to the edge of the image
    const unsigned long border_size = static_cast<unsigned long> ( border * keyPointsIn[i].scale );
    Box2D<int> imageBBox ( 0, 0, imgWidth, imgHeight );
    imageBBox.shrink( border_size );

    if ( imageBBox.contains( keyPointsIn[i].x, keyPointsIn[i].y ) )
    {
      keyPointsOut.push_back ( keyPointsIn[i] );
    }
  }
}

void THIS::maskFilter( std::vector< KeyPoint > keyPointsIn, std::vector< KeyPoint >& keyPointsOut,
                              ImageMask& mask )
{
  keyPointsOut.clear();
  keyPointsOut.reserve ( keyPointsIn.size() );
  for ( unsigned i = 0; i < keyPointsIn.size(); i++ )
  {
    //if ( mask.findValue ( keyPointsIn[i].x, keyPointsIn[i].y, ImageMask::VISIBLE, 1 ) )
    if ( !mask.findValue ( keyPointsIn[i].x, keyPointsIn[i].y, ImageMask::MASKED, keyPointsIn[i].scale* 4.5/1.2 / 2.0 ) )
    {
      keyPointsOut.push_back ( keyPointsIn[i] );
    }
  }
}

bool strengthComp( KeyPoint& a, KeyPoint& b )
{
  return ( a.strength > b.strength );
}

void THIS::getStrongest( std::vector< KeyPoint > keyPointsIn, std::vector< KeyPoint >& keyPointsOut, unsigned numKeyPoints )
{
  if ( keyPointsIn.size() < numKeyPoints )
  {
    // TRACE_ERROR( "Not enough keypoints!" ); // TODO use ros
    numKeyPoints = keyPointsIn.size();
  }
  keyPointsOut.assign( numKeyPoints, KeyPoint() );
  partial_sort_copy( keyPointsIn.begin(), keyPointsIn.end(), keyPointsOut.begin(), keyPointsOut.end(), strengthComp );
}

void THIS::sortByStrength( std::vector< KeyPoint > keyPointsIn, std::vector< KeyPoint >& keyPointsOut )
{
  keyPointsOut.assign( keyPointsIn.size(), KeyPoint() );
  partial_sort_copy( keyPointsIn.begin(), keyPointsIn.end(), keyPointsOut.begin(), keyPointsOut.end(), strengthComp );
}

#undef THIS

