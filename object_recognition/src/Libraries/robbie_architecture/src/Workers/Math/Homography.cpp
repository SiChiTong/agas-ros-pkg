/*******************************************************************************
 *  Homography.cpp
 *
 *  (C) 2007 AG Aktives Sehen <agas@uni-koblenz.de>
 *           Universitaet Koblenz-Landau
 *******************************************************************************/

#include "Homography.h"

#include <math.h>
#include <string.h>
#include <sstream>

#define THIS Homography

THIS::THIS ( double homMat[9] )
{
  memcpy( m_HomMat, homMat, 9*sizeof(double) );
}

THIS::THIS ( const THIS& other )
{
  memcpy( m_HomMat, other.m_HomMat, 9*sizeof(double) );
}

THIS& THIS::operator=( const Homography& other )
{
  memcpy( m_HomMat, other.m_HomMat, 9*sizeof(double) );
  return *this;
}

Point2D THIS::transform ( Point2D point2 )
{
  if ( !point2.isValid() )
  {
    return point2;
  }
  else
  {
    double x = point2.x();
    double y = point2.y();
    double Z = 1. / ( m_HomMat[6] * x + m_HomMat[7] * y + m_HomMat[8] );
    double X = ( m_HomMat[0] * x + m_HomMat[1] * y + m_HomMat[2] ) * Z;
    double Y = ( m_HomMat[3] * x + m_HomMat[4] * y + m_HomMat[5] ) * Z;
    return Point2D( X, Y );
  }
}

void THIS::transform ( std::vector<Point2D>& points2, std::vector<Point2D> &projPoints )
{
  projPoints.reserve( points2.size() );

  // Translate src_corners to dst_corners using homography
  for ( unsigned i = 0; i < points2.size(); i++ )
  {
    if ( !points2[i].isValid() )
    {
      projPoints.push_back( points2[i] );
    }
    else
    {
      double x = points2[i].x();
      double y = points2[i].y();
      double Z = 1. / ( m_HomMat[6] * x + m_HomMat[7] * y + m_HomMat[8] );
      double X = ( m_HomMat[0] * x + m_HomMat[1] * y + m_HomMat[2] ) * Z;
      double Y = ( m_HomMat[3] * x + m_HomMat[4] * y + m_HomMat[5] ) * Z;
      projPoints.push_back( Point2D( X, Y ) );
    }
  }
}

bool THIS::checkValidity ( std::vector<Point2D>& points2 )
{
  // Translate src_corners to dst_corners using homography
  for ( unsigned i = 0; i < points2.size(); i++ )
  {
    if ( !points2[i].isValid() )
    {
      continue;
    }
    else
    {
      double x = points2[i].x();
      double y = points2[i].y();
      double Z = 1. / ( m_HomMat[6] * x + m_HomMat[7] * y + m_HomMat[8] );
      if ( Z < 0 )
      {
        return false;
      }
    }
  }
  return true;
}

std::string THIS::toString()
{
  std::ostringstream s;
  for ( int j=0; j< 3; j++ )
  {
    for ( int i=0; i< 3; i++ )
    {
      s << m_HomMat[i+3*j] << " ";
    }
    s << std::endl;
  }
  return s.str();
}

#undef THIS
