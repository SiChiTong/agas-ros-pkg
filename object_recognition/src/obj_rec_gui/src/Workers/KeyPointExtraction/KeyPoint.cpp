/*******************************************************************************
 *  KeyPoint.cpp
 *
 *  (C) 2007 AG Aktives Sehen <agas@uni-koblenz.de>
 *           Universitaet Koblenz-Landau
 *
 *  Additional information:
 *  $Id: $
 *******************************************************************************/

#include "KeyPoint.h"

#include <string.h>
#include <math.h>

#include "Workers/Math/Math.h"
#include "Workers/Math/vec2.h"

#define THIS KeyPoint

using namespace std;

THIS::THIS ( float _x, float _y, float _scale, float _strength, int _sign, float _orientation, std::vector<double> _featureVector )
{
  x = _x;
  y = _y;
  scale = _scale;
  strength = _strength;
  sign = _sign;
  orientation = _orientation;
  featureVector = _featureVector;
  vectorLimits.push_back(0);
}

THIS::THIS ( float _x, float _y, float _scale, float _strength, int _sign )
{
  x = _x;
  y = _y;
  scale = _scale;
  strength = _strength;
  sign = _sign;
  vectorLimits.push_back(0);
}

THIS::THIS ( )
{
}

THIS::~THIS()
{
}

THIS::THIS ( const KeyPoint& other )
{
  operator= ( other );
}

THIS& THIS::operator= ( const KeyPoint & other )
{
  if ( this == &other )
  {
    return *this;
  }

  x = other.x;
  y = other.y;
  scale = other.scale;
  strength = other.strength;
  orientation = other.orientation;
  sign = other.sign;
  featureVector = other.featureVector;
  vectorLimits = other.vectorLimits;
  return *this;
}

void THIS::addDescriptor( vector<double> descriptor )
{
  featureVector.reserve( featureVector.size() + descriptor.size() );
  for ( unsigned i=0; i<descriptor.size(); i++ )
  {
    featureVector.push_back( descriptor[i] );
  }
//   TRACE_INFO( featureVector.size() );
  vectorLimits.push_back( featureVector.size() );
}

double THIS::squaredDistance ( const KeyPoint& other ) const
{
  if ( featureVector.size() != other.featureVector.size() )
  {
    return 99999;
  }

  double dist = 0;
  for ( unsigned n = 0; n < featureVector.size(); n++ )
  {
  double diff  = featureVector[n] - other.featureVector[n];
  dist += diff * diff;
  }

  /*
  double dist = 0;
  for ( unsigned n = 0; n < featureVector.size(); n++ )
  {
    double diff  = featureVector[n] - other.featureVector[n];
    dist += fabs(diff);
  }
  */

  return dist;
}


double THIS::squaredDistance ( const KeyPoint& other, double max ) const
{
  if ( featureVector.size() != other.featureVector.size() )
  {
    return 99999;
  }

  double dist = 0;
  for ( unsigned n1 = 0; n1 < featureVector.size(); n1+=4 )
  {
    double diff;

    //add block of 4 differences
    diff = featureVector[n1] - other.featureVector[n1];
    dist += diff * diff;
    diff = featureVector[n1+1] - other.featureVector[n1+1];
    dist += diff * diff;
    diff = featureVector[n1+2] - other.featureVector[n1+2];
    dist += diff * diff;
    diff = featureVector[n1+3] - other.featureVector[n1+3];
    dist += diff * diff;

    //check of distance is already too large
    if ( dist >= max )
    {
      return dist;
    }
  }

  return dist;
}


std::vector<Point2D> THIS::getBoundingBox() const
{
  std::vector<Point2D> boxVertices ( 5 );
  float r = scale * 10.0 * sqrt ( 2 );
  for ( int j = 0; j < 5; j++ )
  {
    float alpha = ( float ( j ) + 0.5 ) / 4.0 * 2.0 * M_PI + orientation;
    boxVertices[j] = Point2D ( x + r * sin ( alpha ), y + r * cos ( alpha ) );
  }
  return boxVertices;
}


std::vector<Point2D> THIS::getCircle() const
{
  float steps = 20;
  std::vector<Point2D> boxVertices ( steps+1 );
  float r = scale * 4.5/1.2 / 2.0;
  for ( int j = 0; j <= steps; j++ )
  {
    float alpha = ( float ( j ) + 0.5 ) / steps * 2.0 * M_PI + orientation;
    boxVertices[j] = Point2D ( x + r * sin ( alpha ), y + r * cos ( alpha ) );
  }
  return boxVertices;
}


std::vector<Point2D> THIS::getCenterArrow() const
{
  std::vector<Point2D> arrowVertices ( 6 );
  const float r1 = scale * 5.0;
  const float r3 = r1 + 6.0;
  const float width = 3.0;

  Point2D position ( x, y );

  arrowVertices[0] = position;
  arrowVertices[1] = position + Point2D ( r1 * sin ( orientation ), r1 * cos ( orientation ) );
  arrowVertices[2] = arrowVertices[1] + CVec2 ( width * sin ( orientation + ( M_PI / 2.0 ) ), width * cos ( orientation + ( M_PI / 2.0 ) ) );
  arrowVertices[3] = position + Point2D ( r3 * sin ( orientation ), r3 * cos ( orientation ) );
  arrowVertices[4] = arrowVertices[1] - CVec2 ( width * sin ( orientation + ( M_PI / 2.0 ) ), width * cos ( orientation + ( M_PI / 2.0 ) ) );
  arrowVertices[5] = arrowVertices[1];

  return arrowVertices;
}


// SERIALIZATION ///////////////////////////////////////////////////////

//void THIS::storer ( ExtendedOutStream& extStrm )
//{
//  extStrm << unsigned ( 10 );

//  extStrm << float ( x );
//  extStrm << float ( y );
//  extStrm << scale;
//  extStrm << strength;
//  extStrm << orientation;
//  extStrm << sign;
//  extStrm << unsigned(featureVector.size());

//  for ( unsigned i=0; i<featureVector.size(); i++ )
//  {
//    extStrm << float( featureVector[i] );
//  }
//}

//THIS::THIS ( ExtendedInStream& extStrm )
//{
//  unsigned version;
//  extStrm >> version;

//  extStrm >> x;
//  extStrm >> y;
//  extStrm >> scale;
//  extStrm >> strength;
//  extStrm >> orientation;
//  extStrm >> sign;
//  unsigned vectorLength;
//  extStrm >> vectorLength;

//  for ( unsigned i=0; i<vectorLength; i++ )
//  {
//    float v;
//    extStrm >> v;
//    featureVector[i] = v;
//  }
//}


std::string THIS::toASCII()
{
  ostringstream s;
  s << x << " " << y << " ";

  //ellipse parameters:
  //scale 1.2 corresponds to a filter size of 9x9
//   float r = scale/1.2*4.5;
  float r=scale*10;
  float a = 1/(r*r);
  float b = 0;
  float c = a;

  //TRACE_INFO( scale << " r " << r << " a " << a );


  s << a << " " << b << " " << c << " ";

  for ( unsigned i=0; i<featureVector.size(); i++ )
  {
    s << featureVector[i] << " ";
  }

  return s.str();
}

std::string THIS::toString()
{
  ostringstream s;
  s << x << " " << y << " "  << scale << " " << strength << " " << sign << " " << orientation << " ";

  for ( unsigned i=0; i<featureVector.size(); i++ )
  {
    s << featureVector[i] << " ";
  }

  return s.str();
}

double THIS::calcIntersection( const KeyPoint& other )
{
  //formula from http://mathworld.wolfram.com/Circle-CircleIntersection.html
  double R;
  double r;

  if ( scale > other.scale )
  {
    R=scale*10;
    r=other.scale*10;
  }
  else
  {
    R=other.scale*10;
    r=scale*10;
  }

  double d=sqrt( (other.x-x)*(other.x-x) + (other.y-y)*(other.y-y) );

  //no intersection
  if ( d >= R+r )
  {
    return 0;
  }

  //smaller lies in bigger circle: intersection is area of smaller circle
  if ( d+r < R )
  {
    return M_PI*r*r;
  }

//   TRACE_INFO( "d=" << d << " R=" << R << " r="<<r );

  double a = (d*d + r*r - R*R) / ( 2*d*r );
  double b = (d*d + R*R - r*r) / ( 2*d*R );
  double c = (-d+r+R) * (d+r-R) * (d-r+R) * (d+r+R);

  double A = r*r*acos( a ) + R*R*acos(b) - 0.5*sqrt(c);

//   TRACE_INFO( "a" << acos(a) << " b" << b << " c" << c << " A"<< A );

  return A;
}

double THIS::calcOverlap( const KeyPoint& other )
{
  double R=scale*10;
  double r=other.scale*10;

  double areaInter = calcIntersection( other );
  double areaUnion = M_PI*R*R + M_PI*r*r - areaInter;

  return areaInter / areaUnion;
}

#undef THIS
