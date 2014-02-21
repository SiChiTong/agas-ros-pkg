/*******************************************************************************
 *  Circle2D.cpp
 *
 *  (C) 2006 AG Aktives Sehen <agas@uni-koblenz.de>
 *           Universitaet Koblenz-Landau
 *
 *******************************************************************************/

#include "Circle2D.h"
#include "vec2.h"

#include <math.h>

#define THIS Circle2D

using namespace std;

THIS::THIS() {
  m_Radius=0.0;
}

THIS::THIS(double x, double y, double radius) {
  m_Center=Point2D(x,y);
  m_Radius=radius;
}

THIS::THIS( Point2D center, double radius) {
  m_Center=center;
  m_Radius=radius;
}

THIS::~THIS() {
}

double THIS::x() const{
  return m_Center.x();
}

double THIS::y() const{
  return m_Center.y();
}

double THIS::radius() const
{
  return m_Radius;
}

Point2D THIS::center() const
{
  return m_Center;
}

void THIS::setX(double x) {
  m_Center.setX(x);
}

void THIS::setY(double y) {
  m_Center.setY(y);
}

void THIS::setCenter( Point2D center )
{
  m_Center=center;
}

void THIS::setRadius( double radius )
{
  m_Radius=radius;
}


vector<Point2D> THIS::vertices( int steps )
{
  vector<Point2D> myVertices;
  myVertices.reserve( steps+1 );
  for( float alpha=0.0; alpha<M_PI*2; alpha+=M_PI*2/float(steps) ) {
    myVertices.push_back( m_Center + CVec2( sin(alpha)*m_Radius, cos(alpha)*m_Radius ) );
  }
  myVertices.push_back( m_Center + CVec2( sin(M_PI*2)*m_Radius, cos(M_PI*2)*m_Radius ) );
  return myVertices;
}


#undef THIS
