/*******************************************************************************
 *  Vector3D.h
 *
 *  (C) 2006 AG Aktives Sehen <agas@uni-koblenz.de>
 *           Universitaet Koblenz-Landau
 *
 * Author: Frank Neuhaus
 *******************************************************************************/

#ifndef VEC3_H
#define VEC3_H

#include <math.h>
#include <assert.h>
#include <string>
#include "vec2.h"

class Vector3D
{
  public:

    Vector3D();
    Vector3D ( float x, float y, float z );
    Vector3D ( const CVec2& v, float z);

    Vector3D ( const Vector3D& v2 );

    Vector3D operator+ ( const Vector3D& vVector ) const;
    Vector3D& operator+= ( const Vector3D& vVector );
    Vector3D operator+() const;

    Vector3D operator- ( const Vector3D& vVector ) const;
    Vector3D& operator-= ( const Vector3D& vVector );
    Vector3D operator-() const;

    // Dot
    float operator* ( const Vector3D& vVector ) const;

    // Cross
    Vector3D operator^ ( const Vector3D& vVector2 ) const;

    // Scalar Mult
    Vector3D operator* ( const float num ) const;
    Vector3D& operator*= ( const float num );
    Vector3D operator/ ( float num ) const;
    Vector3D& operator/= ( const float num );

    bool operator < ( const Vector3D& vVec ) const;
    bool operator > ( const Vector3D& vVec ) const;
    bool operator== ( const Vector3D& v1 ) const;

    float operator [] ( const unsigned int i ) const;
    float& operator [] ( const unsigned int i );

    void set ( float fx, float fy, float fz );

    float x() const {
      return m_X;
    }

    float y() const {
      return m_Y;
    }

    float z() const {
      return m_Z;
    }

    void lerp ( const Vector3D& v1, const Vector3D& v2, float f );

    float magnitude() const;

    std::string toString( int precision=3, std::string name="" );

  private:
    float m_X, m_Y, m_Z;
};

#include "vec3_inl.h"


#endif
