/*******************************************************************************
 *  vec3_inl.h
 *
 *  (C) 2006 AG Aktives Sehen <agas@uni-koblenz.de>
 *           Universitaet Koblenz-Landau
 *
 * Author: Frank Neuhaus
 *******************************************************************************/

#define THIS Vector3D

#include <sstream>

//----------------------------------------------------------------------------------------------------------
inline THIS::THIS()
{}
//----------------------------------------------------------------------------------------------------------
inline THIS::THIS ( float x, float y, float z )
{
  m_X = x; m_Y = y; m_Z = z;
};
//----------------------------------------------------------------------------------------------------------
inline THIS::THIS ( const Vector3D& v2 )
{
  m_X=v2[0];
  m_Y=v2[1];
  m_Z=v2[2];
};
//----------------------------------------------------------------------------------------------------------
inline THIS::THIS ( const CVec2& v, float z)
{
  m_X=v[0];
  m_Y=v[1];
  m_Z=z;
}
//----------------------------------------------------------------------------------------------------------
inline Vector3D THIS::operator+ ( const Vector3D& vVector ) const
{
  return Vector3D ( vVector[0] + m_X, vVector[1] + m_Y, vVector[2] + m_Z );
};
//----------------------------------------------------------------------------------------------------------
inline Vector3D THIS::operator-() const
{
  return Vector3D ( -m_X,-m_Y,-m_Z );
};
//----------------------------------------------------------------------------------------------------------
inline Vector3D THIS::operator+() const
{
  return Vector3D ( m_X,m_Y,m_Z );
};
//----------------------------------------------------------------------------------------------------------
inline Vector3D THIS::operator- ( const Vector3D& vVector ) const
{
  return Vector3D ( m_X-vVector[0],m_Y-vVector[1],m_Z-vVector[2] );
};
//----------------------------------------------------------------------------------------------------------
inline Vector3D THIS::operator* ( const float num ) const
{
  return Vector3D ( m_X * num, m_Y * num, m_Z * num );
};
//----------------------------------------------------------------------------------------------------------
inline Vector3D& THIS::operator*= ( const float num )
{
  m_X*=num; m_Y*=num; m_Z*=num;
  return *this;
}
//----------------------------------------------------------------------------------------------------------
inline Vector3D& THIS::operator/= ( const float num )
{
  m_X/=num; m_Y/=num; m_Z/=num;
  return *this;
}
//----------------------------------------------------------------------------------------------------------
inline float THIS::operator* ( const Vector3D& vVector ) const
{
  return vVector[0]*m_X+vVector[1]*m_Y+vVector[2]*m_Z;
};
//----------------------------------------------------------------------------------------------------------
inline Vector3D& THIS::operator+= ( const Vector3D& vVector )
{
  m_X+=vVector[0];
  m_Y+=vVector[1];
  m_Z+=vVector[2];
  return *this;
}
//----------------------------------------------------------------------------------------------------------
inline bool THIS::operator < ( const Vector3D& vVec ) const
{
  if ( m_X<vVec[0] ) return true;
  if ( m_X>vVec[0] ) return false;
  if ( m_Y<vVec[1] ) return true;
  if ( m_Y>vVec[1] ) return false;
  return ( m_Z<vVec[2] );
}
//----------------------------------------------------------------------------------------------------------
inline bool THIS::operator > ( const Vector3D& vVec ) const
{
  if ( m_X<vVec[0] ) return false;
  if ( m_X>vVec[0] ) return true;
  if ( m_Y<vVec[1] ) return false;
  if ( m_Y>vVec[1] ) return true;
  return ( m_Z>vVec[2] );
}
//----------------------------------------------------------------------------------------------------------
inline Vector3D& THIS::operator-= ( const Vector3D& vVector )
{
  m_X-=vVector[0];
  m_Y-=vVector[1];
  m_Z-=vVector[2];
  return *this;
}
//----------------------------------------------------------------------------------------------------------
inline Vector3D THIS::operator^ ( const Vector3D& vVector2 ) const
{
  Vector3D vNormal;

  // Calculate the cross product with the non communitive equation
  vNormal[0] = ( ( m_Y * vVector2[2] ) - ( m_Z * vVector2[1] ) );
  vNormal[1] = ( ( m_Z * vVector2[0] ) - ( m_X * vVector2[2] ) );
  vNormal[2] = ( ( m_X * vVector2[1] ) - ( m_Y * vVector2[0] ) );

  // Return the cross product
  return vNormal;
};
//----------------------------------------------------------------------------------------------------------
inline bool THIS::operator== ( const Vector3D& v1 ) const
{
  // this is evil!
  assert ( 0 );

  static const float EPS=1.0f/100.0f;
  if ( fabs ( v1[0]-m_X ) >EPS ) return false;
  if ( fabs ( v1[1]-m_Y ) >EPS ) return false;
  if ( fabs ( v1[2]-m_Z ) >EPS ) return false;

  return true;
};
//----------------------------------------------------------------------------------------------------------
inline Vector3D THIS::operator/ ( float num ) const
{
  return Vector3D ( m_X / num, m_Y / num, m_Z / num );
};
//----------------------------------------------------------------------------------------------------------
inline float THIS::operator [] ( const unsigned int i ) const
{
  return ( ( float* ) this ) [i];
}
//----------------------------------------------------------------------------------------------------------
inline float& THIS::operator [] ( const unsigned int i )
{
  return ( ( float* ) this ) [i];
}
//----------------------------------------------------------------------------------------------------------
inline void THIS::set ( float fx, float fy, float fz )
{
  m_X=fx; m_Y=fy; m_Z=fz;
}
//----------------------------------------------------------------------------------------------------------
inline void THIS::lerp ( const Vector3D& v1, const Vector3D& v2,float f )
{
  ( *this ) =v1* ( 1-f ) +v2*f;
}
//----------------------------------------------------------------------------------------------------------
inline float THIS::magnitude() const
{
  return sqrtf ( m_X*m_X+m_Y*m_Y+m_Z*m_Z );
}
//----------------------------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------------------------
inline Vector3D operator* ( const float n,const Vector3D& v )
{
  return Vector3D ( v[0]*n,v[1]*n,v[2]*n );
};
//----------------------------------------------------------------------------------------------------------


#undef THIS

