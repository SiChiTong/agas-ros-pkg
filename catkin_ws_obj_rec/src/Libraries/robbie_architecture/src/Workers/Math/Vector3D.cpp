/*******************************************************************************
 *  Vector3D.cpp
 *
 *  (C) 2007 AG Aktives Sehen <agas@uni-koblenz.de>
 *           Universitaet Koblenz-Landau
 *
 *  Additional information:
 *  $Id: $
 *******************************************************************************/

#include "Vector3D.h"

#define THIS Vector3D

using namespace std;

std::string THIS::toString( int precision, std::string name )
{
  std::ostringstream s;

  s.precision( precision );
  s.setf(ios::fixed,ios::floatfield);

  for ( unsigned row=0; row<3; row++ )
  {
    if ( name != "" )
    {
      if ( row==1 )
      {
        s << name << " = ";
      }
      else
      {
        s.width( name.length()+3 );
        s << "";
      }
    }

    s << "( ";
    s.width( precision+4 );
    switch ( row )
    {
      case 0:
        s << m_X << " ";
        break;
      case 1:
        s << m_Y << " ";
        break;
      case 2:
        s << m_Z << " ";
        break;
    }
    s << ")" << endl;
  }
  return s.str();
}

#undef THIS
