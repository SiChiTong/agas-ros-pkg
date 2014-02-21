/***************************************************************************
 *  PointOfInterest.cpp
 *
 *  (C) 2007 AG Aktives Sehen <agas@uni-koblenz.de>
 *           Universitaet Koblenz-Landau
 *
 *  Additional information:
 *  $Id: PointOfInterest.cpp 44313 2011-04-06 22:46:28Z agas $
 **************************************************************************/

#include <algorithm>
#include "PointOfInterest.h"

#define THIS PointOfInterest

void THIS::init ( int id, std::string name, PoiType type, std::string remarks, std::map< std::string, std::string > stringMap,
                  std::map< std::string, float > floatMap, std::map< std::string, int > intMap ) {
  m_Id = id;
  m_Name = name;
  m_Type = type;
  m_Remarks = remarks;
  m_StringMap = stringMap;
  m_FloatMap = floatMap;
  m_IntMap = intMap;
}

THIS::THIS( ): Pose(0, 0, 0)
{
    init( -1 , "empty", DEFAULT, "", std::map< std::string, std::string >(), std::map< std::string, float >(), std::map< std::string, int >() );
}

THIS::THIS( int id, const PointOfInterest* poi ): Pose(poi->x(), poi->y(), poi->theta())
{
  if (poi) {
    init(id, poi->m_Name, poi->m_Type, poi->m_Remarks, poi->m_StringMap, poi->m_FloatMap, poi->m_IntMap);
  } else {
    // TRACE_ERROR("Copy constructor called with 0-pointer as argument"); // TODO use ROS
    init(-1, "",DEFAULT,"",StringMapT(),FloatMapT(),IntMapT());
  }
}

bool THIS::hasName( std::string name ) const
{
  // transform both name strings to upper case
  std::string poiName = m_Name;
  transform( poiName.begin(), poiName.end(), poiName.begin(), ( int ( * ) ( int ) )toupper );
  transform( name.begin(), name.end(), name.begin(), ( int ( * ) ( int ) )toupper );

  return ( poiName.compare( name ) == 0 );
}

bool THIS::hasInName( std::string part ) const
{
  // transform both strings to upper case
  std::string poiName = m_Name;
  transform( poiName.begin(), poiName.end(), poiName.begin(), ( int ( * ) ( int ) )toupper );
  transform( part.begin(), part.end(), part.begin(), ( int ( * ) ( int ) )toupper );

  return ( ( int )poiName.find( part ) != -1 );
}

//void THIS::storer( ExtendedOutStream& extStrm ) const
//{
//  extStrm << m_Id;
//  extStrm << m_Name;
//  extStrm << m_Type;
//  extStrm << m_Remarks;
//  extStrm << m_StringMap;
//  extStrm << m_FloatMap;
//  extStrm << m_IntMap;
//  extStrm << m_X;
//  extStrm << m_Y;
//  extStrm << m_Theta;
//}


//THIS::THIS( ExtendedInStream& extStrm )
//{
//  extStrm >> m_Id;
//  extStrm >> m_Name;
//  extStrm >> m_Type;
//  extStrm >> m_Remarks;
//  extStrm >> m_StringMap;
//  extStrm >> m_FloatMap;
//  extStrm >> m_IntMap;
//  extStrm >> m_X;
//  extStrm >> m_Y;
//  extStrm >> m_Theta;
//}

void THIS::printOn( std::ostream& strm ) const
{
  strm << m_Id << ", ";
  strm << m_Name << ", ";
  strm << m_Type << ", ";
  strm << m_Remarks << ",";
  strm << "(" << m_X << "," << m_Y<< "," << m_Theta << ")";
}

#undef THIS
