/*******************************************************************************
 *  NavigationMap.cpp
 *
 *  (C) 2007 AG Aktives Sehen <agas@uni-koblenz.de>
 *           Universitaet Koblenz-Landau
 *
 *  Information on Code Review state:
 *  Author: CB; DevelTest: 05.03.2007; Reviewer: initials; Review: date; State: NOK
 *
 *  Additional information:
 *  $Id: PoiManager.cpp 44313 2011-04-06 22:46:28Z agas $
 *******************************************************************************/

#include "PoiManager.h"

#include <sstream>

#define THIS PoiManager

//#include <iostream>

using namespace std;


THIS::THIS()
{
  m_CurrentId=0;
}

THIS::THIS ( list<PointOfInterest> pois )
{
  //copy POIs
  m_Pois = pois;

  //search maximum occupied ID
  list<PointOfInterest>::iterator it;
  m_CurrentId=0;

  for ( it=m_Pois.begin() ; it != m_Pois.end(); it++ )
  {
    if ( it->getId() >m_CurrentId )
    {
      m_CurrentId=it->getId();
    }
  }

  //set next ID to be used
  m_CurrentId++;

  ostringstream stream;

  stream << "Copied POI list. Next ID to be used: " << m_CurrentId;

  // TRACE_INFO ( stream.str() ); // TODO use ROS
}

list<PointOfInterest> THIS:: getList()
{
  return m_Pois;
}

bool THIS::addPointOfInterest ( int id, const PointOfInterest* poi )
{
  ostringstream stream;

  //make sure there's no POI with the same ID

  if ( poiExists ( id ) )
  {
    ostringstream stream;
    stream << "Poi with id " << id << " already exists! Doing nothing.";
    // TRACE_ERROR ( stream.str() ); // TODO use ROS
    return false;
  }

  //copy poi & assigning new id
  PointOfInterest new_poi=PointOfInterest ( id,poi );

  stream << "Adding POI '" << new_poi.getName() << "'. New ID: " << id << ".";

  // TRACE_INFO ( stream.str() ); // TODO use ROS

  //insert into list
  m_Pois.push_back ( new_poi );

  return true;
}

int THIS::addPointOfInterest ( const PointOfInterest* poi )
{
  addPointOfInterest ( m_CurrentId,poi );
  m_CurrentId++;

  return m_CurrentId-1;
}

bool THIS::modifyPointOfInterest ( const PointOfInterest* poi )
{
  int id=const_cast < PointOfInterest* > ( poi )->getId();

  list<PointOfInterest>::iterator it;

  for ( it=m_Pois.begin() ; it != m_Pois.end(); it++ )
  {
    if ( it->getId() ==id )
    {
      *it=*poi;
      return true;
    }
  }

  // TRACE_ERROR ( "Cannot modify: POI does not exist!" ); // TODO use ROS

  return false;
}

bool THIS::poiExists ( int id )
{
  list<PointOfInterest>::iterator it;

  for ( it=m_Pois.begin() ; it != m_Pois.end(); it++ )
  {
    if ( it->getId() ==id )
    {
      return true;
    }
  }

  return false;
}

bool THIS::deletePointOfInterest ( int id )
{
  ostringstream s;

  list< PointOfInterest >::iterator it;

  for ( it=m_Pois.begin() ; it != m_Pois.end(); it++ )
  {
    if ( it->getId() == id )
    {
      s << "Erasing POI " << id << ".";
      // TRACE_INFO ( s.str() ); // TODO use ROS

      it = m_Pois.erase ( it );

      return true;
    }
  }

  s << "POI " << id << " does not exist.";

  // TRACE_ERROR ( s.str() ); // TODO use ROS

  return false;
}

int THIS::deletePointOfInterest ( string namePart )
{
  int deleteCount = 0;

  list< PointOfInterest >::iterator it = m_Pois.begin();

  while ( it != m_Pois.end() )
  {
    if ( it->hasInName ( namePart ) )
    {
      ostringstream s;
      s << "Erasing POI " << it->getId() << " named '" << it->getName() << "'.";
      // TRACE_INFO ( s.str() ); // TODO use ROS

      it = m_Pois.erase ( it );
      deleteCount++;
    }
    else
    {
      it++;
    }
  }

  return deleteCount;
}


#undef THIS
