/*******************************************************************************
 *  ObjectProperties.cpp
 *
 *  (C) 2008 AG Aktives Sehen <agas@uni-koblenz.de>
 *           Universitaet Koblenz-Landau
 *
 *  Information on Code Review state:
 *  Author: <initials>; DevelTest: Date; Reviewer: Initials; Review: Date; State: NOK
 *
 *  Additional information:
 *  $Id: ObjectProperties.cpp 31722 2009-04-23 12:48:01Z sisuthie $
 *******************************************************************************/


#include "ObjectProperties.h"

#include "Architecture/Singleton/Clock.h"

#define THIS ObjectProperties

THIS::THIS( std::string name )
{
    m_Name=name;
    m_Type="";
}


THIS::~THIS()
{
    for ( unsigned i=0; i < m_ImageProperties.size(); i++)
    {
        if ( m_ImageProperties[i] ) { delete m_ImageProperties[i]; }
    }
}


THIS::THIS(const ObjectProperties& other)
{
    *this = other;
}


THIS& THIS::operator=(const ObjectProperties& other)
                     {
    for ( unsigned i=0; i < m_ImageProperties.size(); i++)
    {
        if ( m_ImageProperties[i] ) { delete m_ImageProperties[i]; }
    }
    m_ImageProperties.clear();

    //copy new data
    m_Name = other.m_Name;
    m_Type = other.m_Type;

    for ( unsigned i=0; i < other.m_ImageProperties.size(); i++)
    {
        m_ImageProperties.push_back( new ImagePropertiesCV( *(other.m_ImageProperties[i]) ) );
    }

    return *this;
}

void THIS::addImageProperties( ImagePropertiesCV* imageProperties )
{
    m_ImageProperties.push_back( imageProperties );
}

void THIS::deleteImageProperties( std::string name )
{
  std::vector<ImagePropertiesCV*> newImageProperties;

  for ( unsigned i=0; i<m_ImageProperties.size(); i++ )
  {
    if ( m_ImageProperties[i]->getName() == name )
    {
      delete m_ImageProperties[i];
    }
    else
    {
      newImageProperties.push_back( m_ImageProperties[i] );
    }
  }

  m_ImageProperties = newImageProperties;
}

void THIS::deleteImageProperties( int index )
{
  std::vector<ImagePropertiesCV*> newImageProperties;

  for ( unsigned i=0; i<m_ImageProperties.size(); i++ )
  {
    if ( i == (unsigned) index )
    {
      delete m_ImageProperties[i];
    }
    else
    {
      newImageProperties.push_back( m_ImageProperties[i] );
    }
  }

  m_ImageProperties = newImageProperties;
}

const ImagePropertiesCV* THIS::getImageProperties( std::string name ) const
{
  for ( unsigned i=0; i<m_ImageProperties.size(); i++ )
  {
    if ( m_ImageProperties[i]->getName() == name )
    {
      return m_ImageProperties[i];
    }
  }
  return 0;
}

std::vector<std::string> THIS::getImageNames()
{
  std::vector<std::string> result;
  result.reserve( m_ImageProperties.size() );

  for ( unsigned i=0; i<m_ImageProperties.size(); i++ )
  {
    result.push_back( m_ImageProperties[i]->getName() );
//     TRACE_INFO( m_ImageProperties[i]->getName() )
  }


  return result;
}


void THIS::printOn( std::ostream& strm )
{
    unsigned size=m_ImageProperties.size();

    strm << "Object name: " << m_Name << std::endl;
    strm << "Object type: " << m_Type << std::endl;
    strm << "# of images: " << size << std::endl << std::endl;

    strm << "Number keypoints in images:";
    for ( unsigned i=0; i < size; i++)
    {
        strm << " " << m_ImageProperties[i]->getKeyPoints()->size();
    }

    strm << std::endl << std::endl;
}

#undef THIS
