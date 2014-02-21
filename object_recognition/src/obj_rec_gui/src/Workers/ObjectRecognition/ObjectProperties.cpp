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

// TODO
//#include "Architecture/Serializer/ExtendedOutStream.h"
//#include "Architecture/Serializer/ExtendedInStream.h"

#define THIS ObjectProperties

using namespace puma2;
// using namespace NIHCL_NS; // TODO get rid of this


THIS::THIS( std::string name )
{
    m_Name=name;
    m_Type="";
    m_MeanHistogram=0;
}


THIS::~THIS()
{
    if (m_MeanHistogram) {
        delete m_MeanHistogram;
    }
    for ( unsigned i=0; i < m_ImageProperties.size(); i++)
    {
        if ( m_ImageProperties[i] ) { delete m_ImageProperties[i]; }
    }
}


THIS::THIS(const ObjectProperties& other)
{
    m_MeanHistogram=0;
    *this = other;
}


THIS& THIS::operator=(const ObjectProperties& other)
                     {
    //delete old data
    if (m_MeanHistogram) {
        delete m_MeanHistogram;
    }
    for ( unsigned i=0; i < m_ImageProperties.size(); i++)
    {
        if ( m_ImageProperties[i] ) { delete m_ImageProperties[i]; }
    }
    m_ImageProperties.clear();

    //copy new data
    m_Name = other.m_Name;
    m_Type = other.m_Type;
    m_MeanHistogram = new HistogramUV( *(other.m_MeanHistogram) );

    for ( unsigned i=0; i < other.m_ImageProperties.size(); i++)
    {
        m_ImageProperties.push_back( new ImageProperties( *(other.m_ImageProperties[i]) ) );
    }

    return *this;
}

void THIS::addImageProperties( ImageProperties* imageProperties )
{
    m_ImageProperties.push_back( imageProperties );
    if (!m_MeanHistogram) {
        m_MeanHistogram=new HistogramUV( imageProperties->getHistogram()->getBinSize() );
    }
    m_MeanHistogram->add( *(imageProperties->getHistogram()) );
}

void THIS::deleteImageProperties( std::string name )
{
  std::vector<ImageProperties*> newImageProperties;

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
  std::vector<ImageProperties*> newImageProperties;

  for ( unsigned i=0; i<m_ImageProperties.size(); i++ )
  {
    if ( i == index )
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

const ImageProperties* THIS::getImageProperties( std::string name ) const
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


//// SERIALIZATION / DESERIALIZATION //////////////////////////////////////////////////

//THIS::THIS( ExtendedInStream& extStrm )
//{
//    unsigned version = 0;
//    extStrm >> version;

//    if ( version != 12 )
//    {
//      throw "File has wrong version number.";
//    }

//    m_MeanHistogram = 0;

//    extStrm >> m_Name;
//    extStrm >> m_Type;
//    unsigned size;
//    extStrm >> size;
//    m_ImageProperties.reserve( size );
//    for ( unsigned i=0; i < size; i++)
//    {
//        ImageProperties* imageProperties=new ImageProperties( extStrm );
//        addImageProperties( imageProperties );
//    }
//}


//void THIS::storer( ExtendedOutStream& extStrm )
//{
//    extStrm << unsigned(12);

//    extStrm << m_Name;
//    extStrm << m_Type;

//    unsigned size=m_ImageProperties.size();
//    extStrm << size;
//    for ( unsigned i=0; i < size; i++)
//    {
//        m_ImageProperties[i]->storer( extStrm );
//    }
//}


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

    strm << "Histogram:" << std::endl;
    m_MeanHistogram->printOn( strm );
    strm << std::endl << std::endl;
}

#undef THIS
