/*******************************************************************************
 *  ImageSourceSelector.cpp
 *
 *  (C) 2007 AG Aktives Sehen <agas@uni-koblenz.de>
 *           Universitaet Koblenz-Landau
 *
 *  Additional information:
 *  $Id: $ 
 *******************************************************************************/

#include "ImageSourceSelector.h"
#include <map>

#define THIS ImageSourceSelector

THIS::THIS( ImageSources::SourceId sourceId, QWidget* parent, ImageSources::SourceId firstSource, ImageSources::SourceId lastSource ): QComboBox( parent )
{
  
  std::map< ImageSources::SourceId, std::string > sourceDesc=ImageSources::getSourceDesc();
  std::map< ImageSources::SourceId, std::string >::iterator it;
  int i=0;
  int myEntry=0;
  for ( it=sourceDesc.begin(); it!=sourceDesc.end(); it++ )
  {
    if ( ( it->first < firstSource ) || ( it->first > lastSource ) )
    {
      continue;
    }
    std::ostringstream stream;
    stream.width(3);
    stream.fill('0');
    stream << int(it->first);
    stream << " " << it->second;
    addItem( stream.str().c_str() , it->first);
    if ( it->first==sourceId ) { myEntry=i; }
    m_SelectorSourceIds[i]=it->first;
    i++;
  }
  setCurrentIndex(myEntry);

  connect( this, SIGNAL(activated(int)), this, SLOT(selectSource(int)));
  
}

THIS::~THIS()
{
}

void THIS::selectSource( int index )
{
  emit sourceSelected( m_SelectorSourceIds[ index ] );
}

#undef THIS
