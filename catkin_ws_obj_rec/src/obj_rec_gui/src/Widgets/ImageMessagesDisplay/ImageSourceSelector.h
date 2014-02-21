/*******************************************************************************
 *  ImageSourceSelector.h
 *
 *  (C) 2007 AG Aktives Sehen <agas@uni-koblenz.de>
 *           Universitaet Koblenz-Landau
 *
 *  Additional information:
 *  $Id: $ 
 *******************************************************************************/

#ifndef ImageSourceSelector_H
#define ImageSourceSelector_H

#include <QComboBox>
#include <map>
#include "../../Workers/ImageSources/ImageSources.h"

/**
 * @class  ImageSourceSelector
 * @brief  Add description here
 * @author Add name here
 */
class ImageSourceSelector: public QComboBox
{

  Q_OBJECT
      
  public:
  
    /** @brief The constructor */
    ImageSourceSelector( ImageSources::SourceId sourceId=ImageSources::None, QWidget* parent=0, ImageSources::SourceId firstSource=ImageSources::TopCamera, ImageSources::SourceId lastSource=ImageSources::None );

    /** @brief The destructor */
    ~ImageSourceSelector();

  signals:

    void sourceSelected( ImageSources::SourceId );

  public slots:

    void selectSource( int index );
    
  private:

    std::map< int, ImageSources::SourceId > m_SelectorSourceIds;
  
};

#endif
