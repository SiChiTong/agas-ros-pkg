/*******************************************************************************
*  ObjectLearningTab.cpp
*
*  (C) 2006 AG Aktives Sehen <agas@uni-koblenz.de>
*           Universitaet Koblenz-Landau
*
*  Information on Code Review state:
*  §Author:RH$
*
*  Additional information:
*  $Id: ObjectLearningTab.cpp 23983 2008-04-07 00:16:24Z dgossow $
*******************************************************************************/

#include <QGridLayout>
#include <QVBoxLayout>
#include <QDockWidget>

#include <ostream>
#include <sensor_msgs/Image.h>

#include "ObjectLearningTab.h"

#define THIS ObjectLearningTab


THIS::THIS( ros::NodeHandle *nodeHandle, QWidget *parent ) : QWidget( parent ) {

  ObjectLearningControl* learningControl = new ObjectLearningControl( nodeHandle, this );
  
  // main layout
  QGridLayout* mainLayout = new QGridLayout();

  QVBoxLayout* leftLayout = new QVBoxLayout();
  leftLayout->setContentsMargins( 1,1,1,1 );

    QVBoxLayout* rightLayout = new QVBoxLayout();
  rightLayout->setContentsMargins( 1,1,1,1 );
  

  m_ImagesControl=new ObjectImagesControl( nodeHandle, this );
  rightLayout->addWidget( m_ImagesControl );
  rightLayout->setStretchFactor( m_ImagesControl, 2 );

  
  mainLayout->setColumnStretch(0,4);
  mainLayout->setColumnStretch(1,2);

  mainLayout->setContentsMargins( 1,1,1,1 );

  mainLayout->addLayout( leftLayout, 0, 0 );
  mainLayout->addLayout( rightLayout, 0, 1 );
  mainLayout->addWidget( learningControl, 1, 0, 1, 2 );
  
  setLayout( mainLayout );
}



void THIS::processLearningStatus(std::vector<std::string> filenames, std::string objType)
{
    m_ImagesControl->updateImageTable(filenames, objType);
}

#undef THIS
