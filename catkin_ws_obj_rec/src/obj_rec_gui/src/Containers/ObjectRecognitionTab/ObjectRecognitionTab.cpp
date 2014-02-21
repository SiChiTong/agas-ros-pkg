/*******************************************************************************
*  ObjectRecognitionTab.cpp
*
*  (C) 2006 AG Aktives Sehen <agas@uni-koblenz.de>
*           Universitaet Koblenz-Landau
*
*  Information on Code Review state:
*  §Author:RH$
*
*  Additional information:
*  $Id: ObjectRecognitionTab.cpp 23983 2008-04-07 00:16:24Z dgossow $
*******************************************************************************/

#include <QGridLayout>
#include <QBoxLayout>
#include <QDockWidget>
#include <QVBoxLayout>
#include <QPushButton>
#include <QScrollArea>

#include <ostream>

#include "../../Widgets/ObjectRecognition/ObjectList.h"

#include "ObjectRecognitionTab.h"

#define THIS ObjectRecognitionTab


THIS::THIS(ros::NodeHandle *nodeHandle, QWidget *parent ) : QWidget( parent ) {

    QGridLayout* mainLayout = new QGridLayout();

    //Add 'ObjectRecognitionDisplay'; contains the image, 'load' and 'delete' buttons with whole right side bar
    QVBoxLayout* checkboxLayout = new QVBoxLayout();
    m_ObjectRecognitionDisplay=new ObjectRecognitionDisplay(nodeHandle, checkboxLayout, this);
    mainLayout->addWidget( m_ObjectRecognitionDisplay, 0, 0, 1, 1 );

    QWidget* checkboxContainer = new QWidget();
    checkboxContainer->setLayout( checkboxLayout );
    mainLayout->addWidget( checkboxContainer, 0, 1, 1, 3 );

    mainLayout->setColumnStretch(0,4);
    setLayout( mainLayout );
}

void THIS::processObjectNames(std::vector<std::string> names, std::vector<std::string> types)
{
    m_ObjectRecognitionDisplay->updateObjectTable(names, types);
}
