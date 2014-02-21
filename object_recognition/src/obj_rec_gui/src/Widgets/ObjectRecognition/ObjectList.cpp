/*******************************************************************************
*  ObjectList.cpp
*
*  (C) 2008 AG Aktives Sehen <agas@uni-koblenz.de>
*           Universitaet Koblenz-Landau
*
*  §Author: SG;
*
*******************************************************************************/

#include "ObjectList.h"

#include <vector>
#include <string>
#include <QWidget>
#include <QPushButton>
#include <QVBoxLayout>
#include <QString>
#include <QTableWidget>
#include <QHeaderView>
#include <QFileDialog>
#include <QGroupBox>
#include <QLabel>
#include <QMessageBox>

#include <ros/ros.h>

//TODO
//#include "Messages/ORObjectPropertiesM.h"
//#include "Messages/ORObjectNamesM.h"
//#include "Messages/ORCommandM.h"

#include "../../Workers/ObjectRecognition/ObjectProperties.h"

//#include "Architecture/Serializer/BinaryInStream.h"
//#include "Architecture/Serializer/ExtendedInStream.h"

//included for debug Purposes
#include <iostream>
#include <fstream>

#define THIS ObjectList


THIS::THIS(QWidget *parent): QWidget( parent )
{

  setMinimumSize(100,100);

    QPushButton* loadObjectButton;

    loadObjectButton = new QPushButton("Load");
    deleteObjectButton = new QPushButton("Delete");;
    deleteObjectButton->setEnabled(false);

    m_objectsList = new QTableWidget();
    m_objectsList->setColumnCount ( 2 );
    m_objectsList->setRowCount( 0 );
    QStringList headerList;
    headerList << "Name" << "Type";
    m_objectsList->setHorizontalHeaderLabels (headerList);
    m_objectsList->horizontalHeader()->setResizeMode(0, QHeaderView::Interactive);
    m_objectsList->horizontalHeader()->setResizeMode(1, QHeaderView::Stretch);
    m_objectsList->verticalHeader()->hide();
    m_objectsList->setSelectionBehavior(QAbstractItemView::SelectRows);
    m_objectsList->setSortingEnabled(true);

    //create main layout
    QVBoxLayout* mainLayout = new QVBoxLayout();

    QGridLayout* listLayout = new QGridLayout();
    listLayout->addWidget(m_objectsList,0,0,1,2);
    listLayout->addWidget(loadObjectButton,1,0);
    listLayout->addWidget(deleteObjectButton,1,1);

    heading=new QLabel( "Objects (0)" );
    QFont font;
    font.setBold(true);
    heading->setFont(font);

    mainLayout->addWidget( heading );
    mainLayout->addLayout( listLayout );
    mainLayout->setSpacing(0);
    setLayout(mainLayout);

    //signal slot connections
    connect(loadObjectButton,SIGNAL(clicked()),this,SLOT(loadObject()));
    connect(deleteObjectButton,SIGNAL(clicked()),this,SLOT(deleteObject()));

    m_LastOpenPath = "/objectProperties"; // TODO check whether path is correct
}

THIS::~THIS(){}

// TODO
//void THIS::processMessage( Message* newMessage )
//{
//    switch(newMessage->getType()) {

//    case MessageTypes::OR_OBJECT_NAMES_M:
//        {
//            if (ORObjectNamesM* message=Message::castTo<ORObjectNamesM>(newMessage))
//            {
//                map<string,string> names = message->getNames();

//                m_objectsList->setRowCount( 0 );

//                m_objectsList->setSortingEnabled(false);

//                map<string,string>::iterator it;
//                for ( it=names.begin() ; it != names.end(); it++ )
//                {
//                    QString objectName = QString((*it).first.c_str());

//                    int indexRow = m_objectsList->rowCount();
//                    QColor rowColor(235,235,235);

//                    //create item with object name
//                    m_objectsList->insertRow (indexRow);
//                    QTableWidgetItem* item = new QTableWidgetItem(objectName);
//                    item->setBackground(rowColor);
//                    item->setFlags(Qt::ItemIsSelectable);
//                    m_objectsList->setItem (indexRow,0,item);

//                    //create item with object type
//                    item = new QTableWidgetItem(QString((*it).second.c_str()));
//                    item->setBackground(rowColor);
//                    item->setFlags(Qt::ItemIsSelectable);
//                    item->setTextAlignment(Qt::AlignCenter);
//                    m_objectsList->setItem (indexRow,1,item);

//                    //select row in table that was just added
//                    m_objectsList->selectRow(indexRow);
//                    m_objectsList->scrollToItem(m_objectsList->item(indexRow,0), QAbstractItemView::EnsureVisible);
//                }
//                m_objectsList->setSortingEnabled(true);
//                deleteObjectButton->setEnabled(m_objectsList->rowCount()>0);
//                heading->setText("Objects ("+QString::number(m_objectsList->rowCount()) +")");
//            }
//        } break;

//    default:
//        break;

//    }
//}

void THIS::deleteObject(){
    m_objectsList->setSortingEnabled(false);

    QList<QTableWidgetItem *> selectedItems = m_objectsList->selectedItems();

    int i=0;
    while(i<m_objectsList->rowCount())
    {
        if(selectedItems.contains(m_objectsList->item(i,0)))
        {
            // TODO
            //sendMessage(new ORCommandM( ORCommandM::UnloadObject, m_objectsList->item(i,0)->text().toStdString()));
            m_objectsList->removeRow(i);
            --i;
        }
        ++i;
    }
    m_objectsList->setSortingEnabled(true);
    deleteObjectButton->setEnabled(m_objectsList->rowCount()>0);
    heading->setText("Objects ("+QString::number(m_objectsList->rowCount()) +")");
}

void THIS::loadObject() // TODO fix this method to load objects!!!!
{
    QStringList files = QFileDialog::getOpenFileNames(this, tr("Load Object"), m_LastOpenPath,"Objectproperties (*.objprop)");

    for (int i = 0; i < files.size(); i++)
    {
        QString file=files.at(i);
        QDir fileDir( file );
        fileDir.cdUp();
        m_LastOpenPath = fileDir.absolutePath();

        std::ifstream in(file.toStdString().c_str());

        ROS_INFO_STREAM("file: " << file.toStdString());




        // TODO this is needed to load objects !!!
//        BinaryInStream binIn(in);
//        ExtendedInStream extStrm( binIn );
//        ObjectProperties* objectProperties = new ObjectProperties(extStrm);
//        in.close();

//        bool addObject = true;
//        QString objectName = QString(objectProperties->getName().c_str());

//        m_objectsList->setSortingEnabled(false);
//        for(int j=0;j<m_objectsList->rowCount();++j)
//        {
//            if(m_objectsList->item(j,0)->text()==(objectName))
//            {
 //               addObject = false;
//                break;
//            }
//        }
//        m_objectsList->setSortingEnabled(true);


//        //add object only if it wasn't already added
//        if(addObject){
//            // TODO
//            //ORObjectPropertiesM* message = new ORObjectPropertiesM( objectProperties );
//            //sendMessage(message);
//        }
//        else
//        {
//            //Display information if object was already to the table
//            QMessageBox m(this);
//            m.setIcon(QMessageBox::Information);
//            m.setText("Object '" + objectName + "' is already inserted.");
//            m.exec();
//        }
    }
}

#undef THIS
