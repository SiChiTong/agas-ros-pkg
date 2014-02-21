/*******************************************************************************
*  ObjectImagesControl.cpp
*
*  (C) 2008 AG Aktives Sehen <agas@uni-koblenz.de>
*           Universitaet Koblenz-Landau
*
*  §Author: SG;
*
*******************************************************************************/

#include <vector>
#include <string>
#include <fstream>

#include <QWidget>
#include <QPushButton>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QString>
#include <QFileDialog>
#include <QFileInfo>
#include <QInputDialog>
#include <QLineEdit>
#include <QSlider>
#include <QLabel>
#include <QGroupBox>
#include <QComboBox>
#include <QHeaderView>
#include <QMessageBox>
#include <QSortFilterProxyModel>

#include <or_msgs/OrLearnCommand.h>
#include "Modules/ORLearningModule.h"

// TODO
//#include "Messages/ORLearningCommandM.h"
//#include "Messages/ORLearningStatusM.h"

#include "ObjectImagesControl.h"

#define THIS ObjectImagesControl

THIS::THIS ( ros::NodeHandle *nh, QWidget *parent ) : QWidget ( parent )
{
  m_Ready = false;

  QGridLayout* mainLayout = new QGridLayout();

  m_ImageTable = new QTableWidget();
  m_ImageTable->setColumnCount ( 2 );
  m_ImageTable->setRowCount ( 0 );
  QStringList headerList;
  headerList << "#" << "Name";
  m_ImageTable->setHorizontalHeaderLabels ( headerList );
  m_ImageTable->horizontalHeader()->setResizeMode ( 0, QHeaderView::Interactive );
  m_ImageTable->horizontalHeader()->setResizeMode ( 1, QHeaderView::Stretch );
  m_ImageTable->verticalHeader()->hide();
  //m_ImageTable->setSelectionBehavior ( QAbstractItemView::SelectRows );

  mainLayout->addWidget ( m_ImageTable, 1, 0, 1, 3 );

  QHBoxLayout* creationLayout = new QHBoxLayout( );
  m_ObjectNameField = new QLineEdit ( "" );
  m_ObjectNameField->setMinimumWidth ( 80 );
  creationLayout->addWidget ( new QLabel ( "Name:" ) );
  creationLayout->addWidget ( m_ObjectNameField );

  m_TypeComboBox = new QComboBox();
  m_TypeComboBox->addItem ( "Object" );
  m_TypeComboBox->addItem ( "Face" );
  creationLayout->addWidget ( m_TypeComboBox );
  m_saveObjectButton = new QPushButton ( QIcon ( "icons/save.png" ), "Save" );
  creationLayout->addWidget ( m_saveObjectButton );
  mainLayout->addLayout ( creationLayout, 0, 0, 1, 3 );

  m_removeImageButton = new QPushButton ( "Remove Image" );
  mainLayout->addWidget ( m_removeImageButton, 3, 0 );

  m_resetImagesButton = new QPushButton ( "Reset" );
  mainLayout->addWidget ( m_resetImagesButton, 3, 1 );

  QPushButton* loadObjectButton = new QPushButton ( "Load Object" );
  mainLayout->addWidget ( loadObjectButton, 3, 2 );

  mainLayout->addWidget ( new QLabel ( "Image name:" ), 4, 0 );
  m_ImageNameField = new QLineEdit ( "" );
  m_ImageNameField->setMinimumWidth ( 80 );
  mainLayout->addWidget ( m_ImageNameField, 4, 1 );

  QPushButton* saveImageButton = new QPushButton ( "Add Image" );
  mainLayout->addWidget ( saveImageButton, 4, 2 );

  setLayout ( mainLayout );

  connect ( saveImageButton, SIGNAL ( clicked() ), this, SLOT ( saveImage() ) );
  connect ( loadObjectButton, SIGNAL ( clicked() ), this, SLOT ( loadObject() ) );
  connect ( m_removeImageButton, SIGNAL ( clicked() ), this, SLOT ( removeImage() ) );
  // TODO add: connect ( m_resetImagesButton, SIGNAL ( clicked() ), this, SLOT ( resetImages() ) );
  connect ( m_saveObjectButton, SIGNAL ( clicked() ), this, SLOT ( saveObject() ) );
  connect ( m_ObjectNameField, SIGNAL ( textEdited ( QString ) ), this, SLOT ( objectNameFieldEdited ( QString ) ) );
  connect ( m_ObjectNameField, SIGNAL ( returnPressed() ), this, SLOT ( saveObject() ) );
  connect ( m_TypeComboBox, SIGNAL ( activated( QString ) ), this, SLOT ( setObjectType( QString ) ) );
  connect ( m_ImageTable, SIGNAL ( cellClicked ( int, int ) ), this, SLOT ( rowSelected ( int, int ) ) );

  m_LastImageFolder = "objectProperties";

  m_saveObjectButton->setEnabled ( false ); // TODO change to false after testing
  m_removeImageButton->setEnabled ( false );

  m_SelectedRow = 99999;

  setObjectType( "Object" );

  m_ORLearnCommandPublisher = nh->advertise<or_msgs::OrLearnCommand>("/or/learn_commands", 10);
  m_Ready = true;
}

THIS::~THIS() {}

void THIS::setObjectType ( QString type )
{
    if(m_Ready)
    {
        or_msgs::OrLearnCommand learnCommandMsg;
        learnCommandMsg.command = ORLearningModule::SetObjectType;
        learnCommandMsg.string_value = type.toStdString();
        m_ORLearnCommandPublisher.publish(learnCommandMsg);
    }
}

void THIS::saveImage()
{
  std::string name = ( m_ImageNameField->text() ).toStdString();

  if(m_Ready)
  {
      or_msgs::OrLearnCommand learnCommandMsg;
      learnCommandMsg.command = ORLearningModule::SaveImage;
      learnCommandMsg.string_value = name;
      m_ORLearnCommandPublisher.publish(learnCommandMsg);
  }

  //pre-select new image
  m_SelectedRow = m_ImageTable->rowCount();
}

void THIS::removeImage()
{
  if ( m_SelectedRow < m_ImageTable->rowCount() )
  {
      if(m_Ready)
      {
          or_msgs::OrLearnCommand learnCommandMsg;
          learnCommandMsg.command = ORLearningModule::DeleteImage;
          learnCommandMsg.int_value = m_SelectedRow;
          m_ORLearnCommandPublisher.publish(learnCommandMsg);
      }
  }
}

void THIS::saveObject()
{
  std::string objName = ( m_ObjectNameField->text() ).toStdString();
  std::string objType = m_TypeComboBox->currentText().toStdString();

  //TRACE_INFO ( "Creating Template '" + objName + "' with type '" + objType + "'" ); // TODO use ros

  if(m_Ready)
  {
      or_msgs::OrLearnCommand learnCommandMsg;
      learnCommandMsg.command = ORLearningModule::SetObjectType;
      learnCommandMsg.string_value = objType;
      m_ORLearnCommandPublisher.publish(learnCommandMsg);

      or_msgs::OrLearnCommand learnCommandMsg2;
      learnCommandMsg2.command = ORLearningModule::SaveObject;
      learnCommandMsg2.string_value = objName;
      m_ORLearnCommandPublisher.publish(learnCommandMsg2);
  }

  m_ImageTable->setRowCount ( 0 );
  m_ObjectNameField->clear();
  m_saveObjectButton->setEnabled ( false ); // TODO change to false after testing
}

void THIS::objectNameFieldEdited ( QString text )
{
  m_saveObjectButton->setEnabled ( text.length() > 0 && m_ImageTable->rowCount() > 0 );
  //m_saveObjectButton->setEnabled ( true ); // TODO remove after test
}

void THIS::keyReleaseEvent(QKeyEvent * e){
  if(e->key() == Qt::Key_Up)
  {
    if(m_SelectedRow > 0)
    {
      m_SelectedRow--;
      updateTable();
    }
  }
  if(e->key() == Qt::Key_Down)
  {
    if(m_SelectedRow < m_ImageTable->rowCount()-1)
    {
      m_SelectedRow++;
      updateTable();
    }
  }
}

void THIS::rowSelected ( int row, int col )
{
  m_SelectedRow = row;
  updateTable();
}

void THIS::updateTable()
{
  QColor rowColor1 ( 235,235,235 );
  QColor rowColor2 ( 180,180,180 );
  for ( int i = 0; i < m_ImageTable->rowCount(); i++ )
  {
    if ( i == m_SelectedRow )
    {
      m_ImageTable->item ( i,0 )->setBackgroundColor ( rowColor2 );
      m_ImageTable->item ( i,1 )->setBackgroundColor ( rowColor2 );
    }
    else
    {
      m_ImageTable->item ( i,0 )->setBackgroundColor ( rowColor1 );
      m_ImageTable->item ( i,1 )->setBackgroundColor ( rowColor1 );
    }
  }

  m_removeImageButton->setEnabled( m_SelectedRow < m_ImageTable->rowCount() );
  m_saveObjectButton->setEnabled( ( m_ObjectNameField->displayText().length() > 0 ) && ( m_ImageTable->rowCount() > 0 ) );
  //m_saveObjectButton->setEnabled ( true ); // TODO remove after test

  if ( m_SelectedRow < m_ImageTable->rowCount() )
  {
      if(m_Ready)
      {
          or_msgs::OrLearnCommand learnCommandMsg;
          learnCommandMsg.command = ORLearningModule::DisplayImage;
          learnCommandMsg.int_value = m_SelectedRow;
          m_ORLearnCommandPublisher.publish(learnCommandMsg);
      }
  }
  else
  {
    m_SelectedRow = 99999;
  }
}

void THIS::loadObject( )
{
  QStringList files = QFileDialog::getOpenFileNames ( this, tr ( "Select Object" ), m_LastImageFolder, "Object Properties (*.objprop)\nAll files (*)" );
  if( files.size()>0)
  {
    QFileInfo fileInfo( files[0] );
    QString base = fileInfo.baseName();
    QString absolutePath = fileInfo.absoluteFilePath();

    if(m_Ready)
    {
        ROS_INFO_STREAM("base: " << base.toStdString());
        ROS_INFO_STREAM("absolutePath: " << absolutePath.toStdString());

        or_msgs::OrLearnCommand learnCommandMsg;
        learnCommandMsg.command = ORLearningModule::LoadObject;
        learnCommandMsg.string_value = absolutePath.toStdString();
        m_ORLearnCommandPublisher.publish(learnCommandMsg);
    }

    m_ObjectNameField->setText( base );
  }
}


void THIS::updateImageTable(std::vector<std::string> &imageNames, std::string &objType)
{
    ROS_INFO_STREAM("Updating table, adding item type: " << objType );
    m_ImageTable->setRowCount ( 0 );

    for ( unsigned i = 0; i < imageNames.size(); i++ )
    {
      int indexRow = m_ImageTable->rowCount();

      //create item with index number
      m_ImageTable->insertRow ( indexRow );
      std::ostringstream s;
      s << i;
      QTableWidgetItem* item = new QTableWidgetItem ( QString( s.str().c_str() ) );
      item->setTextAlignment ( Qt::AlignCenter );
      item->setFlags ( Qt::ItemIsSelectable );
      m_ImageTable->setItem ( indexRow, 0, item );

      //create item with basename
      item = new QTableWidgetItem ( QString ( imageNames[i].c_str() ) );
      item->setTextAlignment ( Qt::AlignCenter );
      item->setFlags ( Qt::ItemIsSelectable );
      m_ImageTable->setItem ( indexRow, 1, item );
    }

    updateTable();

    m_TypeComboBox->setCurrentIndex( m_TypeComboBox->findText( objType.c_str() ) );

}

#undef THIS
