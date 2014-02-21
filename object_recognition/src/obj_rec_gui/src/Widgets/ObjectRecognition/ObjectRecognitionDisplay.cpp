/*******************************************************************************
 *  ObjectRecognitionDisplay.cpp
 *
 *  (C) 2006 AG Aktives Sehen <agas@uni-koblenz.de>
 *           Universitaet Koblenz-Landau
 *
 *  Additional information:
 *  $Id: ObjectRecognitionDisplay.cpp 23656 2008-03-30 18:21:56Z dgossow $
 *******************************************************************************/

#include <QGridLayout>
#include <QLabel>
#include <QMatrix>
#include <QPixmap>
#include <QSize>
#include <QComboBox>
#include <QCheckBox>
#include <QPushButton>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QDockWidget>
#include <QGroupBox>
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
#include <QComboBox>

#include <or_msgs/OrCommand.h>

#include "ObjectRecognitionDisplay.h"
#include "Modules/ORControlModule.h"

#include "Widgets/ImageMessagesDisplay/ImageSourceSelector.h"
#include "Widgets/GLImageWidget/GLImageWidget.h"

// TODO
//#include "Architecture/Serializer/BinaryInStream.h"
//#include "Architecture/Serializer/ExtendedInStream.h"

// TODO
//#include "Messages/ORObjectNamesM.h"
//#include "Messages/GetImageM.h"
//#include "Messages/ORObjectPropertiesM.h"
//#include "Messages/ORCommandM.h"
//#include "Messages/ORMatchResultM.h"
//#include "Messages/ImageM.h"

#include "Workers/Puma2/ThermalToColorOperator.h"
#include "Workers/ImageSources/ImageSources.h"
#include "Workers/ObjectRecognition/ObjectProperties.h"

//included for debug Purposes
#include <iostream>
#include <fstream>

#define THIS ObjectRecognitionDisplay

using namespace std;
using namespace puma2;


THIS::THIS ( ros::NodeHandle *nh, QBoxLayout* checkboxLayout, QWidget* parent ) : QGLWidget ( parent )
{
    m_Ready = false;
    m_LastOpenPath = "./objectProperties";

    QVBoxLayout* mainLayout = new QVBoxLayout();

    m_GlImageWidget = new GLImageWidget ( this );
    //mainLayout->addWidget ( m_GlImageWidget, 1 );

    //Add 'Grab Image' Widget
    QHBoxLayout* grabImageLayout = new QHBoxLayout();
    QWidget* grabImageContainer = new QWidget();
    grabImageContainer->setMaximumWidth ( 600 );
    grabImageContainer->setLayout ( grabImageLayout );

    //Image source selector
    ImageSourceSelector* sourceSelector = new ImageSourceSelector( ImageSources::TopCamera, this, ImageSources::TopCamera, ImageSources::SourceId(99) );
    setCameraId( ImageSources::TopCamera );
    connect( sourceSelector, SIGNAL( sourceSelected( ImageSources::SourceId ) ), this, SLOT( setCameraId( ImageSources::SourceId ) ) );
    //grabImageLayout->addWidget(sourceSelector);

    //'Grab Image' Button
    QPushButton* grabButton=new QPushButton ( "Grab Image" );
    grabImageLayout->addWidget ( grabButton );
    connect ( grabButton, SIGNAL ( clicked() ), this, SLOT ( grabImage() ) );

    QPushButton* loadButton=new QPushButton ( "Load Image" );
    grabImageLayout->addWidget ( loadButton );
    connect ( loadButton, SIGNAL ( clicked() ), this, SLOT ( loadImage() ) );

    QPushButton* startLoopButton=new QPushButton ( "Start Recognition Loop" );
    grabImageLayout->addWidget ( startLoopButton );
    connect ( startLoopButton, SIGNAL ( clicked() ), this, SLOT ( startLoop() ) );

    QPushButton* stopLoopButton=new QPushButton ( "Stop Recognition Loop" );
    grabImageLayout->addWidget ( stopLoopButton );
    connect ( stopLoopButton, SIGNAL ( clicked() ), this, SLOT ( stopLoop() ) );

    mainLayout->addWidget ( grabImageContainer );


    QGroupBox* groupBox1=new QGroupBox ( tr ( "Scene Image Features" ) );

    QVBoxLayout* checkboxLayout1=new QVBoxLayout();
    addCheckBox ( checkboxLayout1, m_OptionCheckBoxes, "boundingBoxes", "Bounding Boxes", true );
    addCheckBox ( checkboxLayout1, m_OptionCheckBoxes, "sceneAreas", "KeyPoint Area", false );
    addCheckBox ( checkboxLayout1, m_OptionCheckBoxes, "sceneArrows", "KeyPoint Orientation", false );
    addCheckBox ( checkboxLayout1, m_OptionCheckBoxes, "sceneCircles", "KeyPoint Circles", false );
    addCheckBox ( checkboxLayout1, m_OptionCheckBoxes, "sceneArrowsWithinOutline", "KeyPoints within projected outline", false );
    checkboxLayout1->setSpacing ( 0 );
    checkboxLayout1->setMargin ( 0 );
    checkboxLayout1->setContentsMargins ( 10,0,0,0 );
    groupBox1->setLayout ( checkboxLayout1 );
    checkboxLayout->addWidget ( groupBox1 );


    QGroupBox* groupBox2=new QGroupBox ( tr ( "Object matching" ) );

    QVBoxLayout* checkboxLayout2=new QVBoxLayout();
    addCheckBox ( checkboxLayout2, m_OptionCheckBoxes, "outline", "Object Outline", true );
    addCheckBox ( checkboxLayout2, m_OptionCheckBoxes, "stage1", "Stage 1 matches", false );


    addCheckBox ( checkboxLayout2, m_OptionCheckBoxes, "stage2", "Stage 2 matches", false );

    binChooserComboBox = new QComboBox();
    binChooserComboBox->setEnabled(false);
    checkboxLayout2->addWidget ( binChooserComboBox );
    connect ( binChooserComboBox,SIGNAL ( currentIndexChanged(int) ),this,SLOT ( optionsChanged() ) );

    addCheckBox ( checkboxLayout2, m_OptionCheckBoxes, "stage3", "Stage 3 matches", true );
    addCheckBox ( checkboxLayout2, m_OptionCheckBoxes, "scene", "Scene Matches", true );
    addCheckBox ( checkboxLayout2, m_OptionCheckBoxes, "object", "Object Matches", true );
    addCheckBox ( checkboxLayout2, m_OptionCheckBoxes, "areas", "KeyPoint Area", false );
    addCheckBox ( checkboxLayout2, m_OptionCheckBoxes, "arrows", "KeyPoint Orientation", true );
    addCheckBox ( checkboxLayout2, m_OptionCheckBoxes, "lines", "KeyPoint correspondences", true );
    checkboxLayout2->setSpacing ( 0 );
    groupBox2->setLayout ( checkboxLayout2 );
    checkboxLayout->addWidget ( groupBox2 );
    checkboxLayout->addStretch();

    //Add 'Objects' header
    m_ObjCheckboxLayout=new QVBoxLayout();
    m_ObjectsGroupBox=new QGroupBox ( "Objects (0)" );
    m_ObjCheckboxLayout->setSpacing ( 0 );

    //Add Objects List
    m_ObjectsList = new QTableWidget();
    m_ObjectsList->setColumnCount ( 3 );
    m_ObjectsList->setRowCount ( 0 );
    QStringList headerList;
    headerList << "" << "Name" << "Type";
    m_ObjectsList->setHorizontalHeaderLabels ( headerList );

    QTableWidgetItem *checkHeaderItem = new QTableWidgetItem();
    checkHeaderItem->setFlags ( Qt::ItemIsUserCheckable | Qt::ItemIsEnabled );
    checkHeaderItem->setCheckState ( Qt::Checked );

    m_ObjectsList->horizontalHeader()->setResizeMode ( 0, QHeaderView::Interactive );
    m_ObjectsList->horizontalHeader()->setResizeMode ( 1, QHeaderView::Interactive );
    m_ObjectsList->horizontalHeader()->setResizeMode ( 2, QHeaderView::Stretch );
    m_ObjectsList->verticalHeader()->hide();
    m_ObjectsList->setSelectionMode ( QAbstractItemView::NoSelection );
    m_ObjectsList->setSortingEnabled ( true );
    m_ObjCheckboxLayout->addWidget ( m_ObjectsList );

    connect ( m_ObjectsList, SIGNAL ( cellClicked ( int,int ) ), this ,SLOT ( cellClicked ( int,int ) ) );

    //Add load and delete Buttons
    QHBoxLayout* loadDeleteLayout = new QHBoxLayout();
    QPushButton* loadObjectButton = new QPushButton ( "Load" );
    loadDeleteLayout->addWidget ( loadObjectButton );
    m_DeleteObjectButton = new QPushButton ( "Delete" );;
    loadDeleteLayout->addWidget ( m_DeleteObjectButton );
    m_DeleteObjectButton->setEnabled ( false );
    QWidget* loadDeleteContainer = new QWidget();
    loadDeleteContainer->setLayout ( loadDeleteLayout );
    m_ObjCheckboxLayout->addWidget ( loadDeleteContainer );
    m_ObjectsGroupBox->setLayout ( m_ObjCheckboxLayout );
    checkboxLayout->addWidget ( m_ObjectsGroupBox );
    checkboxLayout->addStretch();

    //signal slot connections of load and delete Buttons
    connect ( loadObjectButton,SIGNAL ( clicked() ),this,SLOT ( loadObjectDialog() ) );
    connect ( m_DeleteObjectButton,SIGNAL ( clicked() ),this,SLOT ( deleteObject() ) );

    mainLayout->setSpacing ( 0 );
    setLayout ( mainLayout );

    //m_Timer=Timer ( ProfilerEntry::CODE_SEGMENT,"GUI Thread","OR Display" ); // TODO

    //m_GlImageWidget->addVectorObject ( VectorObject2D ( Circle2D ( 320,240,100 ).vertices(),1,0,0,1.0 ) );
    //m_GlImageWidget->addVectorObject ( VectorObject2D ( Line2D ( Point2D ( 220,140 ), Point2D ( 420,340 ) ).vertices(),0,1,0,1.0 ) );
    //m_GlImageWidget->addVectorObject ( VectorObject2D ( Box2D<> ( 220,140,420,340 ).vertices(),0,0,1,1.0 ) );
    //m_GlImageWidget->setColorImage ( new ColorImageRGB8 ( 640,480 ) );

    m_SelectedRow = 0;

    m_ORCommandPublisher = nh->advertise<or_msgs::OrCommand>("/or/commands", 10);
    m_Ready = true;
}

THIS::~THIS()
{
}

void THIS::grabImage()
{
    if(m_Ready)
    {
        or_msgs::OrCommand commandMsg;
        commandMsg.command = ORControlModule::GrabSingleImage;
        commandMsg.int_value = m_CameraId; // TODO handle camera id in receiving node - use ros topics for cameraid
        m_ORCommandPublisher.publish(commandMsg);
    }
}

void THIS::loadImage()
{
    if(m_Ready)
    {
        QStringList files = QFileDialog::getOpenFileNames ( this, tr ( "Load Image" ), m_LastOpenPath );

        for ( int i = 0; i < files.size(); i++ )
        {
            QString file=files.at ( i );
            QDir fileDir ( file );
            fileDir.cdUp();
            m_LastOpenPath = fileDir.absolutePath();

            or_msgs::OrCommand commandMsg;
            commandMsg.command = ORControlModule::LoadSingleImage;
            commandMsg.string_value = file.toStdString();
            m_ORCommandPublisher.publish(commandMsg);
        }
    }
}

void THIS::startLoop()
{
    if(m_Ready)
    {
        or_msgs::OrCommand commandMsg;
        commandMsg.command = ORControlModule::StartRecognitionLoop;
        commandMsg.int_value = m_CameraId;
        m_ORCommandPublisher.publish(commandMsg);
    }
}

void THIS::stopLoop()
{
    if(m_Ready)
    {
        or_msgs::OrCommand commandMsg;
        commandMsg.command = ORControlModule::StopRecognitionLoop;
        m_ORCommandPublisher.publish(commandMsg);
    }
}

void THIS::addCheckBox ( QBoxLayout* checkboxLayout, std::map<std::string,QCheckBox*>& CBMap, std::string id, std::string label, bool checked )
{
    CBMap[id] = new QCheckBox();
    CBMap[id]->setText ( label.c_str() );
    checkboxLayout -> addWidget ( CBMap[id] );
    CBMap[id]->setChecked ( checked );
    connect ( CBMap[id], SIGNAL ( stateChanged ( int ) ), this ,SLOT ( optionsChanged() ) );
}

// TODO
//void THIS::processMessage ( Message* message )
//{
//  m_Timer.startMeasure();
//  switch ( message->getType() )
//  {

//    case MessageTypes::IMAGE_M:
//    {
//      ImageM* castMessage = Message::castTo<ImageM> ( message );
//      if ( castMessage->getSourceId() == ImageSources::ORPrimary )
//      {
//        m_GlImageWidget->clearForms();
//        std::list< VectorObject2D >::iterator vectorObjectIt;
//        std::list< VectorObject2D > vectorObjects = castMessage->getVectorObjects();
//        for ( vectorObjectIt = vectorObjects.begin(); vectorObjectIt != vectorObjects.end(); vectorObjectIt++ )
//        {
//          m_GlImageWidget->addVectorObject ( *vectorObjectIt );
//        }
//        switch ( castMessage->getColorFormat() )
//        {
//          case ImageGrabber::RGB8:
//            TRACE_SYSTEMINFO( "Received image" )
//                m_GlImageWidget->setColorImage ( castMessage->getRgbImage() );
//            break;
//          case ImageGrabber::Y8UV8:
//            m_GlImageWidget->setColorImage ( castMessage->getGrayLevelImage(), castMessage->getUvImage() );
//            break;
//          default:
//            break;
//        }
//      }
//    }
//    break;

//    case MessageTypes::OR_MATCH_RESULT_M:
//    {
//      ORMatchResultM* castMessage = Message::castTo<ORMatchResultM> ( message );
//      m_SceneKeyPoints = *( castMessage->getSceneKeyPoints() );
//			m_BoundingBoxes = castMessage->getBoundingBoxes();
//      std::vector<MatchResult*> matchResultsPtr =  castMessage->getMatchResults();
//      m_MatchResults.clear();
//      m_MatchResults.reserve( matchResultsPtr.size() );
//      for ( unsigned i=0; i<matchResultsPtr.size(); i++ )
//      {
//        m_MatchResults.push_back( *(matchResultsPtr[i]) );
//      }
//      updateDisplay();
//      m_GlImageWidget->setColorImage( castMessage->getSceneImage() );

//      initBinChooser();
//    }
//    break;

//    default:
//      break;
//  }
//  m_Timer.submit();
//}

void THIS::setCameraId( ImageSources::SourceId cameraId )
{
    //   TRACE_INFO( Tracer::toString(cameraId) );
    m_CameraId=cameraId;
}

void THIS::initBinChooser()
{
    //update bin chooser box for stage2
    for(unsigned int i=0;i<m_MatchResults.size();++i)
    {
        //Update available bins for stage2
        binChooserComboBox->clear();

        int maxBinValue = 0;
        int maxBinIndex = 0;
        for(unsigned int j=0;j<m_MatchResults[i].stage2Matches.size();++j)
        {
            std::string sIndex;
            std::stringstream outIndex;
            outIndex << j;
            sIndex = outIndex.str();

            int binValue = m_MatchResults[i].stage2Matches[j].size();
            std::string sValue;
            std::stringstream outValue;
            outValue << binValue;
            sValue = outValue.str();

            if(maxBinValue<binValue)
            {
                maxBinValue=binValue;
                maxBinIndex=j;
            }

            binChooserComboBox->addItem(QString(sIndex.c_str()).append(" (").append(sValue.c_str()).append(")"));
            binChooserComboBox->setEnabled(binValue>0);
        }
        binChooserComboBox->setCurrentIndex(maxBinIndex);
        binChooserComboBox->setItemText(maxBinIndex,(binChooserComboBox->currentText()).append(QString("*")));
    }
}

void THIS::cellClicked ( int row, int column )
{
    m_DeleteObjectButton->setEnabled ( true );

    //Highlight clicked Cell
    m_ObjectsList->setSortingEnabled ( false );
    QColor rowColor ( 235,235,235 );
    for ( int i=0;i<m_ObjectsList->rowCount();++i )
    {
        if ( m_ObjectsList->item ( i,0 )->backgroundColor() !=QColor ( 0,0,0 ) )
        {
            m_ObjectsList->item ( i,0 )->setBackgroundColor ( rowColor );
        }
        m_ObjectsList->item ( i,1 )->setBackgroundColor ( rowColor );
        m_ObjectsList->item ( i,2 )->setBackgroundColor ( rowColor );
    }

    QColor selColor ( 119,136,153 );
    if ( m_ObjectsList->item ( row,0 )->backgroundColor() !=QColor ( 0,0,0 ) )
    {
        m_ObjectsList->item ( row,0 )->setBackgroundColor ( selColor );
    }
    m_ObjectsList->item ( row,1 )->setBackgroundColor ( selColor );
    m_ObjectsList->item ( row,2 )->setBackgroundColor ( selColor );

    m_SelectedRow = row;

    m_ObjectsList->setSortingEnabled ( true );

    //Draw updated matches
    updateDisplay();
}

void THIS::optionsChanged()
{
    updateDisplay();
}

void THIS::deleteObject()
{
    if ( ( m_SelectedRow >= 0 ) && ( m_SelectedRow < m_ObjectsList->rowCount() ) )
    {
        string name = m_ObjectsList->item ( m_SelectedRow, 1 )->text().toStdString();

        if(m_Ready)
        {
            or_msgs::OrCommand commandMsg;
            commandMsg.command = ORControlModule::UnloadObject;
            commandMsg.string_value = name;
            m_ORCommandPublisher.publish(commandMsg);
        }
    }
}

void THIS::updateDisplay()
{
    m_GlImageWidget->clearForms();

    if ( m_OptionCheckBoxes["sceneAreas"]->isChecked() )
    {
        for ( unsigned i=0; i < m_SceneKeyPoints.size(); i++ )
        {
            std::vector<Point2D> bBox1 = m_SceneKeyPoints[ i ].getBoundingBox();
            m_GlImageWidget->addVectorObject( VectorObject2D( bBox1, 0, 0, 0, 5.0, VectorObject2D::Lines ) );
            m_GlImageWidget->addVectorObject( VectorObject2D( bBox1, 1, 1, 0.5, 3.0, VectorObject2D::Lines ) );
        }
    }

    if ( m_OptionCheckBoxes["sceneCircles"]->isChecked() )
    {
        for ( unsigned i=0; i < m_SceneKeyPoints.size(); i++ )
        {
            std::vector<Point2D> circle = m_SceneKeyPoints[ i ].getCircle();
            m_GlImageWidget->addVectorObject( VectorObject2D( circle, 0, 0, 0, 5.0, VectorObject2D::Lines ) );
            m_GlImageWidget->addVectorObject( VectorObject2D( circle, 1, 1, 1, 3.0, VectorObject2D::Lines ) );
        }
    }

    if ( m_OptionCheckBoxes["sceneArrows"]->isChecked() )
    {
        for ( unsigned i=0; i < m_SceneKeyPoints.size(); i++ )
        {
            std::vector<Point2D> arrow1 = m_SceneKeyPoints[ i ].getCenterArrow();
            m_GlImageWidget->addVectorObject( VectorObject2D( arrow1, 0, 0, 0, 3.0, VectorObject2D::Lines ) );
            m_GlImageWidget->addVectorObject( VectorObject2D( arrow1, 0.5, 1, 1, 1.0, VectorObject2D::Lines ) );
        }
    }

    if ( m_OptionCheckBoxes["boundingBoxes"]->isChecked() )
    {
        for(unsigned i=0;i<m_BoundingBoxes.size();++i)
        {
            m_GlImageWidget->addVectorObject( VectorObject2D( m_BoundingBoxes[i].vertices(), 1, 1, 0, 1.0, VectorObject2D::Lines ) );
        }
    }

    for ( unsigned int i=0; i<m_MatchResults.size(); i++ )
    {
        MatchResult& matchResult = m_MatchResults[i];

        int objectRow = m_ObjectRows[ matchResult.objectName ];
        if ( m_ObjectsList->item ( objectRow, 0 )->checkState() ==Qt::Unchecked )
        {
            continue;
        }

        //FIXME: abhngig von objekt -> muss upgedatet werden
        if ( m_OptionCheckBoxes["sceneArrowsWithinOutline"]->isChecked() )
        {
            for ( unsigned i=0; i < matchResult.sceneKeyPointsWithinOutline.size(); i++ )
            {
                std::vector<Point2D> arrow1 = matchResult.sceneKeyPointsWithinOutline.at(i).getCenterArrow();
                m_GlImageWidget->addVectorObject( VectorObject2D( arrow1, 1, 1, 0, 1.0, VectorObject2D::Lines ) );
            }
        }

        if ( m_OptionCheckBoxes["outline"]->isChecked())
        {
            std::vector<Point2D> objectOutlineScene;
            matchResult.homography.transform( matchResult.outline, objectOutlineScene );
            m_GlImageWidget->addVectorObject( VectorObject2D( objectOutlineScene, 1, 0, 0, 3.0, VectorObject2D::Lines ) );
            m_GlImageWidget->addVectorObject( VectorObject2D( objectOutlineScene, 0, 0, 0, 1.0, VectorObject2D::Lines ) );

            std::vector<Point2D> objectbBoxScene;
            matchResult.homography.transform( matchResult.bBox, objectbBoxScene );
            m_GlImageWidget->addVectorObject( VectorObject2D( objectbBoxScene, 0, 0, 0, 3.0, VectorObject2D::Lines ) );
            m_GlImageWidget->addVectorObject( VectorObject2D( objectbBoxScene, 1, 1, 1, 1.0, VectorObject2D::Lines ) );

            std::vector<Point2D> centerCircleObjPoints;
            std::vector<Point2D> centerCircleScenePoints;
            Circle2D centerCircle( matchResult.center, 10 );
            centerCircleObjPoints = centerCircle.vertices();
            matchResult.homography.transform( centerCircleObjPoints, centerCircleScenePoints );
            m_GlImageWidget->addVectorObject( VectorObject2D( centerCircleScenePoints, 1, 0, 0, 3.0, VectorObject2D::Lines ) );
        }

        if ( m_OptionCheckBoxes["stage1"]->isChecked() )
        {
            updateMatches( matchResult, matchResult.stage1Matches );
        }
        else if ( m_OptionCheckBoxes["stage2"]->isChecked() )
        {
            //Update matches for chosen bin
            int index = binChooserComboBox->currentIndex();

            if(index>=0)
            {
                updateMatches( matchResult, matchResult.stage2Matches[binChooserComboBox->currentIndex()] );
            }
        }
        else if ( m_OptionCheckBoxes["stage3"]->isChecked() )
        {
            updateMatches( matchResult, matchResult.stage3Matches );
        }
    }

    m_GlImageWidget->updateGL();
}

void THIS::updateMatches( MatchResult& matchResult, std::list<KeyPointMatch> &matches )
{
    if ( m_OptionCheckBoxes["areas"]->isChecked() )
    {
        // draw scene keyPoint areas
        if ( m_OptionCheckBoxes["scene"]->isChecked() )
        {
            for ( std::list<KeyPointMatch>::iterator match = matches.begin(); match != matches.end(); match++ )
            {
                std::vector<Point2D> bBox1 = m_SceneKeyPoints[ matchResult.keyPointIndexMap.at( match->index1 ) ].getBoundingBox();
                m_GlImageWidget->addVectorObject( VectorObject2D( bBox1, 1, 1, 0, 1.0, VectorObject2D::Lines ) );
            }
        }

        if ( m_OptionCheckBoxes["object"]->isChecked())
        {
            for ( std::list<KeyPointMatch>::iterator match = matches.begin(); match != matches.end(); match++ )
            {
                std::vector<Point2D> bBox2 = matchResult.objectKeyPoints[ match->index2 ].getBoundingBox();
                std::vector<Point2D> bBox2Trans;
                matchResult.homography.transform( bBox2, bBox2Trans );
                m_GlImageWidget->addVectorObject( VectorObject2D( bBox2Trans, 0, 1, 1, 1.0, VectorObject2D::Lines ) );
            }
        }

    }

    if ( m_OptionCheckBoxes["arrows"]->isChecked() )
    {
        if ( m_OptionCheckBoxes["scene"]->isChecked() )
        {
            for ( std::list<KeyPointMatch>::iterator match = matches.begin(); match != matches.end(); match++ )
            {
                std::vector<Point2D> arrow1 = m_SceneKeyPoints[ matchResult.keyPointIndexMap.at( match->index1 ) ].getCenterArrow();
                m_GlImageWidget->addVectorObject( VectorObject2D( arrow1, 1, 1, 0, 1.0, VectorObject2D::Lines ) );
            }
        }
        if ( m_OptionCheckBoxes["object"]->isChecked())
        {
            for ( std::list<KeyPointMatch>::iterator match = matches.begin(); match != matches.end(); match++ )
            {
                std::vector<Point2D> arrow2 = matchResult.objectKeyPoints[ match->index2 ].getCenterArrow();
                std::vector<Point2D> arrow2Trans;
                matchResult.homography.transform( arrow2, arrow2Trans );
                m_GlImageWidget->addVectorObject( VectorObject2D( arrow2Trans, 0, 1, 1, 1.0, VectorObject2D::Lines ) );
            }
        }
    }

    if ( m_OptionCheckBoxes["lines"]->isChecked())
    {
        for ( std::list<KeyPointMatch>::iterator match = matches.begin(); match != matches.end(); match++ )
        {
            std::vector<Point2D> line;
            line.push_back( m_SceneKeyPoints[ matchResult.keyPointIndexMap.at( match->index1 ) ].position() );
            line.push_back( matchResult.homography.transform( matchResult.objectKeyPoints[ match->index2 ].position() ) );
            m_GlImageWidget->addVectorObject( VectorObject2D( line, 1, 1, 1, 1.0, VectorObject2D::Lines ) );
        }
    }
}


void THIS::loadObjectDialog()
{
    QStringList files = QFileDialog::getOpenFileNames ( this, tr ( "Load Object" ), m_LastOpenPath,"Objectproperties (*.objprop)" );

    for ( int i = 0; i < files.size(); i++ )
    {
        //        QString file=files.at ( i );
        //        QDir fileDir ( file );
        //        fileDir.cdUp();
        //        m_LastOpenPath = fileDir.absolutePath();

        QFileInfo fileInfo( files.at ( i ) );
        QString absolutePath = fileInfo.absoluteFilePath();

        std::cerr << "m_LastOpenPath: " << absolutePath.toStdString() << std::endl;

        loadObject( absolutePath.toStdString() );
    }
}

void THIS::loadObject ( string file )
{
    // file contains absolut path, here the directory is removed to retain only <objname>.objprops
    int slashPos = file.find_last_of( '/' );
    // here the file extension .objprops is removed
    file = file.substr( slashPos, file.length() - slashPos - 8 );

    std::cerr << "Loading object file: " << file << std::endl;

    if(m_Ready)
    {
        or_msgs::OrCommand commandMsg;
        commandMsg.command = ORControlModule::LoadObject;
        commandMsg.string_value = file;
        m_ORCommandPublisher.publish(commandMsg);
    }
}

void THIS::updateObjectTable(std::vector<std::string> names, std::vector<std::string> types)
{
    ROS_INFO_STREAM("Updating Object Table, new size: " << names.size());

    m_ObjectsList->setSortingEnabled ( false );

    //save old state of checkboxes and recognition
    QVector< std::string > m_UncheckedObjects;
    QVector< std::string > m_RecognizedObjects;
    for ( int i=0;i<m_ObjectsList->rowCount();++i )
    {
        if ( m_ObjectsList->item ( i,0 )->checkState() ==Qt::Unchecked )
        {
            m_UncheckedObjects.push_back ( m_ObjectsList->item ( i,1 )->text().toStdString() );
        }
        if ( m_ObjectsList->item ( i,0 )->backgroundColor() == ( QColor ( 0,0,0 ) ) )
        {
            m_RecognizedObjects.push_back ( m_ObjectsList->item ( i,1 )->text().toStdString() );
        }
    }

    m_ObjectsList->setRowCount ( 0 );
    m_ObjectRows.clear();

    QColor rowColor ( 235,235,235 );

    for(unsigned pos = 0; pos < names.size(); pos++)
    {
        std::string name = names.at(pos);
        int indexRow = m_ObjectsList->rowCount();
        m_ObjectRows[name] = indexRow;
        m_ObjectsList->insertRow ( indexRow );

        //create item with checkbox
        QTableWidgetItem* item = new QTableWidgetItem();
        item->setBackground ( rowColor );
        m_ObjectsList->setItem ( indexRow,0,item );

        //keep state of checkboxes
        if ( m_UncheckedObjects.contains ( names.at(pos).c_str() ) )
        {
            m_ObjectsList->item ( indexRow,0 )->setCheckState ( Qt::Unchecked );
        }
        else
        {
            m_ObjectsList->item ( indexRow,0 )->setCheckState ( Qt::Checked );
        }
        //highlight recognized table entries
        if ( m_RecognizedObjects.contains ( names.at(pos).c_str() ) )
        {
            m_ObjectsList->item ( indexRow,0 )->setBackgroundColor ( QColor ( 0,0,0 ) );
        }

        //create item with object name
        QString objectName = QString::fromStdString ( name );
        item = new QTableWidgetItem ( objectName );
        item->setBackground ( rowColor );
        m_ObjectsList->setItem ( indexRow,1,item );

        //create item with object type
        item = new QTableWidgetItem ( QString ( types.at(pos).c_str() ) );
        item->setBackground ( rowColor );
        item->setTextAlignment ( Qt::AlignCenter );
        m_ObjectsList->setItem ( indexRow,2,item );

        //select row in table that was just added
        m_ObjectsList->selectRow ( indexRow );
        m_ObjectsList->scrollToItem ( m_ObjectsList->item ( indexRow,1 ), QAbstractItemView::EnsureVisible );
    }
    m_ObjectsList->setSortingEnabled ( true );
    m_ObjectsList->sortItems ( 1, Qt::AscendingOrder );
    m_ObjectsList->resizeColumnToContents ( 0 );
    m_DeleteObjectButton->setEnabled ( false );
    m_ObjectsGroupBox->setTitle ( "Objects ("+QString::number ( m_ObjectsList->rowCount() ) +")" );

}

#undef THIS
