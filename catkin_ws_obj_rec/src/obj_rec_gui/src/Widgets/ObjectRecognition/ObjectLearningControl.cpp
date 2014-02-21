/*******************************************************************************
*  ObjectLearningControl.cpp
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

#include <QPushButton>
#include <QHBoxLayout>
#include <QString>
#include <QFileDialog>
#include <QSlider>
#include <QLabel>
#include <QGroupBox>
#include <QComboBox>
#include <QCheckBox>

#include <or_msgs/OrLearnCommand.h>
#include "Modules/ORLearningModule.h"

#include "../ImageMessagesDisplay/ImageSourceSelector.h"

#include "ObjectLearningControl.h"

#define THIS ObjectLearningControl

using namespace puma2;

THIS::THIS ( ros::NodeHandle *nh, QWidget *parent ) : QWidget ( parent )
{
    m_Ready = false;

    //create layouts
    QGridLayout* mainLayout = new QGridLayout();

    //Threshold slider
    QSlider* thresholdSlider = new QSlider ( Qt::Horizontal );
    thresholdSlider->setRange ( 0, 100 );
    thresholdSlider->setTickPosition ( QSlider::TicksBothSides );
    QLabel* thresholdLabel = new QLabel();
    QHBoxLayout* thresholdSliderLayout = new QHBoxLayout();
    thresholdSliderLayout->addWidget ( thresholdSlider );
    thresholdSliderLayout->addWidget ( thresholdLabel );
    QGroupBox* thresholdSliderGroupBox = new QGroupBox ( "Background deletion threshold" );
    thresholdSliderGroupBox->setLayout ( thresholdSliderLayout );
    mainLayout->addWidget ( thresholdSliderGroupBox, 0, 0, 2, 1 );

    //Open Radius slider
    QSlider* openSlider = new QSlider ( Qt::Horizontal );
    openSlider->setRange ( 0, 50 );
    openSlider->setTickPosition ( QSlider::TicksBothSides );
    QLabel* openLabel = new QLabel();
    QHBoxLayout* openSliderLayout = new QHBoxLayout();
    openSliderLayout->addWidget ( openSlider );
    openSliderLayout->addWidget ( openLabel );
    QGroupBox* openSliderGroupBox = new QGroupBox ( "Mask open radius" );
    openSliderGroupBox->setLayout ( openSliderLayout );
    mainLayout->addWidget ( openSliderGroupBox, 0, 1, 2, 1 );

    //Open Radius slider
    QSlider* borderSlider = new QSlider ( Qt::Horizontal );
    borderSlider->setRange ( 0, 50 );
    borderSlider->setTickPosition ( QSlider::TicksBothSides );
    QLabel* borderLabel = new QLabel();
    QHBoxLayout* borderSliderLayout = new QHBoxLayout();
    borderSliderLayout->addWidget ( borderSlider );
    borderSliderLayout->addWidget ( borderLabel );
    QGroupBox* borderSliderGroupBox = new QGroupBox ( "Additional border" );
    borderSliderGroupBox->setLayout ( borderSliderLayout );
    mainLayout->addWidget ( borderSliderGroupBox, 0, 2, 2, 1 );

    //"Single segment" check box
    QCheckBox* isolateCheckbox = new QCheckBox ( "Single segment" );
    mainLayout->addWidget ( isolateCheckbox, 1, 3 );

    //Image source selector
    ImageSourceSelector* sourceSelector = new ImageSourceSelector ( ImageSources::TopCamera, this, ImageSources::TopCamera, ImageSources::SourceId ( 99 ) );
    setCameraId ( ImageSources::TopCamera );
    connect ( sourceSelector, SIGNAL ( sourceSelected ( ImageSources::SourceId ) ), this, SLOT ( setCameraId ( ImageSources::SourceId ) ) );
    mainLayout->addWidget ( sourceSelector, 0, 3 );

    //Buttons
    grabBackgroundButton = new QPushButton ( "Grab background" );
    grabForegroundButton = new QPushButton ( "Grab foreground" );

    mainLayout->addWidget ( grabBackgroundButton, 0, 4 );
    mainLayout->addWidget ( grabForegroundButton, 1, 4 );

    loadBackgroundButton = new QPushButton ( "Load background" );
    loadForegroundButton = new QPushButton ( "Load foreground" );

    mainLayout->addWidget ( loadBackgroundButton, 0, 5 );
    mainLayout->addWidget ( loadForegroundButton, 1, 5 );

    mainLayout->setColumnStretch ( 0, 2 );
    mainLayout->setColumnStretch ( 1, 2 );
    mainLayout->setColumnStretch ( 2, 1 );
    mainLayout->setColumnStretch ( 3, 1 );

    setLayout ( mainLayout );

    connect ( grabBackgroundButton , SIGNAL ( clicked() ), this, SLOT ( grabBackgroundImage() ) );
    connect ( grabForegroundButton , SIGNAL ( clicked() ), this, SLOT ( grabForegroundImage() ) );

    connect ( loadBackgroundButton , SIGNAL ( clicked() ), this, SLOT ( loadBackgroundImage() ) );
    connect ( loadForegroundButton , SIGNAL ( clicked() ), this, SLOT ( loadForegroundImage() ) );

    connect ( thresholdSlider, SIGNAL ( valueChanged ( int ) ), thresholdLabel, SLOT ( setNum ( int ) ) );
    connect ( thresholdSlider, SIGNAL ( valueChanged ( int ) ), this, SLOT ( setThreshold ( int ) ) );

    connect ( openSlider, SIGNAL ( valueChanged ( int ) ), openLabel, SLOT ( setNum ( int ) ) );
    connect ( openSlider, SIGNAL ( valueChanged ( int ) ), this, SLOT ( setOpenRadius ( int ) ) );

    connect ( borderSlider, SIGNAL ( valueChanged ( int ) ), borderLabel, SLOT ( setNum ( int ) ) );
    connect ( borderSlider, SIGNAL ( valueChanged ( int ) ), this, SLOT ( setBorderSize ( int ) ) );

    connect ( isolateCheckbox, SIGNAL ( stateChanged ( int ) ), this, SLOT ( setIsolateLargestSegment ( int ) ) );

    thresholdSlider->setValue ( 25 );
    thresholdSlider->setTracking ( false );

    openSlider->setValue ( 15 );
    openSlider->setTracking ( false );

    borderSlider->setValue ( 5 );
    borderSlider->setTracking ( false );

    isolateCheckbox->setCheckState ( Qt::Checked );

    m_LastImageFolder = "/images/ObjectRecognition"; // TODO check if the path is correct
    // m_LastImageFolder = ( getAppPath() + "/images/ObjectRecognition" ).c_str(); // TODO original was like this

    m_ORLearnCommandPublisher = nh->advertise<or_msgs::OrLearnCommand>("/or/learn_commands", 10);
    m_Ready = true;
}


THIS::~THIS() {}

void THIS::setCameraId ( ImageSources::SourceId cameraId )
{
    //TRACE_INFO ( Tracer::toString ( cameraId ) ); // TODO use ROS
    m_CameraId = cameraId;
}

void THIS::setIsolateLargestSegment ( int state )
{
    or_msgs::OrLearnCommand learnCommandMsg;
    learnCommandMsg.command = ORLearningModule::SetIsolateLargestSegment;

    if ( state == Qt::Checked ) {
        learnCommandMsg.string_value = "true";
    } else if ( state == Qt::Unchecked ) {
        learnCommandMsg.string_value = "false";
    }

    if(m_Ready)
    {
        m_ORLearnCommandPublisher.publish(learnCommandMsg);
    }
}


void THIS::setThreshold ( int value )
{
    if(m_Ready)
    {
        or_msgs::OrLearnCommand learnCommandMsg;
        learnCommandMsg.command = ORLearningModule::SetDifferenceThreshold;
        learnCommandMsg.int_value = value;
        m_ORLearnCommandPublisher.publish(learnCommandMsg);
    }
}

void THIS::setOpenRadius ( int value )
{
    if(m_Ready)
    {
        or_msgs::OrLearnCommand learnCommandMsg;
        learnCommandMsg.command = ORLearningModule::SetOpenRadius;
        learnCommandMsg.int_value = value;
        m_ORLearnCommandPublisher.publish(learnCommandMsg);
    }
}

void THIS::setBorderSize ( int value )
{
    if(m_Ready)
    {
        or_msgs::OrLearnCommand learnCommandMsg;
        learnCommandMsg.command = ORLearningModule::SetBorderSize;
        learnCommandMsg.int_value = value;
        m_ORLearnCommandPublisher.publish(learnCommandMsg);
    }
}

void THIS::grabBackgroundImage()
{
    if(m_Ready)
    {
        or_msgs::OrLearnCommand learnCommandMsg;
        learnCommandMsg.command = ORLearningModule::GrabBackgroundImage;
        //learnCommandMsg.int_value = m_CameraId; // TODO
        m_ORLearnCommandPublisher.publish(learnCommandMsg);
    }
}

void THIS::grabForegroundImage()
{
    if(m_Ready)
    {
        or_msgs::OrLearnCommand learnCommandMsg;
        learnCommandMsg.command = ORLearningModule::GrabForegroundImage;
        //learnCommandMsg.int_value = m_CameraId; // TODO
        m_ORLearnCommandPublisher.publish(learnCommandMsg);
    }
}

void THIS::loadBackgroundImage()
{
    QStringList files = QFileDialog::getOpenFileNames ( this, tr ( "Load Image" ), m_LastOpenPath );

    for ( int i = 0; i < files.size(); i++ )
    {
        QString file=files.at ( i );
        QDir fileDir ( file );
        fileDir.cdUp();
        m_LastOpenPath = fileDir.absolutePath();

        if(m_Ready)
        {
            or_msgs::OrLearnCommand learnCommandMsg;
            learnCommandMsg.command = ORLearningModule::LoadBackgroundImage;
            learnCommandMsg.string_value = file.toStdString();
            m_ORLearnCommandPublisher.publish(learnCommandMsg);
        }
    }
}

void THIS::loadForegroundImage()
{
    QStringList files = QFileDialog::getOpenFileNames ( this, tr ( "Load Image" ), m_LastOpenPath );

    for ( int i = 0; i < files.size(); i++ )
    {
        QString file=files.at ( i );
        QDir fileDir ( file );
        fileDir.cdUp();
        m_LastOpenPath = fileDir.absolutePath();

        if(m_Ready)
        {
            or_msgs::OrLearnCommand learnCommandMsg;
            learnCommandMsg.command = ORLearningModule::LoadForegroundImage;
            learnCommandMsg.string_value = file.toStdString();
            m_ORLearnCommandPublisher.publish(learnCommandMsg);
        }
    }
}


#undef THIS
