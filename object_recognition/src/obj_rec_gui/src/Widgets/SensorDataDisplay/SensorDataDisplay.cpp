/*******************************************************************************
 *  SensorDataDisplay.cpp
 *
 *  (C) 2006 AG Aktives Sehen <agas@uni-koblenz.de>
 *           Universitaet Koblenz-Landau
 *
 *  Additional information:
 *  $Id: SensorDataDisplay.cpp 45149 2011-06-06 12:55:08Z cnluzon $
 *******************************************************************************/


#include <QGridLayout>
#include <QBoxLayout>
#include <QLabel>
#include <QPixmap>
#include <QtGui>
#include <vector>
#include <vector>
#include <QDialog>
#include <QComboBox>


#include <QPushButton>

//#include "Messages/SceneGraphM.h"
//#include "Messages/GetLaserscanDirectedM.h"

#include "SensorDataGLWidget.h"
#include "SensorDataDisplay.h"

#include "PainterPlugin.h"
#include "PainterCheckBox.h"
//#include "RobotPainter.h"
//#include "SonarPainter.h"
//#include "ParticlePainter.h"
//#include "ParticleGridPainter.h"
//#include "PathPainter.h"
//#include "POIPainter.h"
//#include "MapPainter.h"
//#include "PersonPainter.h"
//#include "SilhouettePainter.h"
#include "NewLaserDataPainter.h"
//#include "RobotPosesPainter.h"
//#include "LaserScan3DPainter.h"
//#include "RGBDepthPainter.h"
//#include "SceneGraphPainter.h"
//#include "ImagePainter.h"
//#include "TofPainter.h"
//#include "SkeletonPainter.h"
//#include "PersonParticlePainter.h"

#define THIS SensorDataDisplay

QSize THIS::minimumSizeHint() const
{
  return QSize ( 200, 100 );
}
QSize THIS::sizeHint() const
{
  return QSize ( 500, 400 );
}

THIS::THIS ( bool useDefaultPlugins, QWidget *parent ) : QWidget ( parent )
{

  // create the openGL widget
  m_GlWidget = new SensorDataGLWidget ( this );

  // create the painter plugins & options dialog box
  m_OptionsLayout = new QVBoxLayout();

	if ( useDefaultPlugins )
	{
		//NOTE: ordering of this list is important for alpha blending to work correctly
//                addPainter ( new ImagePainter(), true );
//                addPainter ( new LaserScan3DPainter(), true );
//                addPainter ( new RGBDepthPainter(), true );
                addPainter ( new NewLaserDataPainter(), true );
//                addPainter ( new MapPainter(), true );
//                addPainter ( new TofPainter(), true );
//                addPainter ( new SceneGraphPainter(), true );
//                addPainter ( new POIPainter(), true );
//                addPainter ( new SonarPainter(), false );
//                addPainter ( new PathPainter(), true );
//                addPainter ( new PersonPainter(), true );
//                addPainter ( new SilhouettePainter(), true );
//                addPainter ( new SkeletonPainter(), true );
//                addPainter ( new ParticlePainter(), false );
//                addPainter ( new PersonParticlePainter(), true );
//                addPainter ( new RobotPosesPainter(), false );
//                addPainter ( new ParticleGridPainter(), false );
//                addPainter ( new RobotPainter(), false );
	}

  m_OptionsDialog = new QDialog();
  m_OptionsDialog->setWindowTitle ( "Display Options" );
  m_OptionsDialog->setLayout ( m_OptionsLayout );
  m_OptionsDialog->setFocusPolicy(Qt::NoFocus);


  //Drop-down list
  m_NodeSelector = new QComboBox( this );
  m_NodeSelectorInitialized = false;
  m_NodeSelector->setFocusPolicy(Qt::NoFocus);
  connect( m_NodeSelector, SIGNAL(activated(const QString&)), m_GlWidget, SLOT(followNode(const QString&)));

  //create move button...
  QPushButton* buttonCameraMove = new QPushButton ( QString::fromUtf8("360° View"), this );
  connect ( buttonCameraMove, SIGNAL ( pressed() ), m_GlWidget, SLOT ( performCameraMove() ) );
  buttonCameraMove->setFocusPolicy(Qt::NoFocus);


  //create options button...
  QPushButton* buttonOptions = new QPushButton ( "Options", this );
  connect ( buttonOptions, SIGNAL ( pressed() ), m_OptionsDialog, SLOT ( show() ) );
buttonOptions->setFocusPolicy(Qt::NoFocus);


  // create main layout
  QVBoxLayout* vLayout = new QVBoxLayout( );
  vLayout->addWidget ( m_GlWidget );

  QHBoxLayout* hLayout = new QHBoxLayout( );
//  hLayout->addWidget ( checkBoxFixedLookPosition );
  hLayout->addWidget ( m_NodeSelector );
  hLayout->addWidget ( buttonOptions );
  hLayout->addWidget ( buttonCameraMove );

  vLayout->addLayout( hLayout );

/*  gridLayout->setRowStretch ( 0, 2 );
  gridLayout->setRowStretch ( 1, 0 );*/
  setLayout ( vLayout );

  this->setFocus();
}


void THIS::addPainter ( PainterPlugin* painter, bool visible )
{
  m_GlWidget->addPainter ( painter );
  PainterCheckBox* painterCB = new PainterCheckBox ( painter );
  m_OptionsLayout->addWidget ( painterCB );
  connect ( painterCB , SIGNAL ( changed() ), m_GlWidget, SLOT ( updateGL() ) );
  painterCB->setChecked ( visible );
}


//void THIS::processMessage ( Message* newMessage )
//{
//  switch ( newMessage->getType() )
//  {

//    case MessageTypes::STOP_MESSAGE:
//      m_OptionsDialog->close();
//      break;

//    case MessageTypes::SCENE_GRAPH_M:
//    {
//      if ( !m_NodeSelectorInitialized )
//      {
////         TRACE_INFO( "initializing" );
//        if ( SceneGraphM* message = Message::castTo<SceneGraphM> ( newMessage ) )
//        {
//          std::list<std::string> nodeNames=message->getSceneGraph().getSelectableNodeNames();
//          std::list<std::string>::iterator it = nodeNames.begin();
//          int i=0;
//          while ( it != nodeNames.end() )
//          {
//            m_NodeSelector->addItem( it->c_str() );
//            if ( *it == "Robot" )
//            {
//              m_NodeSelector->setCurrentIndex(i);
//            }
////             TRACE_INFO( "Adding " + *it );
//            it++;
//            i++;
//          }
//          m_NodeSelectorInitialized = true;
//        }
//      }
//      break;
//    }

//    default:
//      break;
//  }
//}

THIS::~THIS()
{
}
