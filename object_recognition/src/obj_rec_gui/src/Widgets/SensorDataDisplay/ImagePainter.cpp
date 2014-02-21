/*******************************************************************************
 *  ImagePainter.cpp
 *
 *  (C) 2007 AG Aktives Sehen <agas@uni-koblenz.de>
 *           Universitaet Koblenz-Landau
 *
 *  Additional information:
 *  $Id: $
 *******************************************************************************/

#include "ImagePainter.h"

#include "Messages/ImageStreamM.h"
#include "Messages/ImageM.h"
#include "Messages/SceneGraphM.h"

#include <QtOpenGL>
#include <GL/glut.h>


#define THIS ImagePainter

using namespace puma2;

THIS::THIS() : PainterPlugin( )
{
  setName ( "Image Painter" );

  m_ImageSources ["TopCameraImage"] = ImageSources::TopCameraStream;
  m_ImageSources ["KinectCameraImage"] = ImageSources::KinectCameraStream;
  m_ImageSources ["CenterCameraImage"] = ImageSources::BottomCameraStream;
  m_ImageSources ["BottomCameraImage"] = ImageSources::BottomCameraStream;
  m_ImageSources ["WiiMoteImage"] = ImageSources::WiiMoteStream;
}


THIS::~THIS()
{
}


void THIS::processMessage ( Message* newMessage )
{
  switch ( newMessage->getType() )
  {
    case MessageTypes::IMAGE_STREAM_M:
    {
      ImageStreamM* castMessage = Message::castTo<ImageStreamM> ( newMessage );
      if ( ( castMessage != 0 ) &&
             ( m_ImageSources.find ( m_NodeName ) != m_ImageSources.end() ) &&
             ( castMessage->getSourceId() == m_ImageSources[m_NodeName] )
         )
      {
        m_GLImagePainter.setColorImage ( castMessage->getRgbImage() );

        m_GLImagePainter.clearForms();
        //store vertices
        std::list< VectorObject2D >::iterator vectorObjectIt;
        std::list< VectorObject2D > vectorObjects = castMessage->getVectorObjects();
        for ( vectorObjectIt = vectorObjects.begin(); vectorObjectIt != vectorObjects.end(); vectorObjectIt++ )
        {
          m_GLImagePainter.addVectorObject ( *vectorObjectIt );
        }
        requestRedraw();
      }
      break;
    }


    case MessageTypes::IMAGE_M:
    {
      ImageM* castMessage = Message::castTo<ImageM> ( newMessage );
      if ( ( castMessage != 0 ) &&
             ( m_ImageSources.find ( m_NodeName ) != m_ImageSources.end() ) &&
             ( castMessage->getSourceId() == m_ImageSources[m_NodeName] )
         )
      {
        m_GLImagePainter.setColorImage ( castMessage->getRgbImage() );

        m_GLImagePainter.clearForms();
        //store vertices
        std::list< VectorObject2D >::iterator vectorObjectIt;
        std::list< VectorObject2D > vectorObjects = castMessage->getVectorObjects();
        for ( vectorObjectIt = vectorObjects.begin(); vectorObjectIt != vectorObjects.end(); vectorObjectIt++ )
        {
          m_GLImagePainter.addVectorObject ( *vectorObjectIt );
        }
        requestRedraw();
      }
      break;
    }


    case MessageTypes::SCENE_GRAPH_M:
    {
      if ( SceneGraphM* message = Message::castTo<SceneGraphM> ( newMessage ) )
      {
        if ( m_ImageSources.find ( m_NodeName ) != m_ImageSources.end() )
        {
          m_ImageToWorld = message->getSceneGraph().getTransformation ( m_NodeName, "World" );
          //TRACE_INFO ( m_ImageToWorld.niceString ( 2, "m_ImageToWorld" ) );
        }
      }
      break;
    }

    default:
      break;

  }
}


void THIS::paint ( float next2DLayer )
{
  if ( m_ImageSources.find ( m_NodeName ) != m_ImageSources.end() )
  {
    glDisable( GL_DEPTH_TEST );

    //TRACE_INFO ( "painting" );
    glPushMatrix();

    //transform normalized image coordinates to world coordinates
    double glMat[16];
    m_ImageToWorld.toColumnMajor ( glMat );
    glMultMatrixd ( glMat );

    //make everything bigger
    //glScalef ( 50000, 50000, 50000 );
    glScalef ( 500,500,500 );

    glTranslatef ( 0, 0, 1.0 );
    glScalef ( 2.0, 2.0, 2.0 );

    glPolygonMode ( GL_FRONT_AND_BACK, GL_LINE );
    //paint white border
    glColor3f ( 1.0, 1.0, 1.0 );
    glLineWidth( 1.0 );
    glBegin ( GL_POLYGON );
    glVertex3f ( -0.5, -0.5, 0.0 );
    glVertex3f ( -0.5, 0.5, 0.0 );
    glVertex3f ( 0.5, 0.5, 0.0 );
    glVertex3f ( 0.5, -0.5, 0.0 );
    glEnd();


    m_GLImagePainter.paintImage();
    m_GLImagePainter.paintVectorObjects();

    glPopMatrix();

    glEnable ( GL_DEPTH_TEST );
  }
}

void THIS::nodeSelected ( string nodeName )
{
  m_NodeName = nodeName;
  TRACE_INFO ( nodeName );
}



#undef THIS
