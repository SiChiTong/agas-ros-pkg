/*******************************************************************************
 *  MyPainter.cpp
 *
 *  (C) 2007 AG Aktives Sehen <agas@uni-koblenz.de>
 *           Universitaet Koblenz-Landau
 *
 *  Additional information:
 *  $Id: $
 *******************************************************************************/

#include "TofPainter.h"

#include "Messages/TofDataM.h"
#include "Messages/TofStatusM.h"
#include "Messages/SceneGraphM.h"
#include "Architecture/Config/Config.h"
#include "Architecture/Singleton/Clock.h"

#include <QtOpenGL>
#include <GL/glut.h>

#include<iostream>
using namespace std;

#define THIS TofPainter

THIS::THIS() : PainterPlugin( )
{
  setName ( "TOF Data" );

  m_Height=0;
  m_Width=0;
  m_Amplitudes=0;
  m_Depths=0;
  m_Points=0;

  m_SceneGraphInitialized=false;

  m_ColorFactor=1.0/Config::getFloat("TofPainter.fRedDistance");
  m_AmplitudeScaling=Config::getFloat("TofPainter.fAmplitudeScaling");
  m_AmplitudeThreshold=Config::getFloat("TofPainter.fAmplitudeThreshold");
  m_ViewMode=Config::getInt("TofPainter.iViewMode");
  m_DisplayDuration=Config::getInt("TofPainter.iDisplayDuration");
}


THIS::~THIS()
{
  delete[] m_Depths;
  delete[] m_Amplitudes;
  delete[] m_Points;
}


void THIS::processMessage( Message* newMessage )
{
  switch(newMessage->getType())
  {
  case MessageTypes::TOF_DATA_M:
    {
      if ( TofDataM* message = Message::castTo<TofDataM> ( newMessage ) )
      {
        m_LastDataTime = Clock::getInstance()->getTimestamp();

        if( message->getHeight() != m_Height || message->getWidth() != m_Width)
          initBuffers( message->getHeight(), message->getWidth() );

        memcpy( m_Depths, message->getDepths(),
                message->getHeight()*message->getWidth()*sizeof(float) );
        memcpy( m_Amplitudes, message->getAmplitudes(),
                message->getHeight()*message->getWidth()*sizeof(float) );
        memcpy( m_Points, message->getPoints(),
                message->getHeight()*message->getWidth()*sizeof(BaseLib::Math::Vec3f) );

        BaseLib::Math::Mat4d tofToWorld;

        tofToWorld = m_SceneGraph.getTransformation( "Kinect" , "World" );
        tofToWorld.toColumnMajor( m_Transformation );
        requestRedraw();
      }
      break;
    }

  case MessageTypes::SCENE_GRAPH_M:
    {
      if ( SceneGraphM* message = Message::castTo<SceneGraphM> ( newMessage ) )
      {
        m_SceneGraph = message->getSceneGraph();
        m_SceneGraphInitialized = true;
        requestRedraw();
      }
      break;
    }
  default:
    break;

  }
}

void THIS::paint ( float )
{
  if( Clock::getInstance()->getTimestamp() < m_LastDataTime + m_DisplayDuration
    && m_SceneGraphInitialized )
  {
    glMultMatrixd( m_Transformation );
    glPointSize( 2 );
    glBegin(GL_POINTS);
    for( int i=0; i<m_Width*m_Height ; i++)
    {
      if(m_Amplitudes[i]*m_AmplitudeScaling >m_AmplitudeThreshold )
      {
        if( m_ViewMode==0)
        {
          glColor3f( 2*m_Depths[i] * m_ColorFactor,
                     2*(1-m_Depths[i] * m_ColorFactor),
                     0 );
        }
        else if( m_ViewMode == 1)
        {
          float c = m_Amplitudes[i] * m_AmplitudeScaling;
          glColor3f( c, c, c );
        }
        else if( m_ViewMode == 2)
        {
          float brightness = m_Amplitudes[i] * m_AmplitudeScaling;
          if(brightness < 0 )
          {
            brightness = 0;
          }
          else if(brightness >1 )
          {
            brightness = 1;
          }
          glColor3f( 2*m_Depths[i] * m_ColorFactor * brightness,
                     2*(1 - m_Depths[i] * m_ColorFactor )*brightness,
                     0 );
        }
        glVertex3f( m_Points[i].x, m_Points[i].y, m_Points[i].z );
      }
    }
    glEnd();
  }
}

void THIS::initBuffers ( int height, int width )
{
  if( height == 0 || width == 0){
    m_Height = m_Width = 0;
    m_Depths = 0;
    m_Points = 0;
    m_Amplitudes = 0;
  }
  else if( height != m_Height || width != m_Width) {
    if( m_Depths && m_Amplitudes && m_Points){
      delete m_Depths;
      delete m_Amplitudes;
      delete m_Points;
    }
    m_Height = height;
    m_Width = width;
    m_Depths = new float[ m_Height * m_Width ];
    m_Amplitudes = new float[ m_Height * m_Width ];
    m_Points = new BaseLib::Math::Vec3f[ m_Height * m_Width ];

  }
}

#undef THIS
