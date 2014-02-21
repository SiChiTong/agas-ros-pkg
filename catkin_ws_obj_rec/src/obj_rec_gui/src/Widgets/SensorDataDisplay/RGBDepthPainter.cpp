/*******************************************************************************
 *  RGBDepthPainter.cpp
 *
 *  (C) 2007 AG Aktives Sehen <agas@uni-koblenz.de>
 *           Universitaet Koblenz-Landau
 *
 *  Additional information:
 *  $Id: $
 *******************************************************************************/

#include "RGBDepthPainter.h"

#include "Workers/Math/Math.h"
#include "Messages/SceneGraphM.h"

#include "Architecture/Config/Config.h"

#include <QtOpenGL>
#include <GL/glut.h>

#define THIS RGBDepthPainter

THIS::THIS() : PainterPlugin( )
{
  setName ( "RGB Depth Data" );
}


THIS::~THIS()
{
}


void THIS::processMessage ( Message* newMessage )
{
  PainterPlugin::processMessage ( newMessage );
  switch ( newMessage->getType() )
  {

    case MessageTypes::RGB_DEPTH_M:
    {
      RGBDepthM* message = Message::castTo<RGBDepthM> ( newMessage );
      if ( message )
      {
        m_Rows = message->getRows();
        m_Columns = message->getColumns();

        if(!Config::getBool("Kinect.bCreate3DMap"))
        {
          // Don't store old depth data
          m_Points.clear();
          m_ValidRobotToWorld.clear();
          m_RgbImage.clear();
        }

        m_Points.push_back(message->getPoints());

        if(message->getRgbImage())
        {
          m_RgbImageAvailable = true;
          m_RgbImage.push_back(*message->getRgbImage());
        }
        else
        {
          m_RgbImageAvailable = false;
        }

        m_ValidRobotToWorld.push_back(message->getSceneGraph().getTransformation( "Robot", "World" ));

        requestRedraw();
      }
      break;
    }

  default:
    break;

  }
}

void THIS::paint ( float next2DLayer )
{




  for(unsigned int i=0;i<m_Points.size();++i)
  {
    glPushMatrix();

    /* Transform points from robot to world coordinates */
    GLdouble glMatrix[16];
    m_ValidRobotToWorld.at(i).toColumnMajor ( glMatrix );
    glMultMatrixd ( glMatrix );

    /* Draw points with corresponding color */

    glPointSize ( 1.0 );
    glBegin( GL_POINTS );

    for(unsigned int y=0; y<m_Rows;++y)
    {
      for(unsigned int x=0; x<m_Columns;++x)
      {
        /* Color values have range [0..1] */
        if(m_RgbImageAvailable)
        {
          glColor3f((m_RgbImage.at(i).sample(x,y,0)/255.0),(m_RgbImage.at(i).sample(x,y,1)/255.0),(m_RgbImage.at(i).sample(x,y,2)/255.0));
        }
        else
        {
          glColor3f(1,1,1);
        }
        glVertex3d(m_Points.at(i).at(x+y*m_Columns).x, m_Points.at(i).at(x+y*m_Columns).y, m_Points.at(i).at(x+y*m_Columns).z);
      }
    }

    glEnd();

    glPopMatrix();

  }
}

#undef THIS
