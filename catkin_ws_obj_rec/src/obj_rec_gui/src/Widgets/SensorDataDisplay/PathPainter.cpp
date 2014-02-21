/*******************************************************************************
 *  PathPainter.cpp
 *
 *  (C) 2007 AG Aktives Sehen <agas@uni-koblenz.de>
 *           Universitaet Koblenz-Landau
 *
 *  Additional information:
 *  $Id: $
 *******************************************************************************/

#include "PathPainter.h"

#include "Messages/PathDataM.h"

#include <QtOpenGL>
#include <GL/glut.h>


#define THIS PathPainter

THIS::THIS() : PainterPlugin( )
{
  setName ( "Path (Navigation)" );
}


THIS::~THIS()
{
}


void THIS::processMessage ( Message* newMessage )
{
  PainterPlugin::processMessage ( newMessage );
  switch ( newMessage->getType() )
  {
    case MessageTypes::PATH_DATA_M:
    {
      PathDataM* message = Message::castTo<PathDataM> ( newMessage );
      if ( message )
      {
        m_PathXCoordinates = message->getXCoords();
        m_PathYCoordinates = message->getYCoords();
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
  if ( m_PathYCoordinates.size() == 0 )
  {
    return;
  }

  glTranslatef ( 0.0, 0.0, next2DLayer );

  glBegin ( GL_LINE_STRIP );
  glColor3f ( 1.0, 0.0, 1.0 );
  glLineWidth ( 3 );
  for ( unsigned int i = 0; i < m_PathXCoordinates.size(); i++ )
  {
    glVertex3f ( m_PathXCoordinates[i], m_PathYCoordinates[i], 0.0 );
  }
  glEnd();

  for ( unsigned int i = 0; i < m_PathXCoordinates.size(); i++ )
  {
    glBegin ( GL_POLYGON );
    for ( float alpha = 0; alpha < 2 * M_PI; alpha += M_PI / 8 )
    {
      glVertex3f ( m_PathXCoordinates[i] + 50.0 * sin ( alpha ), m_PathYCoordinates[i] + 50.0 * cos ( alpha ), 0.0 );
    }
    glEnd();
  }
}

#undef THIS
