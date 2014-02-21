/*******************************************************************************
 *  POIPainter.cpp
 *
 *  (C) 2007 AG Aktives Sehen <agas@uni-koblenz.de>
 *           Universitaet Koblenz-Landau
 *
 *  Additional information:
 *  $Id: $
 *******************************************************************************/

#include "POIPainter.h"

#include "Messages/PointsOfInterestM.h"

#include <QtOpenGL>
#include <GL/glut.h>


#define THIS POIPainter

THIS::THIS() : PainterPlugin( )
{
  setName ( "Points Of Interest" );
}


THIS::~THIS()
{
}


void THIS::processMessage ( Message* newMessage )
{
  PainterPlugin::processMessage ( newMessage );
  switch ( newMessage->getType() )
  {

    case MessageTypes::POINTS_OF_INTEREST_M:
    {
      PointsOfInterestM* message = Message::castTo<PointsOfInterestM> ( newMessage );
      if ( message )
      {
        m_Pois = message->getPointsOfInterest();
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
  glTranslatef ( 0.0, 0.0, next2DLayer );
  glPolygonMode( GL_FRONT_AND_BACK, GL_FILL );

  for ( list<PointOfInterest>::iterator it = m_Pois.begin(); it != m_Pois.end(); it++ )
  {
    glPushMatrix();

    glTranslatef ( it->x(), it->y(), 0.0 );

    //Draw name
    glColor3f ( 1, 1, 1 );
    QFont font;
    font.setFamily( "Monospace" );
    font.setPixelSize( 14 );

    m_ParentWidget->renderText ( 0, 0, 300, QString( it->getName().c_str() ), font );

    glLineWidth( 1.0 );
    glBegin( GL_LINES );
    glVertex3f( 0.0, 0.0, 0.0 );
    glVertex3f( 0.0, 0.0, 260.0 );
    glEnd();

    //draw cross
    float s=100.0;
    glRotatef( 45.0, 0, 0, 1 );
    glScalef( s, s, s );

    int color = it->getType() / 100 + 1;
    int r = color % 2;
    int g = ( color / 2 ) % 2;
    int b = ( color / 4 ) % 2;

    //paint white box
    glColor3f ( 1, 1, 1 );
    glPushMatrix();
    glScalef( 1.0, 0.25, .2 );
    glutSolidCube( 1.0 );
    glPopMatrix();
    glPushMatrix();
    glScalef( 0.25, 1.0, .2 );
    glutSolidCube( 1.0 );
    glPopMatrix();

    //paint colored box
    glColor3f ( r, g, b );
    glPushMatrix();
    glScalef( 0.9, 0.2, .25 );
    glutSolidCube( 1.0 );
    glPopMatrix();
    glPushMatrix();
    glScalef( 0.2, 0.9, .25 );
    glutSolidCube( 1.0 );
    glPopMatrix();

    glPopMatrix();
  }
}

#undef THIS
