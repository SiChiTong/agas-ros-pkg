/*******************************************************************************
 *  ParticlePainter.cpp
 *
 *  (C) 2007 AG Aktives Sehen <agas@uni-koblenz.de>
 *           Universitaet Koblenz-Landau
 *
 *  Additional information:
 *  $Id: $
 *******************************************************************************/

#include "ParticlePainter.h"


#include <QtOpenGL>
#include <GL/glut.h>
#include <cmath>


#define THIS ParticlePainter

THIS::THIS() : PainterPlugin( )
{
  setName ( "Particle Arrows" );
}


THIS::~THIS()
{
}


void THIS::processMessage ( Message* newMessage )
{
  PainterPlugin::processMessage ( newMessage );
  switch ( newMessage->getType() )
  {
    case MessageTypes::PARTICLE_DATA_M:

    {
      ParticleDataM* message = Message::castTo<ParticleDataM> ( newMessage );
      if ( message )
      {
        m_ParticleData = * ( message->getParticles() );
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
  glPolygonMode ( GL_FRONT_AND_BACK, GL_LINE );
  glLineWidth ( 1.0 );
  float t;
  glTranslatef ( 0.0, 0.0, next2DLayer );

  for ( unsigned int i = 0; i < m_ParticleData.size(); i++ )
  {
    glPushMatrix();

    glTranslatef ( m_ParticleData[i].x, m_ParticleData[i].y, 0.0 );

    glRotatef ( m_ParticleData[i].theta * 180.0 / M_PI, 0.0, 0.0, 1.0 );
    t = float ( i ) / float ( m_ParticleData.size() );
    glScalef ( 1.5 - t, 1.5 - t, 1.5 - t );
    glColor3f ( 1.0 - t, 1.0 - t, 1.0 - t ); // color coding the index of the particle

    // arrow
    glBegin ( GL_POLYGON );
    glVertex3f ( -15.0,  0.0, 1.0 - t );
    glVertex3f ( -25.0,  5.0, 1.0 - t );
    glVertex3f ( 25.0,  0.0, 1.0 - t );
    glVertex3f ( -25.0, -5.0, 1.0 - t );
    glEnd();

    glPopMatrix();
  }
}

#undef THIS
