/*******************************************************************************
 *  PersonPainter.cpp
 *
 *  (C) 2007 AG Aktives Sehen <agas@uni-koblenz.de>
 *           Universitaet Koblenz-Landau
 *
 *  Additional information:
 *  $Id: $
 *******************************************************************************/

#include "PersonPainter.h"

#include "Messages/TrackedPeopleM.h"
#include "Messages/FollowingStateM.h"


#include <QtOpenGL>
#include <GL/glut.h>

#include "Architecture/Config/Config.h"

#define THIS PersonPainter

THIS::THIS() : PainterPlugin( )
{
  setName ( "Person Tracking & Following" );
	m_PersonModel = SceneGraph ( "config/Person.xml" );
}


THIS::~THIS()
{
}


void THIS::processMessage ( Message* newMessage )
{
  PainterPlugin::processMessage ( newMessage );
  switch ( newMessage->getType() )
  {
    case MessageTypes::FOLLOWING_STATE_M:
    {
      if ( FollowingStateM* message = Message::castTo<FollowingStateM> ( newMessage ) )
      {
        m_Operator = message->getOperator(); 
        requestRedraw();
      }
      break;
    }

    case MessageTypes::TRACKED_PEOPLE_M:
    {
      if ( TrackedPeopleM* message = Message::castTo<TrackedPeopleM> ( newMessage ) )
      {
        m_TrackedPeople = message->getTrackedPeople(); 
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
  if ( m_Operator.getRadius() != 0.0 )
  {
    glPushMatrix();
		glTranslatef( 0,0,next2DLayer );

    glColor4f ( 1.0, 0.0, 0.0, 0.5 );
    glPolygonMode ( GL_FRONT_AND_BACK, GL_FILL );
    glEnable ( GL_BLEND );
    glBlendFunc ( GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA );
    glColor4f ( 1.0, 0.0, 0.0, 0.5 );

    glPushMatrix();
    glTranslatef ( m_Operator.getCenter().x(), m_Operator.getCenter().y(), 0.0 );
    glBegin ( GL_POLYGON );
    for ( float alpha = 0; alpha < 2 * M_PI; alpha += M_PI / 32 )
    {
      glVertex3f ( m_Operator.getRadius() * sin ( alpha ), m_Operator.getRadius() * cos ( alpha ), 0.0 );
    }
    glEnd();
    glLineWidth ( 3.0 );
    glBegin ( GL_LINES );
    glVertex3f ( 0.0, 0.0, 0.0 );
    glVertex3f ( 0.0, 0.0, 500.0 );
    glEnd();
    glLineWidth ( 1.0 );
    glPopMatrix();

    glColor4f ( 1.0, 1.0, 0.0, 0.5 );
    glBegin ( GL_LINES );
    glVertex3f ( 0.0, 0.0, 0.0 );
    glVertex3f ( m_Operator.getCenter().x(), m_Operator.getCenter().y(), 0.0 );
    glEnd();

    glDisable ( GL_BLEND );
    glPopMatrix();
  }


  for ( unsigned i=0; i<m_TrackedPeople.size(); i++ )
  {
    Point2D center = m_TrackedPeople[i].getCenter();
		
		BaseLib::Math::Vec3d position;
		position.x = center.x();
		position.y = center.y();
		m_PersonModel.setTranslationMatrix("Person.position",position);
		m_PersonModel.paintGl();
		
		
    glPolygonMode ( GL_FRONT_AND_BACK, GL_FILL );
		
		if ( m_TrackedPeople[i].isValid() )
		{
			glColor4f ( 1.0, 1.0, 0.0, 0.5 );
		}
		else
		{
			glColor4f ( 0.5, 0.5, 0.5, 0.5 );
		}

    float radius = m_TrackedPeople[i].getRadius() / 2.0;

    glPushMatrix();

    glTranslatef( center.x(), center.y(), 0.0 );
      glutSolidCone( radius, 300, 36, 1 );
    glPopMatrix();
  }
}



#undef THIS
