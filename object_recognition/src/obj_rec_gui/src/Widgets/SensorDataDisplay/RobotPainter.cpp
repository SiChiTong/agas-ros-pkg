/*******************************************************************************
 *  RobotPainter.cpp
 *
 *  (C) 2007 AG Aktives Sehen <agas@uni-koblenz.de>
 *           Universitaet Koblenz-Landau
 *
 *  Additional information:
 *  $Id: $
 *******************************************************************************/

#include "RobotPainter.h"

#include "Messages/FastRobotPoseM.h"
#include "Messages/IMUStateM.h"

#include <QtOpenGL>
#include <GL/glut.h>


#define THIS RobotPainter

THIS::THIS() : PainterPlugin( )
{
  setName ( "Main Robot" );
  m_RollAngle = 0;
  m_PitchAngle = 0;
}


THIS::~THIS()
{
}


//void THIS::processMessage ( Message* newMessage )
//{
//  PainterPlugin::processMessage ( newMessage );
//  switch ( newMessage->getType() )
//  {
//    /*case MessageTypes::ARTIFICIAL_HORIZON_DATA_M:
//    {
//      ArtificialHorizonDataM* message = Message::castTo<ArtificialHorizonDataM> ( newMessage );
//      if ( message )
//      {
//        m_RollAngle = message->getRollAngle();
//        m_PitchAngle = message->getPitchAngle();
//        ostringstream stream;
//      }
//      break;
//    }*/

//    case MessageTypes::IMU_STATE_M:
//      {
//        IMUStateM* message = Message::castTo<IMUStateM> ( newMessage );
//        if ( message )
//        {
//          m_RollAngle = message->getRollAngle();
//          m_PitchAngle = message->getPitchAngle();
//          ostringstream stream;
//        }
//        break;
//    }

//    case MessageTypes::FAST_ROBOT_POSE_M:
//    {
//      FastRobotPoseM* message = Message::castTo<FastRobotPoseM> ( newMessage );
//      if ( message )
//      {
//        m_RobotPose = message->getRobotPose();
//        requestRedraw();
//      }
//      break;
//    }

//    default:
//      break;

//  }
//}

void THIS::paint ( float next2DLayer )
{
  glTranslatef ( m_RobotPose.x(), m_RobotPose.y(), next2DLayer );
  glRotatef ( m_RobotPose.theta() * 180.0 / M_PI, 0.0, 0.0, 1.0 );

  glRotatef ( -m_RollAngle, 1.0, 0.0, 0.0 );
  glRotatef ( -m_PitchAngle, 0.0, 1.0, 0.0 );
  //HomMatrix3D robotToWorld = m_RobotPose.localToGlobal();

  glColor4f ( 0.0, 0.0, 0.0, 0.35 );
  glPolygonMode ( GL_FRONT_AND_BACK, GL_LINE );
  glEnable ( GL_BLEND );
  glBlendFunc ( GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA );

  // draw arrow
  glBegin ( GL_POLYGON );
  glVertex3f ( 160, 0, 0 );
  glVertex3f ( -106, 100, 0 );
  glVertex3f ( -50, 0, 0 );
  glVertex3f ( -106, -100, 0 );
  glEnd();

  glRotatef ( 180, 0.0, 0.0, 1.0 );

  float extrudeHeight = 0.0;

  // draw robbie
  glBegin ( GL_POLYGON );
  glVertex3f ( 232, 115, extrudeHeight );
  glVertex3f ( 240, 136, extrudeHeight );
  glVertex3f ( 240, 225, extrudeHeight );
  glVertex3f ( 40, 225, extrudeHeight );
  glVertex3f ( 40, 172, extrudeHeight );
  glVertex3f ( -10, 172, extrudeHeight );

  glVertex3f ( -10, 225, extrudeHeight );
  glVertex3f ( -215, 225, extrudeHeight );
  glVertex3f ( -215, 135, extrudeHeight );
  glVertex3f ( -285, 135, extrudeHeight );
  glVertex3f ( -285, 12, extrudeHeight );
  glVertex3f ( -285, -12 , extrudeHeight );
  glVertex3f ( -285, -135 , extrudeHeight );
  glVertex3f ( -215, -135 , extrudeHeight );
  glVertex3f ( -215, -225 , extrudeHeight );
  glVertex3f ( -10, -225 , extrudeHeight );

  glVertex3f ( -10, -172 , extrudeHeight );
  glVertex3f ( 40, -172 , extrudeHeight );
  glVertex3f ( 40, -225 , extrudeHeight );
  glVertex3f ( 240, -225 , extrudeHeight );
  glVertex3f ( 240, -136, extrudeHeight );
  glVertex3f ( 232, -115 , extrudeHeight );
  glVertex3f ( 282, -115 , extrudeHeight );
  glVertex3f ( 282, 115 , extrudeHeight );
  glEnd();

  extrudeHeight = 150.0;

  // draw robbie
  glColor4f ( 1.0, 1.0, 1.0, 0.35 );
  glPolygonMode ( GL_FRONT, GL_FILL );
  glPolygonMode ( GL_BACK, GL_NONE );
  glBegin ( GL_POLYGON );
  glVertex3f ( 100, 0, extrudeHeight );
  glVertex3f ( 232, 115, extrudeHeight );
  glVertex3f ( 240, 136, extrudeHeight );
  glVertex3f ( 240, 225, extrudeHeight );
  glVertex3f ( 40, 225, extrudeHeight );
  glVertex3f ( 40, 172, extrudeHeight );
  glVertex3f ( -10, 172, extrudeHeight );
  glVertex3f ( -10, -172 , extrudeHeight );
  glVertex3f ( 40, -172 , extrudeHeight );
  glVertex3f ( 40, -225 , extrudeHeight );
  glVertex3f ( 240, -225 , extrudeHeight );
  glVertex3f ( 240, -136, extrudeHeight );
  glVertex3f ( 232, -115 , extrudeHeight );
  glVertex3f ( 282, -115 , extrudeHeight );
  glVertex3f ( 282, 115 , extrudeHeight );
  glVertex3f ( 232, 115, extrudeHeight );
  glEnd();
  glBegin ( GL_POLYGON );
  glVertex3f ( -100, 0, extrudeHeight );
  glVertex3f ( -10, 225, extrudeHeight );
  glVertex3f ( -215, 225, extrudeHeight );
  glVertex3f ( -215, 135, extrudeHeight );
  glVertex3f ( -285, 135, extrudeHeight );
  glVertex3f ( -285, 12, extrudeHeight );
  glVertex3f ( -285, -12 , extrudeHeight );
  glVertex3f ( -285, -135 , extrudeHeight );
  glVertex3f ( -215, -135 , extrudeHeight );
  glVertex3f ( -215, -225 , extrudeHeight );
  glVertex3f ( -10, -225 , extrudeHeight );
  glVertex3f ( -10, 225, extrudeHeight );
  glEnd();
  glColor4f ( 0.7, 0.7, 0.7, 0.2 );
  glPolygonMode ( GL_BACK, GL_FILL );
  glPolygonMode ( GL_FRONT, GL_NONE );
  glBegin ( GL_TRIANGLE_STRIP );
  glVertex3f ( 232, 115, 0.0 );
  glVertex3f ( 232, 115, extrudeHeight );
  glVertex3f ( 223, 130, 0.0 );
  glVertex3f ( 223, 130, extrudeHeight );
  glVertex3f ( 223, 138, 0.0 );
  glVertex3f ( 223, 138, extrudeHeight );
  glVertex3f ( 240, 136, 0.0 );
  glVertex3f ( 240, 136, extrudeHeight );
  glVertex3f ( 240, 225, 0.0 );
  glVertex3f ( 240, 225, extrudeHeight );
  glVertex3f ( 40, 225, 0.0 );
  glVertex3f ( 40, 225, extrudeHeight );
  glVertex3f ( 40, 172, 0.0 );
  glVertex3f ( 40, 172, extrudeHeight );
  glVertex3f ( -10, 172, 0.0 );
  glVertex3f ( -10, 172, extrudeHeight );
  glVertex3f ( -10, 225, 0.0 );
  glVertex3f ( -10, 225, extrudeHeight );
  glVertex3f ( -215, 225, 0.0 );
  glVertex3f ( -215, 225, extrudeHeight );
  glVertex3f ( -215, 135, 0.0 );
  glVertex3f ( -215, 135, extrudeHeight );
  glVertex3f ( -285, 135, 0.0 );
  glVertex3f ( -285, 135, extrudeHeight );
  glVertex3f ( -285, 12, 0.0 );
  glVertex3f ( -285, 12, extrudeHeight );
  glVertex3f ( -285, -12 , 0.0 );
  glVertex3f ( -285, -12 , extrudeHeight );
  glVertex3f ( -285, -135 , 0.0 );
  glVertex3f ( -285, -135 , extrudeHeight );
  glVertex3f ( -215, -135 , 0.0 );
  glVertex3f ( -215, -135 , extrudeHeight );
  glVertex3f ( -215, -225 , 0.0 );
  glVertex3f ( -215, -225 , extrudeHeight );
  glVertex3f ( -10, -225 , 0.0 );
  glVertex3f ( -10, -225 , extrudeHeight );
  glVertex3f ( -10, -172 , 0.0 );
  glVertex3f ( -10, -172 , extrudeHeight );
  glVertex3f ( 40, -172 , 0.0 );
  glVertex3f ( 40, -172 , extrudeHeight );
  glVertex3f ( 40, -225 , 0.0 );
  glVertex3f ( 40, -225 , extrudeHeight );
  glVertex3f ( 240, -225 , 0.0 );
  glVertex3f ( 240, -225 , extrudeHeight );
  glVertex3f ( 240, -136, 0.0 );
  glVertex3f ( 240, -136, extrudeHeight );
  glVertex3f ( 223, -138, 0.0 );
  glVertex3f ( 223, -138, extrudeHeight );
  glVertex3f ( 223, -130 , 0.0 );
  glVertex3f ( 223, -130 , extrudeHeight );
  glVertex3f ( 232, -115 , 0.0 );
  glVertex3f ( 232, -115 , extrudeHeight );
  glVertex3f ( 282, -115 , 0.0 );
  glVertex3f ( 282, -115 , extrudeHeight );
  glVertex3f ( 282, 115 , 0.0 );
  glVertex3f ( 282, 115 , extrudeHeight );
  glEnd();

  glDisable ( GL_BLEND );

}

#undef THIS
