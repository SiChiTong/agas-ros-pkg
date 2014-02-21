/*******************************************************************************
 *  SonarPainter.cpp
 *
 *  (C) 2007 AG Aktives Sehen <agas@uni-koblenz.de>
 *           Universitaet Koblenz-Landau
 *
 *  Additional information:
 *  $Id: $
 *******************************************************************************/

#include "SonarPainter.h"

#include "Messages/FastRobotPoseM.h"
#include "Devices/PioneerRobot/SonarSensorConfig.h"
#include "Messages/SonarDataM.h"

#include <QtOpenGL>
#include <GL/glut.h>


#define THIS SonarPainter

THIS::THIS() : PainterPlugin( )
{
  setName ( "Sonar Data" );
}


THIS::~THIS()
{
}


void THIS::processMessage ( Message* newMessage )
{
  PainterPlugin::processMessage ( newMessage );
  switch ( newMessage->getType() )
  {
    case MessageTypes::SONAR_DATA_M:
    {
      SonarDataM* message = Message::castTo<SonarDataM> ( newMessage );
      if ( message )
      {
        m_SonarData = message->getRangeVector();
        requestRedraw();
      }
      break;
    }

    case MessageTypes::FAST_ROBOT_POSE_M:
    {
      FastRobotPoseM* message = Message::castTo<FastRobotPoseM> ( newMessage );
      if ( message )
      {
        m_RobotPose = message->getRobotPose();
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

  glTranslatef ( m_RobotPose.x(), m_RobotPose.y(), next2DLayer );
  glRotatef ( m_RobotPose.theta() * 180.0 / M_PI, 0.0, 0.0, 1.0 );
  glPolygonMode ( GL_FRONT_AND_BACK, GL_FILL );

  glColor4f ( 0.0, 1.0, 0.0, 0.7 );

  glEnable ( GL_BLEND );
  glBlendFunc ( GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA );

  for ( unsigned int i = 0; i < m_SonarData.size(); i++ )
  {
    int range = m_SonarData[i];
    if ( range >= SonarSensorConfig::VALID_MIN_RANGE && range <= SonarSensorConfig::VALID_MAX_RANGE )
    {
      float sensorAngle = 6.0;
      float sensorFatFactor = 100;
      glPushMatrix();

      glTranslatef ( SonarSensorConfig::SONAR_SENSOR_POSITIONS_X[i], SonarSensorConfig::SONAR_SENSOR_POSITIONS_Y[i], 10.0 );
      //SonarSensorConfig::SONAR_SENSOR_POSITIONS_Z[i] );
      glRotatef ( SonarSensorConfig::SONAR_SENSOR_ORIENTATIONS_DEG[i], 0.0, 0.0, 1.0 );

      float Rad = 1.0f / 180 * 3.141592654;

      glBegin ( GL_POLYGON );
      glVertex3f ( cos ( Rad * sensorAngle ) * ( range + sensorFatFactor ), sin ( Rad * sensorAngle ) * range, 0.0 );
      glVertex3f ( cos ( Rad * sensorAngle ) * ( range + sensorFatFactor ), ( -1 ) * sin ( Rad * sensorAngle ) * range, 0.0 );
      glVertex3f ( cos ( Rad * sensorAngle ) * ( range ), ( -1 ) * sin ( Rad * sensorAngle ) * range, 0.0 );
      glVertex3f ( cos ( Rad * sensorAngle ) * ( range ), sin ( Rad * sensorAngle ) * range, 0.0 );
      glEnd();

      glPopMatrix();
    }
  }
  glDisable ( GL_BLEND );
}

#undef THIS
