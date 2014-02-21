/*******************************************************************************
 *  ParticleGridPainter.cpp
 *
 *  (C) 2007 AG Aktives Sehen <agas@uni-koblenz.de>
 *           Universitaet Koblenz-Landau
 *
 *  Additional information:
 *  $Id: $
 *******************************************************************************/

#include "ParticleGridPainter.h"

#include "Architecture/Config/Config.h"
#include "Messages/FastRobotPoseM.h"

#include <QtOpenGL>
#include <GL/glut.h>


#define THIS ParticleGridPainter

THIS::THIS() : PainterPlugin( )
{
  setName ( "Particle Density" );

  m_ParticleBinCount =  Config::getInt( "Gui.SensorData3d.iParticleGridBins" );
  m_ParticleBinDisplaySize =  Config::getFloat( "Gui.SensorData3d.fParticleGridBinDisplaySize" );
  m_ParticlePeekHeight = Config::getFloat( "Gui.SensorData3d.fParticleGridBinDisplayPeekHeight" );
  m_WeightGrid = 0;
}


THIS::~THIS()
{
  if ( m_WeightGrid )
  {
    delete m_WeightGrid;
  }
}


void THIS::processMessage ( Message* newMessage )
{
  PainterPlugin::processMessage ( newMessage );
  switch ( newMessage->getType() )
  {
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

    case MessageTypes::PARTICLE_DATA_M:

    {
      ParticleDataM* message = Message::castTo<ParticleDataM> ( newMessage );
      if ( message )
      {
        m_ParticleData = * ( message->getParticles() );
        generateMesh();
        requestRedraw();
      }
      break;
    }

    default:
      break;

  }
}

void THIS::generateMesh()
{
  int meshSize = m_ParticleBinCount;
  int meshSizeHalf = meshSize / 2;

  if ( !m_WeightGrid )
  {
    m_WeightGrid = new float[ meshSize * meshSize ];
  }

  m_HeightPeek = 0.0;

  float scaleFactor = m_ParticleBinDisplaySize;

  float CenterX = m_RobotPose.x(); //m_LookAtX;
  float CenterY = m_RobotPose.y(); //m_LookAtY;

  // initialize/erase mesh
  memset ( m_WeightGrid, 0, sizeof ( m_WeightGrid ) * meshSize * meshSize );

  // set the mesh data and remember max/min for fastened up drawing process
  m_MinX = meshSize / 2;
  m_MinY = meshSize / 2;
  m_MaxX = m_MinX;
  m_MaxY = m_MinY;
  for ( unsigned int i = 0; i < m_ParticleData.size(); ++i )
  {
    if ( m_ParticleData[ i ].poseKind == ParticleDataM::Particle )
    {
      int meshX = ( ( int ( float ( m_ParticleData[i].x ) - CenterX  + 0.5 ) ) + meshSizeHalf * scaleFactor );
      int meshY = ( ( int ( float ( m_ParticleData[i].y ) - CenterY  + 0.5 ) ) + meshSizeHalf * scaleFactor );
      if ( meshX >= 0 && meshX < meshSize * scaleFactor && meshY >= 0 && meshY < meshSize * scaleFactor )
      {
        unsigned meshPositionX = unsigned ( meshX / scaleFactor );
        unsigned meshPositionY = unsigned ( meshY / scaleFactor );

        // update the bounding box
        if ( m_MinX > meshPositionX ) m_MinX = meshPositionX;
        if ( m_MaxX < meshPositionX ) m_MaxX = meshPositionX;
        if ( m_MinY > meshPositionY ) m_MinY = meshPositionY;
        if ( m_MaxY < meshPositionY ) m_MaxY = meshPositionY;
        * ( m_WeightGrid + meshPositionY * meshSize + meshPositionX ) += m_ParticleData[i].weight; // set the height;

        if ( ( * ( m_WeightGrid + meshPositionY * meshSize + meshPositionX ) ) > m_HeightPeek )
          m_HeightPeek = * ( m_WeightGrid + meshPositionY * meshSize + meshPositionX );
      }
    }
  }
}

void THIS::paint ( float next2DLayer )
{
  if ( (!m_WeightGrid) || ( m_ParticleData.size() == 0 ) )
  {
    return;
  }

  glPushMatrix();
  glTranslatef ( 0.0, 0.0, next2DLayer );

  //prepare mesh height

  int meshSize = m_ParticleBinCount;
  //200;
  int meshSizeHalf = meshSize / 2;

  //float mesh[meshSize][meshSize];
  float scaleFactor = m_ParticleBinDisplaySize;
  float alpha = 0.8;
  float weightScaleFactor = 0.0; // m_ParticleBinDisplayHeightFactor;

  float CenterX = m_RobotPose.x(); //m_LookAtX;
  float CenterY = m_RobotPose.y(); //m_LookAtY;

  glPolygonMode ( GL_FRONT, GL_FILL );
  glPolygonMode ( GL_BACK, GL_LINE );


  glEnable ( GL_BLEND );
  glBlendFunc ( GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA );

  glTranslatef ( CenterX, CenterY, 0 );
  //no rotation required!
  //glRotatef(m_RobotPose.theta() * 180.0 / M_PI, 0.0, 0.0, 1.0);
  glTranslatef ( -meshSizeHalf * scaleFactor, -meshSizeHalf * scaleFactor, 0 );
  glColor3f ( 1.0, 0, 0 );


  // calculate the height scale factor out of the peek
  weightScaleFactor = m_ParticlePeekHeight / m_HeightPeek;

  // draw the mesh
  for ( unsigned x = m_MinX; x < m_MaxX - 1; ++x )
  {
    glBegin ( GL_TRIANGLE_STRIP );
    for ( unsigned y = m_MinY; y < m_MaxY; ++y )
    {
      // always draw to points for the triangle stripe

      float curMesh = ( * ( m_WeightGrid + y * meshSize + x ) ) * weightScaleFactor; // get current mesh data
      float h = curMesh;
      if ( curMesh > 0 )
      {
        float green = curMesh / m_ParticlePeekHeight * 1.5;
        if ( green > 1.0 )
          green = 1.0;
        glColor4f ( 1.0, green, 0, alpha );
      }
      else
      {
        glColor4f ( 1.0, 0, 0, 0 );
      }
      glVertex3f ( x * scaleFactor, y * scaleFactor, h );

      curMesh = ( * ( m_WeightGrid + y * meshSize + x + 1 ) ) * weightScaleFactor; // get current mesh data
      h = curMesh;
      if ( curMesh > 0 )
      {
        float green = curMesh / m_ParticlePeekHeight * 1.5;
        if ( green > 1.0 )
          green = 1.0;
        glColor4f ( 1.0, green, 0, alpha );
      }
      else
      {
        glColor4f ( 1.0, 0, 0, 0 );
      }
      glVertex3f ( ( x + 1 ) * scaleFactor, y * scaleFactor, h );
    }
    glEnd(); // triangle strip
  }

  glDisable ( GL_BLEND );
  glPopMatrix();
}

#undef THIS
