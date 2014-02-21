/*******************************************************************************
 *  MapPainter.cpp
 *
 *  (C) 2007 AG Aktives Sehen <agas@uni-koblenz.de>
 *           Universitaet Koblenz-Landau
 *
 *  Additional information:
 *  $Id: $
 *******************************************************************************/

#include "MapPainter.h"

#include "Messages/MapDataM.h"

#include "GUI/RobbieWidget/RobbieGLWidget.h"

#include <QtOpenGL>
#include <GL/glut.h>

#include "Architecture/Config/Config.h"

#define THIS MapPainter

THIS::THIS() : PainterPlugin( )
{
  setName ( "SLAM Occupancy Map" );

  m_MapMmSize = Config::getInt( "Map.iSize" );
  m_MapPixelSize = m_MapMmSize / Config::getInt( "Map.iCellSize" ) + 1;

  m_TextureSize = RobbieGLWidget::calcTextureSize ( m_MapPixelSize, m_MapPixelSize );
  m_TextureId = -1;
  m_TextureData = new unsigned char[m_TextureSize*m_TextureSize];
  memset ( m_TextureData, 127, m_TextureSize*m_TextureSize );
  m_TextureLoaded = false;

}


THIS::~THIS()
{
  if ( m_TextureData )
  {
    TRACE_SYSTEMINFO( "Deleting map texture" )
    delete m_TextureData;
  }

  if ( m_TextureId > 0 )
  {
    TRACE_SYSTEMINFO( "Deleting OpenGL textures" )
    glDeleteTextures ( 1, &m_TextureId );
  }
}


void THIS::processMessage ( Message* newMessage )
{
  PainterPlugin::processMessage ( newMessage );
  switch ( newMessage->getType() )
  {
    case MessageTypes::MAP_DATA_M:
    {
      MapDataM* message = Message::castTo<MapDataM> ( newMessage );
      if ( message )
      {
        updateMap ( message->getMapPointer() );
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
  glEnable ( GL_BLEND );
  glBlendFunc ( GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA );
  glPolygonMode ( GL_FRONT_AND_BACK, GL_FILL );

  glColor4f ( 0.5, 0.7, 1.0, 0.6 );
//  glColor4f ( 1,1,1,1 );

  glEnable ( GL_TEXTURE_2D );

  if ( !m_TextureLoaded )
  {
    loadGlTexture();
  }
  else
  {
    glBindTexture ( GL_TEXTURE_2D, m_TextureId );
  }

  glTexParameterf ( GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST );
  glTexParameterf ( GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST );
  glTexEnvf ( GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE );
//  glTexEnvf ( GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE );

  //clamp texture so it doesn't repeat
  glTexParameterf( GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP );
  glTexParameterf( GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP );


  float halfSize = float ( m_TextureSize ) / float ( m_MapPixelSize ) * float ( m_MapMmSize ) / 2.0;
  float s=sqrt(2);

  glBegin ( GL_POLYGON );
  /*
  glTexCoord2f ( 0.0,  0.0 );
  glVertex3f ( halfSize,  halfSize, 0.0 );
  glTexCoord2f ( 0.0,  1.0 );
  glVertex3f ( -halfSize,  halfSize, 0.0 );
  glTexCoord2f ( 1.0,  1.0 );
  glVertex3f ( -halfSize, -halfSize, 0.0 );
  glTexCoord2f ( 1.0,  0.0 );
  glVertex3f ( halfSize, -halfSize, 0.0 );
  */

  //paint circle with map as texture
  for ( unsigned i=0; i<100; i++ )
  {
    float angle = float(i) / 100.0 * 2.0 * M_PI;
    float x = cos(angle)*s;
    float y = sin(angle)*s;
    glTexCoord2f ( 0.5-0.5*y, 0.5-0.5*x );
    glVertex3f ( halfSize*x,  halfSize*y, 0.0 );
  }


  glEnd();

  glDisable ( GL_TEXTURE_2D );
}


void THIS::updateMap ( unsigned char* mapData )
{
  if ( m_TextureData )
  {
    int textureCenter = m_TextureSize / 2;
    int startX = textureCenter - ( m_MapPixelSize / 2 );
    int startY = textureCenter - ( m_MapPixelSize / 2 );

    //pointer to top-left corner of the draw window inside the texture
    unsigned char* texturePos = m_TextureData + startX + ( startY * m_TextureSize );
    unsigned char* mapPos = mapData;

    //draw new image in the center
    for ( int y = 0; y < m_MapPixelSize ; y++ )
    {
      memcpy ( texturePos, mapPos, m_MapPixelSize );
      texturePos += m_TextureSize;
      mapPos += m_MapPixelSize;
    }

    m_TextureLoaded = false;

  }
}


void THIS::loadGlTexture()
{
  if ( m_TextureId < 1 )
  {
    glGenTextures ( 1, &m_TextureId );
    ostringstream stream;
    stream << "Using OpenGL texture # " << m_TextureId;
    TRACE_INFO( stream.str() );
  }

  if ( m_TextureData == 0 || m_TextureSize == 0 )
  {
    return;
  }


  glBindTexture ( GL_TEXTURE_2D, m_TextureId );
  glTexImage2D ( GL_TEXTURE_2D, 0, GL_LUMINANCE, m_TextureSize, m_TextureSize, 0, GL_LUMINANCE, GL_UNSIGNED_BYTE, m_TextureData );
  m_TextureLoaded = true;
}



#undef THIS
