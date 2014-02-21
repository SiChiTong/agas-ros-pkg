/*******************************************************************************
 *  SceneGraphPainter.cpp
 *
 *  (C) 2007 AG Aktives Sehen <agas@uni-koblenz.de>
 *           Universitaet Koblenz-Landau
 *
 *  Additional information:
 *  $Id: $
 *******************************************************************************/

#include "SceneGraphPainter.h"

#include "Messages/SceneGraphM.h"

#include <QtOpenGL>
#include <GL/glut.h>

#include <math.h>

#define THIS SceneGraphPainter

THIS::THIS() : PainterPlugin( )
{
  setName ( "SceneGraph" );
	m_AutoUpdate = true;
}


THIS::~THIS()
{
}


void THIS::processMessage ( Message* newMessage )
{
  switch ( newMessage->getType() )
  {
    case MessageTypes::SCENE_GRAPH_M:
    {
			if ( m_AutoUpdate )
			{
				if ( SceneGraphM* message = Message::castTo<SceneGraphM> ( newMessage ) )
				{
					m_SceneGraph = message->getSceneGraph();
					requestRedraw();
				}
      }
      break;
    }

    default:
      break;
  }
}

void THIS::setSceneGraph( SceneGraph &sceneGraph ) 
{ 
	m_SceneGraph = sceneGraph; 
	requestRedraw();
}

void THIS::paint ( float next2DLayer )
{
  //TRACE_INFO( m_SceneGraph.toString() );
  m_SceneGraph.paintGl();
}

#undef THIS
