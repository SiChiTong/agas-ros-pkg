/*******************************************************************************
 *  SkeletonPainter.cpp
 *
 *  (C) 2011 AG Aktives Sehen <agas@uni-koblenz.de>
 *           Universitaet Koblenz-Landau
 *
 *  Additional information:
 *  $Id: $
 *******************************************************************************/

#include "SkeletonPainter.h"

#include "Messages/TrackedPeopleM.h"
#include "Messages/FollowingStateM.h"
#include "Messages/RosSkeletonM.h"

#include <QtOpenGL>
#include <GL/glut.h>

#include "Architecture/Config/Config.h"

#define THIS SkeletonPainter

THIS::THIS() : PainterPlugin( )
{
  setName ( "Skeleton Tracking & Following" );
  m_SkeletonModel = SceneGraph ( "config/Skelett.xml" );
}


THIS::~THIS()
{
}


void THIS::processMessage ( Message* newMessage )
{
  PainterPlugin::processMessage ( newMessage );
  switch ( newMessage->getType() )
  {
    case MessageTypes::ROS_SKELETON_M:
      {
        if ( RosSkeletonM* message = Message::castTo<RosSkeletonM> ( newMessage ) )
        {
          m_TrackedSkeletons = message->getSceneGraph();
          /*for (unsigned int i=0; i<m_TrackedSkeletons.size();++i)
          {
              m_TrackedSkeletons[i].paintGl();
          }*/
          requestRedraw();
        }
        break;
      }
    default:
        break;

  } // switch
}

void THIS::paint ( float next2DLayer )
{
    for (unsigned int i=0; i<m_TrackedSkeletons.size();++i)
    {
        m_TrackedSkeletons[i].paintGl();
    }

}



#undef THIS
