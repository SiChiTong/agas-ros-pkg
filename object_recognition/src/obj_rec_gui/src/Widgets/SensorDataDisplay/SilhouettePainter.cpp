/*******************************************************************************
 *  SilhouettePainter.cpp
 *
 *  (C) 2007 AG Aktives Sehen <agas@uni-koblenz.de>
 *           Universitaet Koblenz-Landau
 *
 *  Additional information:
 *  $Id: $
 *******************************************************************************/

#include "SilhouettePainter.h"


#include "Messages/SilhouettesM.h"



#include <QtOpenGL>
#include <GL/glut.h>

#include "Architecture/Config/Config.h"

#define THIS SilhouettePainter

THIS::THIS() : PainterPlugin( )
{
    setName ( "Silhouette Painter & Following" );
    m_PersonModel = SceneGraph ( "config/PersonTorso.xml" );
    m_HeadWithHandModel= SceneGraph ( "config/PersonHeadWithHand.xml" );
    m_HandModel= SceneGraph ( "config/PersonHand.xml" );
}


THIS::~THIS()
{
}


void THIS::processMessage ( Message* newMessage )
{
    PainterPlugin::processMessage ( newMessage );
    switch ( newMessage->getType() )
    {

    case MessageTypes::SILHOUETTES_M:
    {
        if ( SilhouettesM* message = Message::castTo<SilhouettesM> ( newMessage ) )
        {
            // std::cout<<"get silhouettesM in SilhouettePainter\n";
            m_SilhouettesPositions3d = message->get3DPositions( );
            m_HeadsWithHand = message->getheadsWithHands();
            m_Hands = message->gethands();
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

    for ( unsigned i=0; i<m_SilhouettesPositions3d.size(); i++ )
    {
        BaseLib::Math::Vec3d position =  m_SilhouettesPositions3d[i];

        m_PersonModel.setTranslationMatrix("Person.position",position);
        m_PersonModel.paintGl();

        glPolygonMode ( GL_FRONT_AND_BACK, GL_FILL );
        glColor4f ( 1.0, 1.0, 0.0, 0.5 );
        float radius = 2.0;
        glPushMatrix();
        glTranslatef( position.x, position.y, 0.0 );
        glutSolidCone( radius, 300, 36, 1 );
        glPopMatrix();
    }

    for ( unsigned i=0; i<m_Hands.size(); i++ )
    {
        BaseLib::Math::Vec3d position =  m_Hands[i];

        m_HandModel.setTranslationMatrix("Person.position",position);
        m_HandModel.paintGl();

        glPolygonMode ( GL_FRONT_AND_BACK, GL_FILL );
        glColor4f ( 1.0, 1.0, 0.0, 0.5 );
        float radius = 2.0;
        glPushMatrix();
        glTranslatef( position.x, position.y, 0.0 );
        glutSolidCone( radius, 300, 36, 1 );
        glPopMatrix();
    }

    for ( unsigned i=0; i<m_HeadsWithHand.size(); i++ )
    {
        BaseLib::Math::Vec3d position =  m_HeadsWithHand[i];

        m_HeadWithHandModel.setTranslationMatrix("Person.position",position);
        m_HeadWithHandModel.paintGl();

        glPolygonMode ( GL_FRONT_AND_BACK, GL_FILL );
        glColor4f ( 1.0, 1.0, 0.0, 0.5 );
        float radius = 2.0;
        glPushMatrix();
        glTranslatef( position.x, position.y, 0.0 );
        glutSolidCone( radius, 300, 36, 1 );
        glPopMatrix();
    }



}



#undef THIS
