/*******************************************************************************
 *  PersonParticlePainter.cpp
 *
 *  (C) 2011 AG Aktives Sehen <agas@uni-koblenz.de>
 *           Universitaet Koblenz-Landau
 *
 *  Additional information:
 *  $Id: $
 *******************************************************************************/

#include "PersonParticlePainter.h"

#include "Messages/TrackedPeopleM.h"
#include "Messages/PersonParticleFilterDataM.h"

#include <QtOpenGL>
#include <GL/glut.h>

#include "Architecture/Config/Config.h"

#define THIS PersonParticlePainter

THIS::THIS() : PainterPlugin( )
{
  setName ( "People particles (test)" );
  m_ParticleModel = SceneGraph ( "config/PersonParticle.xml" );
  m_OtherParticleModel = SceneGraph ( "config/OtherParticle.xml" );
  m_NumParticlesToDraw = Config::getInt("testParticleFilter.iNumParticlesToDraw");
}


THIS::~THIS()
{
}


void THIS::processMessage ( Message* newMessage )
{
  PainterPlugin::processMessage ( newMessage );
  switch ( newMessage->getType() )
  {
    case MessageTypes::PERSON_PARTICLE_FILTER_DATA_M:
      {
        if ( PersonParticleFilterDataM* message = Message::castTo<PersonParticleFilterDataM> ( newMessage ) )
        {
          //TRACE_INFO("Person Particle Message received");
          m_BestParticles = message->getBestCandidates();
          m_ParticleFilters = message->getParticleFilters();

          if (m_BestParticles.size() != m_ParticleFilters.size())
            TRACE_WARNING("Number of best candidates is different to the number of particle filters!");

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
//  if (m_BestParticles.size() != m_ParticleFilters.size())
//  {
//    TRACE_ERROR("Number of best candidates is different to the number of particle filters!");
//  }
//  else
//  {
//    for (unsigned int i=0; i<m_BestParticles.size();++i)
//    {
//      //TRACE_INFO("Drawing particle " << i);
//        // x front (depth), y left, z up (don't think there's anything to change here bc we receive positions already from robbie)
//        m_ParticleModel.setTranslationMatrix("center.position", BaseLib::Math::Vec3d(m_BestParticles.at(i).getCenterXPos(), m_BestParticles.at(i).getCenterYPos(), 10.0));
//        m_ParticleModel.paintGl();
//        //m_ParticleModel.findNode("center")->paintGl();
//        // Draw the rest of the X particles for each particle filter

//        unsigned int particlesLimit;
//        if(m_ParticleFilters.at(i)!=0)
//        {
//          particlesLimit = m_ParticleFilters.at(i)->numParticles();
//        }
//        else
//        {
//          TRACE_ERROR("m_ParticleFilters.at(i) is a Null-Pointer");
//          break;
//        }
//        for (unsigned int j=0; j<m_NumParticlesToDraw && j<particlesLimit; j++)
//        {
//        //    TRACE_INFO("Painting rest of the particles");
//            PersonState p = m_ParticleFilters.at(i)->getState(j);

//            m_OtherParticleModel.setTranslationMatrix("other.position", BaseLib::Math::Vec3d(p.getCenterXPos(),p.getCenterYPos(), 10.0));
//            m_OtherParticleModel.paintGl();
//        }
//    }
//  }
}



#undef THIS
