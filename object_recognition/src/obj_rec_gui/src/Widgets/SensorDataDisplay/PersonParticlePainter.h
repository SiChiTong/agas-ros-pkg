/*******************************************************************************
 *  PersonParticlePainter.h
 *
 *  (C) 2011 AG Aktives Sehen <agas@uni-koblenz.de>
 *           Universitaet Koblenz-Landau
 *
 *  Additional information:
 *  $Id: $
 *******************************************************************************/

#ifndef PersonParticlePainter_H
#define PersonParticlePainter_H

#include "PainterPlugin.h"

#include "Workers/PeopleTracker/TrackedPerson.h"
#include "Workers/SceneGraph/SceneGraph.h"
#include "Workers/NewParticleFilter/PersonParticleFilter/PersonState.h"
#include "Workers/NewParticleFilter/GenericParticleFilter/ParticleFilter.h"

#include <QtOpenGL>

/**
 * @class  PersonParticlePainter
 * @brief  Paints the information about the best particles in the particle filters for people tracking
 * @author Carmen Navarro (R16)
 */
class PersonParticlePainter: public PainterPlugin
{
    Q_OBJECT

  public:

    /** @brief The constructor */
    PersonParticlePainter();

    /** @brief The destructor */
    ~PersonParticlePainter();

    /** @brief Paint everything using OpenGL */
    virtual void paint ( float next2DLayer );

  public slots:

    /** @brief Process an incoming message */
    virtual void processMessage ( Message* newMessage );

  private:
    std::vector<PersonState> m_BestParticles;
    std::vector< AF::ParticleFilter<PersonState>* > m_ParticleFilters;

    int m_NumParticlesToDraw;

    SceneGraph m_ParticleModel;
    SceneGraph m_OtherParticleModel;
};

#endif // SKELETONPAINTER_H
