/*******************************************************************************
 *  ParticlePainter.h
 *
 *  (C) 2007 AG Aktives Sehen <agas@uni-koblenz.de>
 *           Universitaet Koblenz-Landau
 *
 *  Additional information:
 *  $Id: $
 *******************************************************************************/

#ifndef ParticlePainter_H
#define ParticlePainter_H

#include "PainterPlugin.h"

#include "Messages/ParticleDataM.h"



/**
 * @class  ParticlePainter
 * @brief  Paints arrows representing the SLAM particles
 * @author David Gossow (RX-R12)
 */
class ParticlePainter: public PainterPlugin
{
    Q_OBJECT

  public:

    /** @brief The constructor */
    ParticlePainter();

    /** @brief The destructor */
    ~ParticlePainter();

    /** @brief Paint everything using OpenGL */
    virtual void paint ( float next2DLayer );

  public slots:

    /** @brief Process an incoming message */
    virtual void processMessage ( Message* newMessage );

  private:

    std::vector<ParticleDataM::ParticleData> m_ParticleData;

};

#endif
