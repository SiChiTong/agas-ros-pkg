/*******************************************************************************
 *  RobotPosesPainter.h
 *
 *  (C) 2007 AG Aktives Sehen <agas@uni-koblenz.de>
 *           Universitaet Koblenz-Landau
 *
 *  Additional information:
 *  $Id: $
 *******************************************************************************/

#ifndef RobotPosesPainter_H
#define RobotPosesPainter_H

#include "PainterPlugin.h"

#include "Messages/ParticleDataM.h"

#include <vector>

/**
 * @class  RobotPosesPainter
 * @brief  Paints arrows representing the pose hypothesis of all particle filters
 * @author Unknown (R12), David Gossow (RX-R12)
 */
class RobotPosesPainter: public PainterPlugin
{
    Q_OBJECT

  public:

    /** @brief The constructor */
    RobotPosesPainter();

    /** @brief The destructor */
    ~RobotPosesPainter();

    /** @brief Paint everything using OpenGL */
    virtual void paint ( float next2DLayer );

  public slots:

    /** @brief Process an incoming message */
    virtual void processMessage ( Message* newMessage );

  private:

    std::vector<ParticleDataM::ParticleData> m_ParticleData;

};

#endif
