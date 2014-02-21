/*******************************************************************************
 *  ParticleGridPainter.h
 *
 *  (C) 2007 AG Aktives Sehen <agas@uni-koblenz.de>
 *           Universitaet Koblenz-Landau
 *
 *  Additional information:
 *  $Id: $
 *******************************************************************************/

#ifndef ParticleGridPainter_H
#define ParticleGridPainter_H

#include "PainterPlugin.h"

#include "Messages/ParticleDataM.h"
#include "Workers/Math/Pose.h"

/**
 * @class  ParticleGridPainter
 * @brief  Paints a weighted density grid of all SLAM particles
 * @author Christian Fuchs (R12)
 */
class ParticleGridPainter: public PainterPlugin
{
    Q_OBJECT

  public:

    /** @brief The constructor */
    ParticleGridPainter();

    /** @brief The destructor */
    ~ParticleGridPainter();

    /** @brief Paint everything using OpenGL */
    virtual void paint ( float next2DLayer );

  public slots:

    /** @brief Process an incoming message */
    virtual void processMessage ( Message* newMessage );

  private:

    void generateMesh();

    std::vector<ParticleDataM::ParticleData> m_ParticleData;
    float * m_WeightGrid;

    /** @brief mesh size used for the 3d visualization of the particles */
    int m_ParticleBinCount;
    float m_ParticleBinDisplaySize;
    float m_ParticlePeekHeight;
    float m_HeightPeek;

    unsigned m_MinX;
    unsigned m_MinY;
    unsigned m_MaxX;
    unsigned m_MaxY;

    Pose m_RobotPose;

};

#endif
