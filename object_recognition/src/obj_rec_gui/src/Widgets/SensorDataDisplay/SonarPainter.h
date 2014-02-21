/*******************************************************************************
 *  SonarPainter.h
 *
 *  (C) 2007 AG Aktives Sehen <agas@uni-koblenz.de>
 *           Universitaet Koblenz-Landau
 *
 *  Additional information:
 *  $Id: $
 *******************************************************************************/

#ifndef SonarPainter_H
#define SonarPainter_H

#include "PainterPlugin.h"

#include "Workers/Math/Pose.h"


/**
 * @class  SonarPainter
 * @brief  Paints the current sonar measurements
 * @author Unknown (R6-8), David Gossow (RX-R12)
 */
class SonarPainter: public PainterPlugin
{
    Q_OBJECT

  public:

    /** @brief The constructor */
    SonarPainter();

    /** @brief The destructor */
    ~SonarPainter();

    /** @brief Paint everything using OpenGL */
    virtual void paint ( float next2DLayer );

  public slots:

    /** @brief Process an incoming message */
    virtual void processMessage ( Message* newMessage );

  private:

    /** @brief the current position of the robot in mm. */
    Pose m_RobotPose;

    /** @brief the sonar data. */
    std::vector<int> m_SonarData;
};

#endif
