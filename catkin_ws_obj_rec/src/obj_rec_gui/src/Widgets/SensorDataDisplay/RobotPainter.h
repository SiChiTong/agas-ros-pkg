/*******************************************************************************
 *  RobotPainter.h
 *
 *  (C) 2007 AG Aktives Sehen <agas@uni-koblenz.de>
 *           Universitaet Koblenz-Landau
 *
 *  Additional information:
 *  $Id: $
 *******************************************************************************/

#ifndef RobotPainter_H
#define RobotPainter_H

#include "PainterPlugin.h"

#include "../../Math/Pose.h"


/**
 * @class  RobotPainter
 * @brief  Paints the robot
 * @author Unknown (R6-8), David Gossow (RX-R12)
 */
class RobotPainter: public PainterPlugin
{
    Q_OBJECT

  public:

    /** @brief The constructor */
    RobotPainter();

    /** @brief The destructor */
    ~RobotPainter();

    /** @brief Paint everything using OpenGL */
    virtual void paint ( float next2DLayer );

  public slots:

    /** @brief Process an incoming message */
   // virtual void processMessage ( Message* newMessage );

  private:

    /** @brief the current position of the robot in mm. */
    Pose m_RobotPose;

    float m_RollAngle;
    float m_PitchAngle;

};

#endif
