/*******************************************************************************
 *  PathPainter.h
 *
 *  (C) 2007 AG Aktives Sehen <agas@uni-koblenz.de>
 *           Universitaet Koblenz-Landau
 *
 *  Additional information:
 *  $Id: $
 *******************************************************************************/

#ifndef PathPainter_H
#define PathPainter_H

#include "PainterPlugin.h"

#include "Workers/Math/Pose.h"


/**
 * @class  PathPainter
 * @brief  Paints the planned navigation path
 * @author Unknown (R6-X), David Gossow (RX-R12)
 */
class PathPainter: public PainterPlugin
{
    Q_OBJECT

  public:

    /** @brief The constructor */
    PathPainter();

    /** @brief The destructor */
    ~PathPainter();

    /** @brief Paint everything using OpenGL */
    virtual void paint ( float next2DLayer );

  public slots:

    /** @brief Process an incoming message */
    virtual void processMessage ( Message* newMessage );

  private:

    /** @brief Store the path points. */
    std::vector<float> m_PathXCoordinates;
    std::vector<float> m_PathYCoordinates;

};

#endif
