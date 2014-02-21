/*******************************************************************************
 *  POIPainter.h
 *
 *  (C) 2007 AG Aktives Sehen <agas@uni-koblenz.de>
 *           Universitaet Koblenz-Landau
 *
 *  Additional information:
 *  $Id: $
 *******************************************************************************/

#ifndef POIPainter_H
#define POIPainter_H

#include "PainterPlugin.h"

#include "Workers/PointOfInterest/PointOfInterest.h"


/**
 * @class  POIPainter
 * @brief  Paints all Points of Interest
 * @author David Gossow (R12)
 */
class POIPainter: public PainterPlugin
{
    Q_OBJECT

  public:

    /** @brief The constructor */
    POIPainter();

    /** @brief The destructor */
    ~POIPainter();

    /** @brief Paint everything using OpenGL */
    virtual void paint ( float next2DLayer );

  public slots:

    /** @brief Process an incoming message */
    virtual void processMessage ( Message* newMessage );

  private:

    list<PointOfInterest> m_Pois;

};

#endif
