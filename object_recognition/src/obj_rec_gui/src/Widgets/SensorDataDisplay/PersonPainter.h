/*******************************************************************************
 *  PersonPainter.h
 *
 *  (C) 2007 AG Aktives Sehen <agas@uni-koblenz.de>
 *           Universitaet Koblenz-Landau
 *
 *  Additional information:
 *  $Id: $
 *******************************************************************************/

#ifndef PersonPainter_H
#define PersonPainter_H

#include "PainterPlugin.h"

#include "Workers/PeopleTracker/TrackedPerson.h"
#include "Workers/SceneGraph/SceneGraph.h"

#include <QtOpenGL>

/**
 * @class  PersonPainter
 * @brief  Paints the currently tracked person
 * @author David Gossow (R12)
 */
class PersonPainter: public PainterPlugin
{
    Q_OBJECT

  public:

    /** @brief The constructor */
    PersonPainter();

    /** @brief The destructor */
    ~PersonPainter();

    /** @brief Paint everything using OpenGL */
    virtual void paint ( float next2DLayer );

  public slots:

    /** @brief Process an incoming message */
    virtual void processMessage ( Message* newMessage );

  private:

		TrackedPerson m_Operator;
    vector<TrackedPerson> m_TrackedPeople;
		
		SceneGraph m_PersonModel;
};

#endif
