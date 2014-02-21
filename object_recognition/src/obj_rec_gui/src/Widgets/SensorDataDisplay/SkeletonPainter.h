/*******************************************************************************
 *  SkeletonPainter.h
 *
 *  (C) 2011 AG Aktives Sehen <agas@uni-koblenz.de>
 *           Universitaet Koblenz-Landau
 *
 *  Additional information:
 *  $Id: $
 *******************************************************************************/

#ifndef SkeletonPainter_H
#define SkeletonPainter_H

#include "PainterPlugin.h"

#include "Workers/PeopleTracker/TrackedPerson.h"
#include "Workers/SceneGraph/SceneGraph.h"

#include <QtOpenGL>

/**
 * @class  SkeletonPainter
 * @brief  Paints the information about the tracked skeletons
 * @author Carmen Navarro (R16)
 */
class SkeletonPainter: public PainterPlugin
{
    Q_OBJECT

  public:

    /** @brief The constructor */
    SkeletonPainter();

    /** @brief The destructor */
    ~SkeletonPainter();

    /** @brief Paint everything using OpenGL */
    virtual void paint ( float next2DLayer );

  public slots:

    /** @brief Process an incoming message */
    virtual void processMessage ( Message* newMessage );

  private:
    std::vector<SceneGraph> m_TrackedSkeletons;
    SceneGraph m_SkeletonModel;
};

#endif // SKELETONPAINTER_H
