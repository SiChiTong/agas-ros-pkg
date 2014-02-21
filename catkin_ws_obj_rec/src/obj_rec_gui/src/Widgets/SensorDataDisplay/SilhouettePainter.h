/*******************************************************************************
 *  SilhouettePainter.h
 *
 *  (C) 2007 AG Aktives Sehen <agas@uni-koblenz.de>
 *           Universitaet Koblenz-Landau
 *
 *  Additional information:
 *  $Id: $
 *******************************************************************************/

#ifndef SilhouettePainter_H
#define SilhouettePainter_H

#include "PainterPlugin.h"

#include "Workers/PeopleTracker/TrackedPerson.h"
#include "Workers/SceneGraph/SceneGraph.h"

#include <QtOpenGL>

/**
 * @class  SilhouettePainter
 * @brief  
 * @author 
 */
class SilhouettePainter: public PainterPlugin
{
    Q_OBJECT

public:

    /** @brief The constructor */
    SilhouettePainter();

    /** @brief The destructor */
    ~SilhouettePainter();

    /** @brief Paint everything using OpenGL */
    virtual void paint ( float next2DLayer );

public slots:

    /** @brief Process an incoming message */
    virtual void processMessage ( Message* newMessage );

private:


    SceneGraph m_PersonModel;
    SceneGraph m_HeadWithHandModel;
    SceneGraph m_HandModel;
    vector< BaseLib::Math::Vec3d > m_SilhouettesPositions3d;
    vector< BaseLib::Math::Vec3d > m_HeadsWithHand;
    vector< BaseLib::Math::Vec3d > m_Hands;


};

#endif
