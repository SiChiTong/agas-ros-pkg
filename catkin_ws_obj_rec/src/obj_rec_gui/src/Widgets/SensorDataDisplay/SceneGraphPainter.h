/*******************************************************************************
 *  SceneGraphPainter.h
 *
 *  (C) 2007 AG Aktives Sehen <agas@uni-koblenz.de>
 *           Universitaet Koblenz-Landau
 *
 *  Additional information:
 *  $Id: $
 *******************************************************************************/

#ifndef SceneGraphPainter_H
#define SceneGraphPainter_H

#include "PainterPlugin.h"


#include "Workers/SceneGraph/SceneGraph.h"



/**
 * @class  SceneGraphPainter
 * @brief  Default Painter Plugin template
 * @author David Gossow (R12)
 */
class SceneGraphPainter: public PainterPlugin
{
  Q_OBJECT

  public:

    /** @brief The constructor */
    SceneGraphPainter();

    /** @brief The destructor */
    ~SceneGraphPainter();

    /** @brief Paint everything using OpenGL */
    virtual void paint ( float next2DLayer );

  public slots:

    /** @brief Process an incoming message */
    virtual void processMessage ( Message* newMessage );
		
		void setAutoUpdate( bool autoUpdate ) { m_AutoUpdate = autoUpdate; }

		void setSceneGraph( SceneGraph &sceneGraph );

  private:

    SceneGraph m_SceneGraph;
		
		bool m_AutoUpdate;

};

#endif
