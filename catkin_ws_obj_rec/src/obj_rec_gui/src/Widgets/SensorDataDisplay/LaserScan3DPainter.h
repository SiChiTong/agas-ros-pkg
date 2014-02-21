/*******************************************************************************
 *  LaserScan3DPainter.h
 *
 *  (C) 2007 AG Aktives Sehen <agas@uni-koblenz.de>
 *           Universitaet Koblenz-Landau
 *
 *  Additional information:
 *  $Id: $
 *******************************************************************************/

#ifndef LaserScan3DPainter_H
#define LaserScan3DPainter_H

#include "PainterPlugin.h"

#include "Workers/BaseLib/Vec.h"
#include "Workers/BaseLib/Mat.h"
#include "Messages/Debug3DM.h"

#include <vector>

/**
 * @class  LaserScan3DPainter
 * @brief  Paints the LaserScan3DPointM
 * @author David Gossow
 */
class LaserScan3DPainter: public PainterPlugin
{
    Q_OBJECT

  public:

    /** @brief The constructor */
    LaserScan3DPainter();

    /** @brief The destructor */
    ~LaserScan3DPainter();

    /** @brief Paint everything using OpenGL */
    virtual void paint ( float next2DLayer );

  public slots:

    /** @brief Process an incoming message */
    virtual void processMessage ( Message* newMessage );

  private:

    std::map< std::string, std::vector<Debug3DM::VertexSet> > m_VertexSets;
    std::map< std::string, BaseLib::Math::Mat4d > m_TransformationMatrices;

    BaseLib::Math::Mat4d m_RobotToWorld;
		int m_RawDataCounter;
};

#endif
