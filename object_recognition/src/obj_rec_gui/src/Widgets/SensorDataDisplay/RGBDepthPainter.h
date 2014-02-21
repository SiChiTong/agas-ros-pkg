/*******************************************************************************
 *  RGBDepthPainter.h
 *
 *  (C) 2007 AG Aktives Sehen <agas@uni-koblenz.de>
 *           Universitaet Koblenz-Landau
 *
 *  Additional information:
 *  $Id: $
 *******************************************************************************/

#ifndef RGBDepthPainter_H
#define RGBDepthPainter_H

#include "PainterPlugin.h"

#include "Workers/BaseLib/Vec.h"
#include "Workers/BaseLib/Mat.h"
#include "Messages/RGBDepthM.h"

#include <vector>

/**
 * @class  RGBDepthPainter
 * @brief  Paints the RGBDepthM
 * @author Susanne Thierfelder (R16)
 */
class RGBDepthPainter: public PainterPlugin
{
    Q_OBJECT

  public:

    /** @brief The constructor */
    RGBDepthPainter();

    /** @brief The destructor */
    ~RGBDepthPainter();

    /** @brief Paint everything using OpenGL */
    virtual void paint ( float next2DLayer );

  public slots:

    /** @brief Process an incoming message */
    virtual void processMessage ( Message* newMessage );

  private:

    BaseLib::Math::Mat4d m_RobotToWorld;
    std::vector<BaseLib::Math::Mat4d> m_ValidRobotToWorld;

    /** Data from the RGBDepthM needs to be copied, because contents are deleted later */

    unsigned m_Rows, m_Columns;
    std::vector< std::vector<BaseLib::Math::Vec3d> > m_Points;
    std::vector<puma2::ColorImageRGB8> m_RgbImage;

    bool m_RgbImageAvailable;
};

#endif
