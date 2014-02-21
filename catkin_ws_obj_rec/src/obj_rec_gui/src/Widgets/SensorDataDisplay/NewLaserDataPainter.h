/*******************************************************************************
 *  NewLaserDataPainter.h
 *
 *  (C) 2007 AG Aktives Sehen <agas@uni-koblenz.de>
 *           Universitaet Koblenz-Landau
 *
 *  Additional information:
 *  $Id: $
 *******************************************************************************/

#ifndef NewLaserDataPainter_H
#define NewLaserDataPainter_H

#include <QtOpenGL>
#include <vector>

#include <sensor_msgs/LaserScan.h>

#include "PainterPlugin.h"
#include "Vec.h"

/**
 * @class  NewLaserDataPainter
 * @brief  Paints the current LRF measurements
 * @author Unknown (R6-RX), David Gossow (RX-R12)
 */
class NewLaserDataPainter: public PainterPlugin
{
    Q_OBJECT

  public:

    /** @brief The constructor */
    NewLaserDataPainter();

    /** @brief The destructor */
    ~NewLaserDataPainter();

    /** @brief Paint everything using OpenGL */
    virtual void paint ( float next2DLayer );

    void updateData(const sensor_msgs::LaserScan::ConstPtr& msg);

  public slots:

    /** @brief Process an incoming message */
    // virtual void processMessage ( Message* newMessage );

  private:

    /** @brief laser data as 3D points in world coordinates */
    std::vector<BaseLib::Math::Vec3f> m_WorldPoints;
    /** @brief measurement ranges */
    std::vector<float> m_Ranges;
    /** @brief maximum scanner range */
    float m_RangeMax;
    /** @brief number of measurement points per laser scan */
    unsigned int m_NumLaserPoints;
    /** @brief flag for initialization on first received laser scan */
    bool m_FirstScan;
    /** @brief lookup-table for angles */
    std::vector<BaseLib::Math::Vec3f> m_UnitLaserDirections;
    /** @brief scale factor for improved visualization */
    float m_ScaleFactor;
};

#endif
