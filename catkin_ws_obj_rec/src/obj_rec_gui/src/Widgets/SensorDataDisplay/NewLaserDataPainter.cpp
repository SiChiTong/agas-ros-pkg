/*******************************************************************************
 *  NewLaserDataPainter.cpp
 *
 *  (C) 2007 AG Aktives Sehen <agas@uni-koblenz.de>
 *           Universitaet Koblenz-Landau
 *
 *  Additional information:
 *  $Id: $
 *******************************************************************************/

#include "NewLaserDataPainter.h"

//#include "Messages/LaserDataM.h"
//#include "Messages/SceneGraphM.h"

#include <QtOpenGL>
#include <GL/glut.h>
#include <iostream>

#define THIS NewLaserDataPainter

THIS::THIS() : PainterPlugin( )
{
    setName ( "2D Laser Data" );
    m_FirstScan = true;
    m_ScaleFactor = 1000; // input data is in meters, scale it for better visualization
}

THIS::~THIS()
{
}


void THIS::updateData(const sensor_msgs::LaserScan::ConstPtr& scan)
{
    if(m_FirstScan)
    {
        m_FirstScan = false;
        m_RangeMax = scan->range_max;
        // determine number of datapoints in lrf scan
        m_NumLaserPoints = (scan->angle_max - scan->angle_min) / scan->angle_increment;

        // build lookup-table
        float alpha = scan->angle_min;
        for (unsigned int i = 0; i < m_NumLaserPoints; i++)
        {
          float x = cos(alpha);
          float y = sin(alpha);
          BaseLib::Math::Vec3f unitPoint(x, y, 0);
          m_UnitLaserDirections.push_back(unitPoint * m_ScaleFactor);
          alpha += scan->angle_increment;
        }
    }

    m_WorldPoints.clear();
    m_Ranges.clear();

    // process all data in laser scan
    for(unsigned i = 0; i < m_NumLaserPoints; i++)
    {
        float range = scan->ranges[i];
        m_Ranges.push_back(range);
        if(range < scan->range_min || range > scan->range_max)
        {
            continue;
        }
        m_WorldPoints.push_back( m_UnitLaserDirections.at(i) * range);
    }

    requestRedraw();
}


void THIS::paint ( float next2DLayer )
{
    glPushMatrix();
    //  TODO:  BaseLib::Math::Mat4d laserToWorld = m_SceneGraph.getTransformation( lrfId, "World" );

    glBlendFunc ( GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA );
    glEnable ( GL_BLEND );

    //Paint dots
    for ( unsigned i=0; i < m_WorldPoints.size(); i++ )
    {
        float t = float ( m_Ranges.at(i) ) / m_RangeMax * 3;
        if ( t < 0.0 ) t = 0.0;
        if ( t > 1.0 ) t = 1.0;

        glColor4f ( 1.0, t, 0.0, 1.0 );

        glPointSize( 5.0 );
        glBegin( GL_POINTS );
        glVertex3f( m_WorldPoints.at(i).x, m_WorldPoints.at(i).y, m_WorldPoints.at(i).z );
        glEnd();
    }

    glPopMatrix();
}


#undef THIS
