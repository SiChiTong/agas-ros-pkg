/*******************************************************************************
 *  MyPainter.h
 *
 *  (C) 2007 AG Aktives Sehen <agas@uni-koblenz.de>
 *           Universitaet Koblenz-Landau
 *
 *  Additional information:
 *  $Id: $
 *******************************************************************************/

#ifndef TOFPAINTER_H
#define TOFPAINTER_H

#include "PainterPlugin.h"
#include "Workers/BaseLib/Vec.h"
#include "Workers/BaseLib/Vec3.h"
#include "Workers/SceneGraph/SceneGraph.h"

/**
 * @class  TofPainter
 * @brief  Time-of-Flight camera Painter Plugin
 * @author amuetzel
 */
class TofPainter: public PainterPlugin
{
    Q_OBJECT

  public:

    /** @brief The constructor */
    TofPainter();

    /** @brief The destructor */
    ~TofPainter();

    /** @brief Paint everything using OpenGL */
    virtual void paint ( float next2DLayer );

  public slots:

    /** @brief Process an incoming message */
    virtual void processMessage ( Message* newMessage );

  private:

    void initBuffers( int height=0, int width = 0 );

    SceneGraph m_SceneGraph;
    bool m_SceneGraphInitialized;

    int m_Height;
    int m_Width;

    float *m_Amplitudes;
    float *m_Depths;
    BaseLib::Math::Vec3f *m_Points;

    float m_ColorFactor;
    float m_AmplitudeScaling;
    float m_AmplitudeThreshold;
    int m_ViewMode;

    unsigned int m_LastDataTime;
    unsigned int m_DisplayDuration;

    double m_Transformation[16];
};

#endif
