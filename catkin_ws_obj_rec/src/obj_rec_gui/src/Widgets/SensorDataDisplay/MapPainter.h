/*******************************************************************************
 *  MapPainter.h
 *
 *  (C) 2007 AG Aktives Sehen <agas@uni-koblenz.de>
 *           Universitaet Koblenz-Landau
 *
 *  Additional information:
 *  $Id: $
 *******************************************************************************/

#ifndef MapPainter_H
#define MapPainter_H

#include "PainterPlugin.h"

#include "Workers/PointOfInterest/PointOfInterest.h"

#include <QtOpenGL>

/**
 * @class  MapPainter
 * @brief  Paints all Points of Interest
 * @author David Gossow (R12)
 */
class MapPainter: public PainterPlugin
{
    Q_OBJECT

  public:

    /** @brief The constructor */
    MapPainter();

    /** @brief The destructor */
    ~MapPainter();

    /** @brief Paint everything using OpenGL */
    virtual void paint ( float next2DLayer );

  public slots:

    /** @brief Process an incoming message */
    virtual void processMessage ( Message* newMessage );

  private:

    /** @brief copies the map data into an opengl texture */
    void updateMap ( unsigned char* mapData );

    void loadGlTexture();

    /** @brief map texture data */
    unsigned char* m_TextureData;

    /** @brief specifies if the current texture has been loaded to the graphic card memory */
    bool m_TextureLoaded;

    /** @brief dimension of the map. */
    int m_MapPixelSize;
    int m_MapMmSize;

    /** @brief a texture index for the map. */
    GLuint m_TextureId;
    int m_TextureSize;

};

#endif
