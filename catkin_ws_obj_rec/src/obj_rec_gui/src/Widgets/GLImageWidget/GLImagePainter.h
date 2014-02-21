/*******************************************************************************
 *  GLImagePainter.h
 *
 *  (C) 2006 AG Aktives Sehen <agas@uni-koblenz.de>
 *           Universitaet Koblenz-Landau
 *
 * $Id $
 ******************************************************************************/

#ifndef GLImagePainter_H
#define GLImagePainter_H

#include <vector>
#include <iostream>
#include <cmath>
#include <QRgb>
#include <QtOpenGL>

#include "../../Workers/Puma2/ColorImageRGB8.h"
#include "../../Workers/Puma2/GrayLevelImage8.h"
#include "../../Workers/Puma2/ColorImageUV8.h"

#include "../../Devices/ImageGrabber/ImageGrabber.h"
#include "Workers/Math/Box2D.h"
#include "Workers/Math/Circle2D.h"
#include "Workers/Math/Line2D.h"
#include "../../Workers/VectorGraphics/VectorObject2D.h"

/**
 * @brief displays a color image (ColorImageRGB8) using OpenGL
 * @author Unknown, David Gossow (RX)
 */
class GLImagePainter {

  public:
    GLImagePainter();
    ~GLImagePainter();

    /** @brief Update the contents of the OpenGL Texture, clear the box list and render the image. */
    void setColorImage(const puma2::ColorImageRGB8* image, float aspect=1.0 );
    void setColorImage(const puma2::GrayLevelImage8* yImage, const puma2::ColorImageUV8* uvImage, float aspect=1.0 );
    void setThermalImage(const puma2::GrayLevelImage8* image, float roomTemp=30.0 );
    void setGrayLevelImage(const puma2::GrayLevelImage8* image, float aspect=1.0 );

    void setAnaglyphImage( const puma2::GrayLevelImage8* leftYImage, const puma2::GrayLevelImage8* rightYImage, float aspect=1.0 );

    /** @brief Set the pixel aspect ratio (width/height)
     *  @note  Default value is 1.0 (quadratic pixels)
     *  @note  Values above 1 will display pixels horizontally stretched
     */
    void setPixelAspectRatio(float ratio) { m_PixelAspectRatio=ratio; };

    /** @brief Adds a free form given by it's corner coordinates */
    void addVectorObject( VectorObject2D vectorObject );

    /** @brief Clear the list of painted boxes */
    void clearForms();

    void saveImage( std::string filename );

    /** @brief generate & display texture */
    void paintImage( float alpha=1.0 );

    /** @brief paint all registered boxes / lines / circles */
    void paintVectorObjects();

  private:

    // TODO this method is from RobbieGLWidget --> leave here or find better location for it
    /** @brief calculates the next larger power of two that width and height fit into */
    static int calcTextureSize( int width, int height );

  protected :

    void initTexture( unsigned resolution, unsigned imageWidth, unsigned imageHeight, ImageGrabber::ColorFormat textureFormat );
    void updateTexture( unsigned char* imageData, unsigned width, unsigned height, ImageGrabber::ColorFormat format );
    void clearTexture();

    unsigned m_ImageWidth;
    unsigned m_ImageHeight;

    float m_PixelAspectRatio;

    unsigned m_TextureResolution;

    GLuint m_TextureId;
    unsigned char* m_TextureData;
    ImageGrabber::ColorFormat m_TextureFormat;
    unsigned int m_TextureByteSize;

    std::list< VectorObject2D > m_VectorObjects;

};

#endif
