/*******************************************************************************
 *  GLImageWidget.h
 *
 *  (C) 2006 AG Aktives Sehen <agas@uni-koblenz.de>
 *           Universitaet Koblenz-Landau
 *
 * $Id $
 ******************************************************************************/

#ifndef GLImageWidget_H
#define GLImageWidget_H

#include <vector>
#include <iostream>
#include <cmath>
#include <QRgb>
#include <QGLWidget>

#include "../../Workers/Puma2/ColorImageRGB8.h"
#include "../../Workers/Puma2/GrayLevelImage8.h"
#include "../../Workers/Puma2/ColorImageUV8.h"

#include "../../Devices/ImageGrabber/ImageGrabber.h"
#include "Workers/Math/Box2D.h"
#include "Workers/Math/Circle2D.h"
#include "../../Workers/VectorGraphics/VectorObject2D.h"

/**
 * @brief displays a color image (ColorImageRGB8) using OpenGL
 * @author Unknown, David Gossow (RX)
 */
class GLImageWidget : public QGLWidget {

    Q_OBJECT

  public:
    GLImageWidget(QGLWidget *parent = 0);
    ~GLImageWidget();

  public slots:

    /** @brief Update the contents of the OpenGL Texture, clear the box list and render the image. */
    void setColorImage(const unsigned char *image, unsigned width, unsigned height, float aspect=1.0 );
    void setColorImage(const puma2::ColorImageRGB8* image, float aspect=1.0 );
    void setColorImage(const puma2::GrayLevelImage8* imageY, const puma2::ColorImageUV8* imageUV, float aspect=1.0 );
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

    /** @brief zoom into current shown image*/
    void zoom(float z);

  private:

    /** @brief generate & display texture */
    void paintImage();

    /** @brief paint all registered boxes / lines / circles */
    void paintVectorObjects();

    /** @brief set up OpenGL */
    void initializeGL();

    /** @brief called when the window size changes */
    void resizeGL(int w, int h);

    /** @brief render image */
    void paintGL();

    // USER INTERACTION ///////////////

    /** @brief resets the display */
    void mouseDoubleClickEvent(QMouseEvent *event);

    /** @brief called when a mouse button is pressed */
    void mousePressEvent(QMouseEvent *event);

    /** @brief if mouse button is pressed, move images */
    void mouseMoveEvent(QMouseEvent* event);

    /** @brief used to zoom the image */
    void wheelEvent(QWheelEvent *event);




    // TODO originally in RobbieWidget
    int calcTextureSize( int width, int height );


 protected :

    void initTexture( unsigned resolution, unsigned imageWidth, unsigned imageHeight, ImageGrabber::ColorFormat textureFormat );
    void updateTexture( unsigned char* imageData, unsigned width, unsigned height, ImageGrabber::ColorFormat format );
    void clearTexture();

    QPoint m_MousePosOld;

    float m_Zoom;
    float m_PosX;
    float m_PosY;

    unsigned m_ImageWidth;
    unsigned m_ImageHeight;

    unsigned m_ViewportWidth;
    unsigned m_ViewportHeight;

    float m_PixelAspectRatio;

    /// @note the texture resolution is the next larger potency of 2 that the image fits into
    unsigned m_TextureResolution;

    GLuint m_TextureId;
    unsigned char* m_TextureData;
    ImageGrabber::ColorFormat m_TextureFormat;
    unsigned int m_TextureByteSize;

    std::list< VectorObject2D > m_VectorObjects;

};

#endif
