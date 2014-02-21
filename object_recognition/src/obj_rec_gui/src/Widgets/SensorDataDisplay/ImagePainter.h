/*******************************************************************************
 *  ImagePainter.h
 *
 *  (C) 2007 AG Aktives Sehen <agas@uni-koblenz.de>
 *           Universitaet Koblenz-Landau
 *
 *  Additional information:
 *  $Id: $
 *******************************************************************************/

#ifndef ImagePainter_H
#define ImagePainter_H

#include "PainterPlugin.h"

#include "Workers/ImageSources/ImageSources.h"
#include "Workers/VectorGraphics/VectorObject2D.h"
#include "Workers/BaseLib/Mat.h"
#include "GUI/Widgets/GLImageWidget/GLImagePainter.h"

#include "Workers/Puma2/ColorImageRGB8.h"

#include <map>
#include <string>


/**
 * @class  ImagePainter
 * @brief  Default Painter Plugin template
 * @author David Gossow (R12)
 */
class ImagePainter: public PainterPlugin
{
  Q_OBJECT

  public:

    /** @brief The constructor */
    ImagePainter();

    /** @brief The destructor */
    ~ImagePainter();

    /** @brief Paint everything using OpenGL */
    virtual void paint ( float next2DLayer );

  public slots:

    /** @brief Process an incoming message */
    virtual void processMessage ( Message* newMessage );

    /** @brief Called when the host widget follows another node in the scenegraph */
    virtual void nodeSelected( std::string nodeName );

  private:

    BaseLib::Math::Mat4d m_ImageToWorld;

    std::map<std::string, ImageSources::SourceId> m_ImageSources;

    std::list< VectorObject2D > m_VectorObjects;

    string m_NodeName;

    GLImagePainter m_GLImagePainter;
};

#endif
