/*******************************************************************************
 *  PainterPlugin.h
 *
 *  (C) 2007 AG Aktives Sehen <agas@uni-koblenz.de>
 *           Universitaet Koblenz-Landau
 *
 *  Additional information:
 *  $Id: $
 *******************************************************************************/

#ifndef PainterPlugin_H
#define PainterPlugin_H

#include <string>
#include <QObject>

class QGLWidget;

/**
 * @class  PainterPlugin
 * @brief  Abstract class for SensorDataGLWidget plugins that paint some kind of 3D data using OpenGL
 * @author David Gossow (R12)
 */
class PainterPlugin: public QObject
{
    Q_OBJECT

  public:

    /** @brief The constructor */
    PainterPlugin( );

    /** @brief The destructor */
    ~PainterPlugin();

    /** @brief this is called by the host widget (e.g. SensorDataGLWidget) and should contain the OpenGL drawing routines */
    virtual void paint ( float next2DLayer ) = 0;

    /** @brief specifies if this plugin is currently active. */
    void setVisible ( bool visible ) { m_Visible = visible; }
    bool isVisible( ) { return m_Visible; }

    /** @brief Specifies the name that is displayed e.g. in the options dialogue */
    void setName ( std::string name ) { m_Name = name; }
    std::string getName( ) { return m_Name; }

    /** @brief If the needsRedraw flag is set to true after processing a message, the host widget will know it neeeds to redraw the scene */
    bool needsRedraw() { return m_NeedsRedraw; }
    void wasRedrawn() { m_NeedsRedraw = false; }

  public slots:

    /** @brief Called when the host widget changes it lookAt node in the scenegraph */
    virtual void nodeSelected() {};

    /** @brief Do the message processing. Has to be reimplemented in derived classes */
   // virtual void processMessage ( Message* message UNUSED ) { }

    /** @brief Called when the host widget follows another node in the scenegraph */
    virtual void nodeSelected( std::string nodeName ) {};
    // TODO: vorher so:  virtual void nodeSelected( std::string nodeName UNUSED) {};

    /** @brief Called by parent widget. The painter class can then access functions of the parent, e.g. for text rendering */
    void setParent( QGLWidget* parentWidget ) { m_ParentWidget=parentWidget; }

  protected:

    /// @brief request a redraw on the next OpenGL update
    void requestRedraw() { m_NeedsRedraw = true; }

    QGLWidget* m_ParentWidget;

  private:

    bool m_Visible;

    std::string m_Name;

    bool m_NeedsRedraw;

};

#endif
