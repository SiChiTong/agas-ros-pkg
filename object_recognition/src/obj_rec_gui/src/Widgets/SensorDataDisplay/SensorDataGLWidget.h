/*******************************************************************************
 *  SensorDataGLWidget.h
 *
 *  (C) 2006 AG Aktives Sehen <agas@uni-koblenz.de>
 *           Universitaet Koblenz-Landau
 *
 *  Additional information:
 *  $Id: SensorDataGLWidget.h 44313 2011-04-06 22:46:28Z agas $
 *******************************************************************************/

#ifndef SENSORDATAGLWIDGET_H
#define SENSORDATAGLWIDGET_H

#include <QtOpenGL>
#include <vector>
#include <QRgb>

#include <set>
#include <map>

//#include "GUI/RobbieWidget/RobbieGLWidget.h"
//#include "Architecture/Profiler/Timer.h"

#include "Vec.h"
#include "Mat.h"
//#include "Workers/SceneGraph/SceneGraph.h"

#include "PainterPlugin.h"

/**
 * @class SensorDataGLWidget
 * @author David Gossow (RX/R12)
 * @brief OpenGL widget to display sensor data and the 3D world model
 *
 * This widget displays nothing in its initial state. Additional display functions can be added as PainterPlugin objects
 */
class SensorDataGLWidget : public QGLWidget
{

    Q_OBJECT

public:

    /**
     * The constructor initializes all member variables with meaningful values and gives the
     * given parent to the constructor of the base class.
     * @param parent Pointer to the parent-widget.
     */
    SensorDataGLWidget ( QWidget *parent = 0 );

    /**
     * The destructor deletes all dynamically allocated memory.
     */
    ~SensorDataGLWidget();

    void addPainter ( PainterPlugin *painter );

    PainterPlugin* getPainter( std::string p) { return m_PaintersMap[p]; }


public slots:

    /// @brief Set clear color
    void setBgColor ( float r, float g, float b, float a = 0.0 );

    /** @brief Performs a 360 degree camera move around the robot
    * @param deltaPhi defines the angle wich the camera is turned by per step.
    **/
    void performCameraMove(); // float deltaPhi );

    /** @brief Process ingoing messages */
    //virtual void processMessage ( Message* message );

    /// @brief attach the camera to this node
    void followNode( const QString& nodeName );

    /// @brief set camera rotation relative to look-at node [deg]
    void setCameraRotation( float rotationY, float rotationZ );

    void setLookAt( BaseLib::Math::Vec3d lookAt );

protected:

    /**
     * @brief This method enables the OpenGL features that are used by the widget and sets the initial perspective.
     * It is called by Qt once while creating the widget.
     */
    void initializeGL();

    /**
     * This method is automatically called by Qt when the widget was resized. It re-sets the
     * perspective, so that everything looks fine.
     * @param w new width of the widget (given by Qt)
     * @param h new height of the widget (given by Qt)
     */
    void resizeGL ( int w, int h );

    /**
     * This is the main painting method of the widget. It is automatically called by Qt when
     * the window has to be drawn. It is also called when incoming messages (by the message
     * receive slot) change the data that is displayed.
     * The method calls the other painting methods (i.e. paintSonarData(), paintRobot(), ...)
     * if the corresponding switches (m_PaintSonarData, m_PaintRobot) are set to true.
     */
    void paintGL();

    /**
     * When the widget receives this event, the mouse position is saved to enable zooming and tilting
     * with moving the mouse.
     * @param event The event that Qt gives to this widget when a mouse button was pressed above this widget.
     */
    void mousePressEvent ( QMouseEvent* event );

    /**
     * This event is caught by the widget to implement zooming and changing the view angle by dragging the mouse.
     * The pressed button is checked and the zoom or the view angle are changed according to the distance
     * the mouse was moved.
     * It is only used if the left or the right mouse button is pressed.
     * @param event The event that Qt gives to this widget when the mouse is moving across this widget.
     */
    void mouseMoveEvent ( QMouseEvent* event );

    /**
     * This event is used to manipulate the z-position (zoom) of the camera.
     * @param event The event that Qt gives to this widget when the scroll wheel of the mouse was turned
     * above this widget.
     */
    void wheelEvent ( QWheelEvent* event );

    /** @return Minimum size of the widget */
    virtual QSize minimumSizeHint() const;

    /** @return Preferred size of this widget. */
    virtual QSize sizeHint() const;

private:

    void setProjectionMatrix();

    //SceneGraph m_SceneGraph; // TODO
    bool m_SceneGraphInitialized;

    /// Node in SceneGraph to follow
    std::string m_LookAtNode;

    /// Look-At coordinates relative to LookAtNode
    BaseLib::Math::Vec3d m_LookAt; // TODO

    /// Camera Position relative to LookAt point (degrees / mm)
    float m_CameraRotationZ;
    float m_CameraRotationY;
    float m_CameraDistance;

    float m_CameraOpeningAngle;

    /** @brief variables used for camera turn */
    float m_CameraTurnStartTime;
    int m_CameraTurning;
    QTimer* m_RedrawTimer;

    /** @brief the position of the mouse when a button is pressed. */
    QPoint m_MousePosOld;

    // Timer m_Timer; TODO

    std::list<PainterPlugin*> m_Painters;
    std::map<std::string, PainterPlugin*> m_PaintersMap;

    bool m_CameraImageMode;

    int m_ViewportWidth;
    int m_ViewportHeight;

    bool m_ForceRedraw;

    static int WidgetCounter;
};
#endif
