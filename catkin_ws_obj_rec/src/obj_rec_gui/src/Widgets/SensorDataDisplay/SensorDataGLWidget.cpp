/*******************************************************************************
 *  SensorDataGLWidget.cpp
 *
 *  (C) 2006 AG Aktives Sehen <agas@uni-koblenz.de>
 *           Universitaet Koblenz-Landau
 *
 *  Additional information:
 *  $Id: SensorDataGLWidget.cpp 44313 2011-04-06 22:46:28Z agas $
 *******************************************************************************/


//#include <QtOpenGL>
#include <GL/glut.h>
#include <iostream>
#include <cmath>
#include <sstream>
#include <fstream>

#include "SensorDataGLWidget.h"

//#include "Architecture/Config/Config.h"
//#include "Architecture/Tracer/Tracer.h"
//#include "Architecture/Singleton/Clock.h"

//#include "Messages/SceneGraphM.h"

#define THIS SensorDataGLWidget

using namespace std;

int THIS::WidgetCounter = 0;

// the zooming step that is made when the mouse wheel is turned
const float ZOOM_STEP = 250.0;

THIS::THIS ( QWidget *parent ) : QGLWidget ( parent )
{
    m_CameraTurning = false;
    m_CameraTurnStartTime = 0;

    // default cursor = drag cursor
    setCursor ( Qt::SizeAllCursor );

    m_ViewportWidth = 100;
    m_ViewportHeight = 100;

    followNode( "RobotView" );

    ostringstream s;
    s << "3D Sensor Display " << WidgetCounter;
    WidgetCounter++;
  //  m_Timer = Timer ( ProfilerEntry::CODE_SEGMENT, "GUI Thread", s.str() );

    setBgColor ( 0,0,0,0 );

    m_RedrawTimer = new QTimer( this ); // create internal timer
    connect( m_RedrawTimer, SIGNAL(timeout()), SLOT( updateGL() ) );
    m_RedrawTimer->start( 1000/25 );

}


THIS::~THIS()
{
}

//void THIS::processMessage ( Message* newMessage )
//{
//    ostringstream stream;

//    m_Timer.startMeasure();
//    switch ( newMessage->getType() )
//    {
//    case MessageTypes::SCENE_GRAPH_M:
//        {
//            if ( SceneGraphM* message = Message::castTo<SceneGraphM> ( newMessage ) )
//            {
//                m_SceneGraph = message->getSceneGraph();
//            }
//            break;
//        }

//    default:
//        break;
//    }

//    //call processMessage functions of all plugins
//    list<PainterPlugin*>::iterator it = m_Painters.begin();
//    while ( it != m_Painters.end() )
//    {
//        if ( ( *it )->isVisible() )
//        {
//            ( *it )->processMessage ( newMessage );
//        }
//        it++;
//    }

//    m_Timer.submit();
//}

void THIS::followNode( const QString& nodeName )
{
    m_LookAtNode = nodeName.toStdString();

    //tell painters that node has changed
    list<PainterPlugin*>::iterator it = m_Painters.begin();
    while ( it != m_Painters.end() )
    {
        (*it)->nodeSelected( m_LookAtNode );
        it++;
    }

    if ( m_LookAtNode.find( "Image" ) == m_LookAtNode.length()-5 )
    {
        m_LookAtNode = m_LookAtNode.substr( 0, m_LookAtNode.length()-5 );
        m_CameraRotationZ = 0;
        m_CameraRotationY = 0;
        m_CameraDistance = 0;
        m_CameraOpeningAngle = 60.0;
        m_CameraImageMode = true;
    }
    else
    {
        m_CameraRotationZ = 0;
        m_CameraRotationY = 30;
        m_CameraDistance = 3000;
        m_CameraOpeningAngle = 60.0;
        m_CameraImageMode = false;
    }

    m_LookAt = BaseLib::Math::Vec3d(0,0,0);

    m_ForceRedraw = true;
    updateGL();
}


void THIS::initializeGL()
{
    // initial opengl settings
    glEnable ( GL_DEPTH_TEST );
}

void THIS::setProjectionMatrix()
{
    // 	TRACE_INFO( "Setting projection to opening angle: " << m_CameraOpeningAngle );

    glMatrixMode ( GL_PROJECTION );
    glLoadIdentity();
    gluPerspective ( m_CameraOpeningAngle, ( float ) m_ViewportWidth / ( float ) m_ViewportHeight, 150, 100000 );

    glMatrixMode ( GL_MODELVIEW );
}


void THIS::resizeGL ( int w, int h )
{
    //   TRACE_ERROR( "resize" + Tracer::toString( w ) );
    glViewport ( 0, 0, w, h );
    m_ViewportWidth = w;
    m_ViewportHeight = h;
    m_ForceRedraw = true;
    updateGL();
}

void THIS::setBgColor ( float r, float g, float b, float a )
{
    glClearColor ( r, g, b, a );
}

void THIS::addPainter ( PainterPlugin *painter )
{
    if ( !painter )
    {
        //TRACE_ERROR( "Received 0-pointer!" );
        return;
    }
    m_Painters.push_back ( painter );
    m_PaintersMap[painter->getName()] = painter;
    painter->setParent( this );
}


// DISPLAY ROUTINES //////////////////////////////////////////////////////////////

void THIS::paintGL()
{
    //m_Timer.startMeasure();

    bool redraw = m_ForceRedraw || m_CameraTurning;

    //check if we need to re-draw
    list<PainterPlugin*>::iterator it = m_Painters.begin();
    while ( it != m_Painters.end() )
    {
        if ( ( *it )->needsRedraw() )
        {
            redraw = true;
            ( *it )->wasRedrawn();
        }
        it++;
    }

    if ( redraw && isVisible() )
    {
        setProjectionMatrix();

        glClear ( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );

        //     glMatrixMode ( GL_MODELVIEW );
        glLoadIdentity();

        if ( !m_CameraImageMode )
        {
            glTranslatef( 0,-m_CameraDistance*0.3,0 );
        }

        BaseLib::Math::Vec3d cameraOpposite( 1,0,0 ); // TODO

        //rotate around y and Z axis
        cameraOpposite = BaseLib::Math::rotationY<double,3>( m_CameraRotationY/180.0*M_PI ) * cameraOpposite;
        cameraOpposite = BaseLib::Math::rotationZ<double,3>( m_CameraRotationZ/180.0*M_PI ) * cameraOpposite;

        BaseLib::Math::Vec3d cameraPos = cameraOpposite;
        cameraPos *= m_CameraDistance * -1.0;

        //note: cameraOpposite is needed if m_CameraDistance==0.0

        gluLookAt ( cameraPos.x, cameraPos.y, cameraPos.z, // position of the eye
                    cameraOpposite.x, cameraOpposite.y, cameraOpposite.z, // eye position mirrored at 0,0
                    0.0, 0.0, 1.0 ); // Up-vector

        if ( m_CameraTurning )
        {
            float time = 1;//Clock::getInstance()->getTimestamp(); TODO
            //time for full turn (sec)
            float turnTime = 3.0;
            //t runs from 0 to 1 in <turnTime> seconds
            float t = ( time - m_CameraTurnStartTime ) / 1000.0 / turnTime;
            if ( t > 1.0 ) {
                m_CameraTurning = false;
               // TRACE_INFO( "360 degree camera move finished." );
            }
            else
            {
                //tSmooth also runs from 0 to 1, but with smooth acceleration and decceleration
                float tSmooth = -0.5*cos ( t*M_PI ) + 0.5;
                //calculate turn angle from t
                float deltaRad = tSmooth * 2.0 * M_PI;
                glRotatef( deltaRad/M_PI*180.0,0,0,1 );
                // scale the image to make the camera zoom
                float sSmooth = 1 - sin( t * M_PI ) / 2.0;
                glScalef( sSmooth, sSmooth, sSmooth);
            }
        }
        /*
    // draw coord system in start point
    glLineWidth ( 1.0 );
    glBegin ( GL_LINES );
    glColor3f ( 0.5, 0.0, 0.0 );
    glVertex3f ( 100.0, 0.0, 0.0 );
    glVertex3f ( 0.0, 0.0, 0.0 );
    glColor3f ( 0.0, 0.5, 0.0 );
    glVertex3f ( 0.0, 100.0, 0.0 );
    glVertex3f ( 0.0,   0.0, 0.0 );
    glColor3f ( 0.0, 0.0, 0.5 );
    glVertex3f ( 0.0, 0.0, 100.0 );
    glVertex3f ( 0.0, 0.0,   0.0 );
    glEnd();

*/
        // draw look at point
        glColor3f ( 0.5, 0.5, 0.5 );
        glLineWidth ( 3.0 );
        glBegin ( GL_LINES );
        glVertex3f ( 30.0,  30.0, 10.0 );
        glVertex3f ( -30.0, -30.0, 10.0 );
        glVertex3f ( -30.0,  30.0, 10.0 );
        glVertex3f ( 30.0, -30.0, 10.0 );
        glEnd();

        glTranslatef( m_LookAt.x, m_LookAt.y, 0 );

//        if ( !m_SceneGraph.empty() ) // TODO
//        {
//            BaseLib::Math::Mat4d worldToLookAtNode = m_SceneGraph.getTransformation( "World", m_LookAtNode );
//            //TRACE_INFO( worldToLookAtNode.niceString( 1, "worldToLookAtNode" ) );
//            double glMat[16];
//            worldToLookAtNode.toColumnMajor( glMat );
//            glMultMatrixd( glMat );
//        }

        //nextLayer: used to stack elements on floor in y-direction
        //to avoid z-buffer problems
        float nextLayer = 0.0;

        //     ostringstream s;
        //     s << "Paint order:" << endl;

        list< PainterPlugin*>::iterator it = m_Painters.begin();
        while ( it != m_Painters.end() )
        {
            if ( ( *it )->isVisible() )
            {
                glPushMatrix();
                ( *it )->paint ( nextLayer );
                glPopMatrix();
                //s << nextLayer << ": " << (*it)->getName() << endl;
                nextLayer += 5.0;
            }
            it++;
        }
        //TRACE_INFO( s.str() );

    }
    //m_Timer.submit();
}


void THIS::performCameraMove()
{
    if ( !m_CameraTurning )
    {
        m_CameraTurning = true;
        //m_CameraTurnStartTime = Clock::getInstance()->getTimestamp(); // TODO
        //make sure updateGL() is called in short intervals
        //TRACE_INFO( "Starting 360 degree camera move." );
    }
}


QSize THIS::minimumSizeHint() const
{
    return QSize ( 150, 120 );
}

QSize THIS::sizeHint() const
{
    return QSize ( 400, 300 );
}


///// EVENTS //////////////////////////////////////////////////////////


void THIS::mousePressEvent ( QMouseEvent* event )
{
    // save position
    m_MousePosOld = event->pos();
}


void THIS::mouseMoveEvent ( QMouseEvent* event )
{
    if ( m_CameraImageMode ) {
        return;
    }

    float deltaX = -m_MousePosOld.x() + event->x();
    float deltaY = -m_MousePosOld.y() + event->y();
    m_MousePosOld = event->pos();

    if ( event->buttons() & Qt::RightButton )
    {
        float rotateZ = deltaX/2.0;
        float rotateY = deltaY/2.0;

        m_CameraRotationZ -= rotateZ;
        m_CameraRotationY += rotateY;

        if ( m_CameraRotationZ >= 360 ) { m_CameraRotationZ -= 360; }
        if ( m_CameraRotationZ < 0 ) { m_CameraRotationZ += 360; }
        if ( m_CameraRotationY >= 89.9 ) { m_CameraRotationY = 89.9; }
        if ( m_CameraRotationY <= -89.9 ) { m_CameraRotationY = -89.9; }
        m_ForceRedraw = true;
    }
    else if ( event->buttons() & Qt::LeftButton )
    {
        BaseLib::Math::Vec3d translation(-deltaY,-deltaX,0);

        //make translation axis relative to current camera rotation
        translation = BaseLib::Math::rotationZ<double,3>( m_CameraRotationZ/180.0*M_PI ) * translation;
        //translate more if we're far away
        translation *= m_CameraDistance/500.0;

        m_LookAt += translation;
        m_ForceRedraw = true;
    }
}


void THIS::wheelEvent ( QWheelEvent* event )
{
    if ( ( event->delta() == 0 ) )
    {
        return;
    }

    if ( m_CameraImageMode )
    {
        if ( event->delta() > 0 ) // wheel goes up
        {
            m_CameraOpeningAngle += 5;
        }
        else // wheel goes down
        {
            m_CameraOpeningAngle -= 5;
        }
        if ( m_CameraOpeningAngle < 5.0 )
        {
            m_CameraOpeningAngle = 5.0;
        }
        if ( m_CameraOpeningAngle > 85.0 )
        {
            m_CameraOpeningAngle = 85.0;
        }
        m_ForceRedraw = true;
        updateGL();
    }
    else
    {
        if ( event->delta() > 0 ) // wheel goes up
        {
            m_CameraDistance = m_CameraDistance/1.05 - 10.0;
        }
        else // wheel goes down
        {
            m_CameraDistance = m_CameraDistance*1.05 + 10.0;
        }
        if ( m_CameraDistance < 0.0 )
        {
            m_CameraDistance = 0.0;
        }
        if ( m_CameraDistance > 20000.0 )
        {
            m_CameraDistance = 20000.0;
        }
        m_ForceRedraw = true;
    }
}

void THIS::setCameraRotation( float rotationY, float rotationZ )
{
    m_CameraRotationY = rotationY;
    m_CameraRotationZ = rotationZ;
    m_ForceRedraw = true;
}

void THIS::setLookAt( BaseLib::Math::Vec3d lookAt )
{
    m_LookAt = lookAt;
    m_ForceRedraw = true;
}

#undef THIS

