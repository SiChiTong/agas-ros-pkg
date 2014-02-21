/*******************************************************************************
 *  LaserScan3DPainter.cpp
 *
 *  (C) 2007 AG Aktives Sehen <agas@uni-koblenz.de>
 *           Universitaet Koblenz-Landau
 *
 *  Additional information:
 *  $Id: $
 *******************************************************************************/

#include "LaserScan3DPainter.h"

#include "Workers/Math/Math.h"

#include "Messages/PointCloudM.h"
#include "Messages/SceneGraphM.h"

#include <QtOpenGL>
#include <GL/glut.h>

#define THIS LaserScan3DPainter

THIS::THIS() : PainterPlugin( )
{
    setName ( "3D Debug Data" );
    m_RawDataCounter = 0;
}


THIS::~THIS()
{
}


void THIS::processMessage ( Message* newMessage )
{
    PainterPlugin::processMessage ( newMessage );
    switch ( newMessage->getType() )
    {
    case MessageTypes::DEBUG_3D_M:
        {
            Debug3DM* message = Message::castTo<Debug3DM> ( newMessage );
            if ( message )
            {
                m_VertexSets[message->getChannelId()] = message->getVertexSets();
                m_TransformationMatrices[message->getChannelId()] = message->getTransformationMatrix();
                requestRedraw();
            }
            break;
        }

    case MessageTypes::SCENE_GRAPH_M:
        {
            SceneGraphM* message = Message::castTo<SceneGraphM> ( newMessage );
            if ( message )
            {
                m_RobotToWorld = message->getSceneGraph().getTransformation( "Robot", "World" );
            }
            break;
        }

    case MessageTypes::POINT_CLOUD_M:
        {
            PointCloudM* message = Message::castTo<PointCloudM> ( newMessage );
            if ( message )
            {
                // 				TRACE_INFO( "PointCloudM #id: " << message->getID() << " #points: " << message->getPoints().size() );
                Debug3DM::VertexSet vs;
                vs.r = 1;
                vs.g = 1;
                vs.b = 1;
                vs.paintStyle = Debug3DM::SmallDots;
                vs.points = message->getPoints();
                string id = "Raw PointCloudM Data" + Tracer::toString( m_RawDataCounter );
                // 				m_RawDataCounter++;
                m_VertexSets[id] = vector<Debug3DM::VertexSet>();
                m_VertexSets[id].push_back( vs );
                m_TransformationMatrices[id] = m_RobotToWorld;
                // 				TRACE_INFO( m_TransformationMatrices[id].niceString(2,"m_RobotToWorld") );
                requestRedraw();
            }
            break;
        }

    default:
        break;

    }
}

void THIS::paint ( float next2DLayer )
{

    std::map< std::string, std::vector<Debug3DM::VertexSet> >::iterator vertexSetIt;

    for ( vertexSetIt = m_VertexSets.begin(); vertexSetIt != m_VertexSets.end(); vertexSetIt++ )
    {
        glPushMatrix();

        GLdouble glMatrix[16];
        m_TransformationMatrices[ vertexSetIt->first ].toColumnMajor ( glMatrix );
        glMultMatrixd ( glMatrix );

        std::vector<Debug3DM::VertexSet>& vertexSet = vertexSetIt->second;
        for ( unsigned i=0; i<vertexSet.size(); i++ )
        {
            float r = vertexSet[i].r;
            float g = vertexSet[i].g;
            float b = vertexSet[i].b;


            switch ( vertexSet[i].paintStyle )
            {
            case Debug3DM::Dots:
                glPointSize ( 3.0 );
                glBegin( GL_POINTS );
                break;

            case Debug3DM::SmallDots:
                glPointSize ( 1.0 );
                glBegin( GL_POINTS );
                break;

            case Debug3DM::LineStrip:
                glPointSize ( 3.0 );
                glBegin( GL_LINE_STRIP );
                break;

            case Debug3DM::LineLoop:
                glPointSize ( 3.0 );
                glBegin( GL_LINE_LOOP );
                break;

            case Debug3DM::DotFlowers:
                glPointSize ( 3.0 );
                glBegin( GL_POINTS );
                break;

            case Debug3DM::Quads:
                glBegin( GL_QUADS );
                break;
            }

            for ( unsigned j=0; j<vertexSet[i].points.size(); j++ )
            {
                glColor3f( r,g,b );
                glVertex3f( vertexSet[i].points[j].x, vertexSet[i].points[j].y, vertexSet[i].points[j].z );
            }
            glEnd();

            switch ( vertexSet[i].paintStyle )
            {
            case Debug3DM::DotFlowers:
                glLineWidth ( 1.0 );
                glBegin( GL_LINES );
                for ( unsigned j=0; j<vertexSet[i].points.size(); j++ )
                {
                    glColor3f( 0.5,0.5,0.5 );
                    glVertex3f( vertexSet[i].points[j].x, vertexSet[i].points[j].y, 0 );
                    glVertex3f( vertexSet[i].points[j].x, vertexSet[i].points[j].y, vertexSet[i].points[j].z );
                }
                glEnd();
                break;

        default:
                break;
            }
        }
        glPopMatrix();
    }

}

#undef THIS
