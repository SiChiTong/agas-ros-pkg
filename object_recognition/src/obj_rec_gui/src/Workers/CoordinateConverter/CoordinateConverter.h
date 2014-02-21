/*******************************************************************************
 *  CoordinateConverter.h
 *
 *  (C) 2006 AG Aktives Sehen <agas@uni-koblenz.de>
 *           Universitaet Koblenz-Landau
 *
 *  $Id: CoordinateConverter.h 44313 2011-04-06 22:46:28Z agas $
 ******************************************************************************/

#ifndef COORDINATECONVERTER_H
#define COORDINATECONVERTER_H

#include <vector>

#include "Workers/Math/Point2D.h"
#include "Workers/Math/Vector3D.h"
#include "Workers/Math/Pixel.h"
#include "Workers/Math/Pose.h"
#include "Workers/Math/Transformation2D.h"

#define M_PI 3.14159265358979323846

/**
 * @brief Class for converting 2D points between various coordinate systems
 */
class CoordinateConverter {

  public:

    /**
     * Destructor
     */
    virtual ~CoordinateConverter();


    /**
     * From robot to all others
     */
    static Point2D robotToWorld(Pose robotPose, Point2D robotPoint);
    static Pose robotToWorld( Pose robotPose, Pose pose);
    static Transformation2D robotToWorld(Pose robotPose, Transformation2D transformation);
    static std::vector<Point2D> robotToWorld(Pose robotPose, const std::vector<Point2D>& robotPoints);

    /**
     * From world to all others
     */

    static Point2D worldToRobot(Pose robotPose, Point2D worldPoint);
    static Transformation2D worldToRobot(Pose robotPose, Transformation2D transformation);

    static Point2D polarToCartesian(float theta, float distance);

    /**
     * From spherical to cartesian
     * @param range the distance from the origin to a given point P
     * @param phi the angle between the positive z-axis and the line formed
     *            between the origin and the point P
     * @param theta the angle between the positive x-axis and the line from
     *            the origin to the point P projected onto the xy-plane
     */
    static Vector3D sphericalToCartesian(float range, float phi, float theta);

    /**
     * From spherical plane slice to world coordinate
     * @param range the distance from the LRF-origin to a measured point P
     * @param h the distance from the LRF-origin to the rotation axis of the servo-motor
     * @param theta the angle between the positive z-axis and the line g between the rotation axis and the measured point P
     * @param phi the angle between the positive x-axis and the projected line g onto the xy-plane
     */
    static Vector3D sphericalPlaneToWorldCoordRad(float range, float h, float theta_rad, float phi_rad);
    static Vector3D sphericalPlaneToWorldCoordDeg(float range, float h, float theta_deg, float phi_deg){
      return sphericalPlaneToWorldCoordRad(range, h, (theta_deg*M_PI/180.0), (phi_deg*M_PI/180.0));
    }

    /**
     * From simLaser3D (spheric) to laser3D (cartesian)
     */
    static Vector3D simLaser3DToLaser3D(float alpha, float beta, float range);

    /**
     * Convert simLaser3D points given by a vector of ranges, horizontal and
     * vertical FOV and resolution
     */
    static std::vector<Vector3D> simLaser3DToLaser3D(float fovH, float fovV, float resH, float resH2, std::vector<float>* range);



    /// @brief ************ Deprecated. DO NOT USE! *****************
    static CoordinateConverter* getInstance();

    void robotToMap(float robotX, float robotY, float robotTheta, float pointX, float pointY, int& mapX, int& mapY);
    Pixel robotToMap(Pose robotPose, Point2D robotPoint);
    std::vector<Pixel> robotToMap(Pose robotPose, const std::vector<Point2D>& robotPoints);
    Pixel worldToMap(Point2D worldPoint);
    Point2D worldToMapPoint(Point2D worldPoint);
    Pixel worldToMap(Pose worldPose);
    void worldToMap(float worldX, float worldY, int& mapX, int& mapY);
    std::vector<Pixel> worldToMap( const std::vector<Point2D>& worldPoints);
    Point2D mapToWorld(Pixel mapPoint);
    std::vector<Point2D> mapToWorld( const std::vector<Pixel>& mapPoints );

  private:

    /**
     * Private default Constructor (-->Singleton)
     */
    CoordinateConverter();

    /**
     * Single instance of the singleton ;-)
     */
    static CoordinateConverter* m_Instance;


    float m_CellSize;
    float m_MapWidth;
    float m_MapHeight;
    float m_MapHalfWidth;
    float m_MapHalfHeight;

};

#endif
