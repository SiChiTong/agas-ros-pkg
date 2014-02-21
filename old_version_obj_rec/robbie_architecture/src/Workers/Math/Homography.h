/*******************************************************************************
 *  CvHomography.h
 *
 *  (C) 2007 AG Aktives Sehen <agas@uni-koblenz.de>
 *           Universitaet Koblenz-Landau
 *******************************************************************************/

#ifndef Homography_H
#define Homography_H

#include "Point2D.h"
#include <vector>

/**
 * @class  Homography
 * @brief  Represents a homography
 * @author David Gossow
 */
class Homography
{
  public:

    Homography( ) {}

    Homography( double homMat[9] );

    Homography( const Homography& other );

    Homography& operator=( const Homography& other );

    /** Transform point2 using the homography */
    Point2D transform ( Point2D point2 );

    /** Transform keyPoints2 using the homography and store them in projPoints
     * @return if one of the resulting points has z<0, that means it would lie behind the camera, return false
     */
    void transform ( std::vector<Point2D>& points2, std::vector<Point2D> &projPoints );

    /// @return true if all the given points lie in front of the camera (z>0)
    bool checkValidity ( std::vector<Point2D>& points2 );

    std::string toString();

 // private: // FIXME made public to create ROS message

    double m_HomMat[9];
};

#endif
