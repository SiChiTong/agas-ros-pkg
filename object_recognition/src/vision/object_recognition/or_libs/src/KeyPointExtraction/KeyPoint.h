/*******************************************************************************
 *  KeyPoint.h
 *
 *  (C) 2007 AG Aktives Sehen <agas@uni-koblenz.de>
 *           Universitaet Koblenz-Landau
 *
 *  Additional information:
 *  $Id: $
 *******************************************************************************/

#ifndef KeyPoint_H
#define KeyPoint_H

#include <vector>
#include "Workers/Math/Point2D.h"


/**
 * @class  KeyPoint
 * @brief  Describes features of an image region
 * @author David Gossow (RX)
 */
class KeyPoint
{
  public:

    /**
     * @brief The constructor
     * @param x,y position within the image
     * @param scale describes the dimension of the feature, the actual pixel size depends on which feature extractor is used
     * @param strength strength (e.g. contrast) of the feature
     * @param orientation main direction of the feature
     * @param sign leading sign of the feature (e.g. sign of laplacian provided by SURF)
     * @param featureVector feature data
     */
    KeyPoint( float _x, float _y, float _scale, float _strength, int _sign, float _orientation, std::vector<double> _featureVector );
    KeyPoint( float _x, float _y, float _scale, float _strength, int _sign );
    KeyPoint( );

    /** @brief The destructor */
    ~KeyPoint();

    KeyPoint( const KeyPoint& other );

    KeyPoint& operator=( const KeyPoint& other );

    /** @brief allocate descriptor memory and copy from given source */
    template< class T >
    void copyDescriptor( T* descriptor, int length );

    void addDescriptor( std::vector<double> descriptor );

    Point2D position() { return Point2D( x, y ); }

    /** @return squared euclidean distance */
    double squaredDistance( const KeyPoint& other ) const;

    /**
    @brief speed-optimized partial calculation of descriptor distance
           calculates the squared euclidean distance, but cancels the calculation when a given minumum value is reached
    @return squared euclidean distance or something larger than max */
    double squaredDistance( const KeyPoint& other, double max ) const;

    /** @return vertices of circle representing detection scale */
    std::vector<Point2D> getCircle() const;

    /** @return vertices of bounding box */
    std::vector<Point2D> getBoundingBox() const;

    /** @return vertices of arrow pointing to up direction */
    std::vector<Point2D> getCenterArrow() const;

    // SERIALIZATION

    /** @return ASCII string as used in Mikolaicyk's test environment */
    std::string toASCII();

    /** @return string used to store keypoint for evaluation */
    std::string toString();

    /** @return intersection of area with other keypoint */
    double calcIntersection( const KeyPoint& other );

    /** @return quotient of intersection and union of the keypoint areas */
    double calcOverlap( const KeyPoint& other );

    /** @brief properties
     * @see constructor */
    float x;
    float y;

    float scale;
    float strength;
    float orientation;
    int sign;

    std::vector<double> featureVector;
    std::vector<unsigned> vectorLimits;
};


template< class T >
void KeyPoint::copyDescriptor( T* descriptor, int length )
{
  featureVector.resize(length);
  for ( int i = 0; i<length; i++ )
  {
    featureVector[i] = descriptor[i];
  }

}



#endif
