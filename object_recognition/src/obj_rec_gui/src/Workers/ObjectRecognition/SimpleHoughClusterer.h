/*******************************************************************************
 *  SimpleHoughClusterer.h
 *
 *  (C) 2007 AG Aktives Sehen <agas@uni-koblenz.de>
 *           Universitaet Koblenz-Landau
 *
 *  Additional information:
 *  $Id: $
 *******************************************************************************/

#ifndef SimpleHoughClusterer_H
#define SimpleHoughClusterer_H

#include <vector>
#include <deque>
#include <sstream>
#include <list>

#include "Workers/KeyPointExtraction/KeyPoint.h"
#include "Workers/KeyPointExtraction/KeyPointMatch.h"

#include "Workers/Math/Math.h"
#include "Workers/Math/Box2D.h"

/**
 * @class  SimpleHoughClusterer
 * @brief  Matches keypoints by their feature vectors and geometric properties
 * @author David Gossow (RX)
 */
class SimpleHoughClusterer
{
  public:

    /** @brief The constructor
    */
    SimpleHoughClusterer(  std::vector< KeyPoint >* keyPoints1,  std::vector< KeyPoint >* keyPoints2, std::list< KeyPointMatch >& matches );

    /** @brief The destructor */
    ~SimpleHoughClusterer();

    /** @brief Create an angle histogram and remove nonfitting matches
     *  @param threshold maximal angle difference to mean angle
    */
    void eliminateByOrientation( );

    /** @brief Create a scale quotient histogram and remove nonfitting matches
     *  @param threshold maximal difference to mean relative scale
     */
    void eliminateByScale( );

    /** @brief Translates all points into the same coordinate system and removes matches with a too high distance
     *  @param maxDistance maximal distance between original and translated match (multiplied by the ipoint scale)
     *  @todo implement this using a histogram
     */
    void eliminateByPosition( float maxDistance );

    /// @return remaining matches
    std::list< KeyPointMatch > getMatches() { return m_Matches; }

    std::string getLog();

  private:

    //detect maxima & return a map of which bins need to be deleted
    std::vector<bool> computeDeleteMap( std::vector<double> hist, int numBins, int windowSize );

    void eraseMatches( std::list<unsigned>& indices );

    float getMeanTurnAngle() const;
    float getMeanScaleQuotient() const;

    void getCenters ( Point2D& centerA, Point2D& centerB );

    inline int deg( float rad ) { return int( rad/Math::Pi*180.0 ); }

    //scenePoints
    std::vector< KeyPoint >* m_KeyPoints1;
    ///objectImagePoints
    std::vector< KeyPoint >* m_KeyPoints2;

    std::list< KeyPointMatch > m_Matches;

    std::ostringstream m_Log;

};

#endif
