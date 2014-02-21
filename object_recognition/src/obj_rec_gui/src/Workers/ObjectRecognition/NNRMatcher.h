/*******************************************************************************
 *  NNRMatcher.h
 *
 *  (C) 2007 AG Aktives Sehen <agas@uni-koblenz.de>
 *           Universitaet Koblenz-Landau
 *
 *  Additional information:
 *  $Id: $
 *******************************************************************************/

#ifndef NNRMatcher_H
#define NNRMatcher_H


#include <vector>
#include <deque>
#include <sstream>
#include <list>

#include "Workers/Math/Math.h"
#include "Workers/Math/Box2D.h"
#include "Workers/KeyPointExtraction/KeyPoint.h"
#include "Workers/KeyPointExtraction/KeyPointMatch.h"

/**
 * @class  NNRMatcher
 * @brief  Matches keypoints by their feature vectors and geometric properties
 * @author David Gossow (RX)
 */
class NNRMatcher
{
  public:

    /** @brief The constructor
     *  @param first,second Lists of keypoints to match with each other
    */
    NNRMatcher( std::vector< KeyPoint >* keyPointsA, std::vector< KeyPoint >* keyPointsB );

    /** @brief The destructor */
    ~NNRMatcher();

    /**
     * KeyPointMatch features by distance ratio strategy
     * @param maxDistRatio Maximal ratio between closest and second-closest match
     * @param symmetric if true, only keep matches that a
     */
    void match( float maxDistRatio=0.7 );

    std::list< KeyPointMatch > getMatches() { return m_Matches; }

    /** @return number of remaining matches */
    int getNumMatches() { return m_Matches.size(); }

    /** @return Remaining matched keypoints */
    std::vector< std::pair< KeyPoint, KeyPoint > > getMatchedKeyPoints();

    std::string getLog();

  private:

    /** @brief If more than one keypoint of First have been matched with the same of Second, keep only closest */
    void eliminateMultipleMatches( );

    std::vector< KeyPoint >* m_KeyPointsA;
    std::vector< KeyPoint >* m_KeyPointsB;

    //stores a list of matches
    typedef std::list< KeyPointMatch > MatchList;

    //iterator for accessing and deleting elements of the match list
    typedef std::list<KeyPointMatch>::iterator MatchElem;

    MatchList m_Matches;

    std::ostringstream m_Log;
};

#endif
