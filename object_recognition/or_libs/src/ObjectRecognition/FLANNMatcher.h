/*******************************************************************************
 *  FLANNMatcher.h
 *
 *  (C) 2007 AG Aktives Sehen <agas@uni-koblenz.de>
 *           Universitaet Koblenz-Landau
 *
 *  Additional information:
 *  $Id: $
 *******************************************************************************/

#ifndef FLANNMatcher_H
#define FLANNMatcher_H

#include <vector>
#include <deque>
#include <sstream>
#include <list>
#include <flann/flann.h>

#include <ros/ros.h>

#include "Workers/Math/Math.h"
#include "Workers/Math/Box2D.h"

#include "../KeyPointExtraction/KeyPoint.h"
#include "../KeyPointExtraction/KeyPointMatch.h"

/**
 * @class  FLANNMatcher
 * @brief  Matches keypoints by their feature vectors and geometric properties
 * @author David Gossow (RX)
 */
class FLANNMatcher
{
  public:

    /**
     * @brief constructor
     * @param[in] reference list of keypoints, an index list for this set is created
     */ 
    FLANNMatcher();

    /** @brief Assignment operator */

    FLANNMatcher( const FLANNMatcher& other );
    FLANNMatcher& operator=( const FLANNMatcher& other );


    /** @brief The destructor */
    ~FLANNMatcher();
   
    /**
     * Create a new index
     */

    void createIndex( std::vector< KeyPoint >* keyPoints );

    /**
     * KeyPointMatch features by distance ratio strategy
     * @param keyPoints[in] keypoints which will be matched against the indexed set
     * @param maxDistRatio Maximal ratio between closest and second-closest match
     * @param symmetric if true, only keep matches that a
     */
    void match( std::vector< KeyPoint >* keyPoints, float maxDistRatio=0.7 );

    std::list< KeyPointMatch > getMatches() { return m_Matches; }

    bool hasIndex(){return m_hasIndex;}

    /** @return number of remaining matches */
    int getNumMatches() { return m_Matches.size(); }

//    int getNumIndexedKeypoints(){return m_Matches.size();}

    FLANNParameters& getFlannParameters(){
       return m_flannParams;
     }

    std::string getLog();

  private:

    /** @brief If more than one keypoint of First have been matched with the same of Second, keep only closest */
    void eliminateMultipleMatches( );
    void clearFLANNMembers();
    void fillFlannDataWithDescriptors(const std::vector<KeyPoint>* features, float* flannDataPtr);

    //stores a list of matches
    typedef std::list< KeyPointMatch > MatchList;

    //iterator for accessing and deleting elements of the match list
    typedef std::list<KeyPointMatch>::iterator MatchElem;

    MatchList m_Matches;

    std::ostringstream m_Log;
   
    FLANN_INDEX m_flannIndex;
    FLANNParameters m_flannParams;
    bool m_hasIndex;
    unsigned int m_descriptorLength;

    float* m_FlannModelData;

};

#endif
