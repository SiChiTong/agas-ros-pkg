/*******************************************************************************
 *  CvHomography.h
 *
 *  (C) 2007 AG Aktives Sehen <agas@uni-koblenz.de>
 *           Universitaet Koblenz-Landau
 *******************************************************************************/

#ifndef CvHomography_H
#define CvHomography_H

#include <vector>
#include <list>
#include <ros/ros.h>
#include "../KeyPointExtraction/KeyPointMatch.h"
#include "../KeyPointExtraction/KeyPoint.h"
#include "Workers/Math/Homography.h"

/**
 * @class  CvHomography
 * @brief  Find a homography between matched keypoints using opencv
 * @author David Gossow
 */
class CvHomography
{
  public:

    /** @brief Finds the homography from keyPoints2 to keyPoints1 */
    CvHomography( std::vector< KeyPoint >* keyPoints1, std::vector< KeyPoint >* keyPoints2, std::list< KeyPointMatch >& matches );

    /** @brief The destructor */
    ~CvHomography();

    bool computeHomography();

    void eliminateBadMatches();

    /// @return remaining matches
    std::list< KeyPointMatch > getMatches() { return m_Matches; }

    Homography getHomography() { return m_Homography; }

  private:

    Homography m_Homography;

    std::vector< KeyPoint >* m_KeyPoints1;
    std::vector< KeyPoint >* m_KeyPoints2;

    std::list< KeyPointMatch > m_Matches;

    bool m_Success;

    int m_MaxReprojectionError;
};

#endif
