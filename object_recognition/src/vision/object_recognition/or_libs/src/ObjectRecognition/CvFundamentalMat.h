/*******************************************************************************
 *  CvFundamentalMat.h
 *
 *  (C) 2007 AG Aktives Sehen <agas@uni-koblenz.de>
 *           Universitaet Koblenz-Landau
 *******************************************************************************/

#ifndef CvFundamentalMat_H
#define CvFundamentalMat_H

#include <vector>
#include <list>
#include <ros/ros.h>
#include "../KeyPointExtraction/KeyPointMatch.h"
#include "../KeyPointExtraction/KeyPoint.h"
// #include "Workers/Math/Homography.h"
#include <cv.h>

/**
 * @class  CvFundamentalMat
 * @brief  Find a fundamental matrix between matched keypoints using opencv
 * @author Susanne Thierfelder
 */
class CvFundamentalMat
{
  public:

    /** @brief Finds the homography from keyPoints2 to keyPoints1 */
    CvFundamentalMat( std::vector< KeyPoint >* keyPoints1, std::vector< KeyPoint >* keyPoints2, std::list< KeyPointMatch >& matches );

    /** @brief The destructor */
    ~CvFundamentalMat();

    bool computeFundamentalMat();

    /// @return remaining matches
    std::list< KeyPointMatch > getMatches() { return m_Matches; }

    void eliminateBadMatches();

  private:

    std::vector< KeyPoint >* m_KeyPoints1;
    std::vector< KeyPoint >* m_KeyPoints2;

    std::list< KeyPointMatch > m_Matches;

    int m_Success;

    int m_MaxReprojectionError;

    CvMat m_FundMatCv;

    CvMat m_Points1CvMat;
    CvMat m_Points2CvMat;
};

#endif
