/*******************************************************************************
 *  MatchResult.h
 *
 *  (C) 2007 AG Aktives Sehen <agas@uni-koblenz.de>
 *           Universitaet Koblenz-Landau
 *******************************************************************************/

#ifndef MatchResult_H
#define MatchResult_H

#include "Workers/KeyPointExtraction/KeyPoint.h"
#include "Workers/KeyPointExtraction/KeyPointMatch.h"

#include "Workers/Math/Point2D.h"
#include "Workers/Math/Homography.h"

#include "Workers/Puma2/ColorImageRGB8.h"

/**
 * @class  MatchResult
 * @brief  Container struct describing result of matching one object
 * @author David Gossow
 */
struct MatchResult
{
  /** @brief object parameters */
  std::string objectName;
  std::string objectType;

  /** @brief which object image has been matched */
  puma2::ColorImageRGB8* image;
  int imageIndex;
  std::string imageName;
  std::vector<Point2D> outline;
  std::vector<Point2D> bBox;
  Point2D center;

	/** @brief index of area of interest (bounding box) where the object was found. */
	int boundingBoxIndex;
	/** @brief maps indices of keypoints from the bounding box (as in stage[123]Matches) to scene keypoints */
	std::vector< unsigned > keyPointIndexMap;

  /** @brief keypoints belonging to the object image */
  std::vector< KeyPoint > objectKeyPoints;

  /** @brief keypoint correspondences after different matching stages */
  std::list< KeyPointMatch > stage1Matches;
  std::vector< std::list< KeyPointMatch> > stage2Matches;
  std::list< KeyPointMatch > stage3Matches;

  std::vector< KeyPoint > sceneKeyPointsWithinOutline;

  /** @brief homography from object image to camera image */
  Homography homography;
};

#endif
