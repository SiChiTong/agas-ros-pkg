/*******************************************************************************
 *  KeyPointHelper.h
 *
 *  (C) 2007 AG Aktives Sehen <agas@uni-koblenz.de>
 *           Universitaet Koblenz-Landau
 *******************************************************************************/

#ifndef KeyPointHelper_H
#define KeyPointHelper_H

#include "KeyPoint.h"

#include "Workers/Math/Box2D.h"
//#include "Workers/ImageHelpers/ImageMaskCV.h"
#include "ImageMaskCV.h"

#include <vector>

/**
 * @class  KeyPointHelper
 * @brief  Useful functions related to keypoint extraction
 * @author David Gossow
 */
class KeyPointHelper
{

  public:

  /**  @brief only keep keypoints inside the given bounding box
    *  @param keyPointsIn input keypoints [in]
    *  @param keyPointsOut filtered keypoints [out]
  */
  static void bBoxFilter( std::vector< KeyPoint > keyPointsIn, std::vector< KeyPoint >& keyPointsOut,
               Box2D<> boundingBox );

  /** @brief  only keep keypoints inside the given bounding mask
    *  @param keyPointsIn input keypoints [in]
    *  @param keyPointsOut filtered keypoints [out]
    */
  static void maskFilter( std::vector< KeyPoint > keyPointsIn, std::vector< KeyPoint >& keyPointsOut,
                   ImageMaskCV &mask );

  static void getStrongest( std::vector< KeyPoint > keyPointsIn, std::vector< KeyPoint >& keyPointsOut, unsigned numKeyPoints );

  static void sortByStrength( std::vector< KeyPoint > keyPointsIn, std::vector< KeyPoint >& keyPointsOut );

  /** @brief only keep keypoints which don't have a descriptor area that goes beyond the image border (needed for dlib surf) */
  static void imageBorderFilter( std::vector< KeyPoint > keyPointsIn, std::vector< KeyPoint >& keyPointsOut, int imgWidth, int imgHeight );

};

#endif
