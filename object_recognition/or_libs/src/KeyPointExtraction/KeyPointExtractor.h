/*******************************************************************************
 *  KeyPointExtractor.h
 *
 *  (C) 2008 AG Aktives Sehen <agas@uni-koblenz.de>
 *           Universitaet Koblenz-Landau
 *
 *******************************************************************************/

#ifndef KeyPointExtractor_H
#define KeyPointExtractor_H

#include <vector>
#include <string>

#include <opencv2/opencv.hpp>

#include "Workers/Math/Box2D.h"
#include "Workers/Math/Point2D.h"

#include "KeyPoint.h"

/**
 * @class  KeyPointExtractor
 * @author David Gossow (RX)
 * @brief  Abstract class for extracting keypoints and their feature vectors from images
 */
class KeyPointExtractor
{
  public:

    KeyPointExtractor( ) {};

    virtual ~KeyPointExtractor( ) {};

    /// @brief Copy the given image for later feature extraction
    virtual void setImage ( const cv::Mat &image ) = 0;

    /// @brief detect keypoints and assign orientation and descriptor
    virtual void getKeyPoints ( std::vector< KeyPoint >& keyPoints ) = 0;
    
    virtual std::string getName() = 0;

    /** @return list of all parameters and values */
    virtual std::string getDescription() = 0;
};


#endif

