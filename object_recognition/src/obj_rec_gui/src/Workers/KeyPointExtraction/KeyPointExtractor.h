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

#include "../../Workers/Puma2/GrayLevelImage8.h"
#include "../../Workers/Puma2/ColorImageRGB8.h"

#include "Workers/Math/Box2D.h"
#include "Workers/Math/Point2D.h"
#include "../../Workers/Puma2/ImageMask.h"

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
    virtual void setImage ( const puma2::ColorImageRGB8 &pumaImage ) = 0;

    /// @brief Copy the given image for later feature extraction
    virtual void setImage ( const puma2::GrayLevelImage8 &pumaImage ) = 0;

    /// @brief detect keypoints and assign orientation and descriptor
    virtual void getKeyPoints ( std::vector< KeyPoint >& keyPoints ) = 0;
    
    virtual std::string getName() = 0;

    /** @return list of all parameters and values */
    virtual std::string getDescription() = 0;
};


#endif

