/*******************************************************************************
 *  OrigSurfExtractor.h
 *
 *  (C) 2008 AG Aktives Sehen <agas@uni-koblenz.de>
 *           Universitaet Koblenz-Landau
 *
 *******************************************************************************/

#ifndef OrigSurfExtractor_H
#define OrigSurfExtractor_H

#include <vector>
#include <string>


#include "Workers/Math/Box2D.h"
#include "Workers/Math/Point2D.h"

#include "Architecture/Serializer/ExtendedOutStream.h"
#include "Architecture/Serializer/ExtendedInStream.h"

#include "Workers/Puma2/ImageMask.h"
#include "Workers/Puma2/GrayLevelImage8.h"

#include "KeyPoint.h"
#include "SurfExtractorBase.h"


#include <surf/ipoint.h>
#include <surf/image.h>
#include <surf/surf.h>


/**
 * @class  OrigSurfExtractor
 * @author David Gossow (RX)
 * @brief  Extracts keypoints and their SURF features from an image
 */
class OrigSurfExtractor: public SurfExtractorBase
{
  public:

    /** @brief Default constructor. Reads the SURF parameters from the configuration file. */
    OrigSurfExtractor( );

    /** @brief Copy constructor */
    OrigSurfExtractor( const OrigSurfExtractor& other );

    /** @brief The destructor */
    virtual ~OrigSurfExtractor();

    /** @brief Assignment operator */
    OrigSurfExtractor& operator=( const OrigSurfExtractor& other );

    /** @brief Copy the given image for later feature extraction */
    virtual void setImage( const puma2::GrayLevelImage8 &pumaImage );

    /** @brief Copy the given image for later feature extraction */
    virtual void setImage( const puma2::ColorImageRGB8 &pumaImage );

    /// @brief detect keypoints and assign orientation and descriptor
    virtual void getKeyPoints ( std::vector< KeyPoint >& keyPoints );
    
    virtual std::string getName();

  private:

    surf::Image* m_IntegralImage;
};


#endif

