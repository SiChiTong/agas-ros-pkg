/*******************************************************************************
 *  SurfExtractorBase.h
 *
 *  (C) 2008 AG Aktives Sehen <agas@uni-koblenz.de>
 *           Universitaet Koblenz-Landau
 *
 *******************************************************************************/

#ifndef SurfExtractorBase_H
#define SurfExtractorBase_H

#include <vector>
#include <string>
#include "../../Workers/Puma2/GrayLevelImage8.h"

#include "Workers/Math/Box2D.h"
#include "Workers/Math/Point2D.h"
#include "../../Workers/Puma2/ImageMask.h"

//#include "Architecture/Serializer/ExtendedOutStream.h" // TODO get rid
//#include "Architecture/Serializer/ExtendedInStream.h" // TODO get rid

#include "KeyPoint.h"
#include "KeyPointExtractor.h"


/**
 * @class  SurfExtractorBase
 * @author David Gossow (RX)
 * @brief  Base class for keypoint extractors based on SURF features
 */
class SurfExtractorBase: public KeyPointExtractor
{
  public:

    /** @brief Default constructor. Reads the SURF parameters from the configuration file. */
    SurfExtractorBase( );

    /** @brief Copy constructor */
    SurfExtractorBase( const SurfExtractorBase& other );

    /** @brief The destructor */
    virtual ~SurfExtractorBase();

    /** @brief Assignment operator */
    SurfExtractorBase& operator=( const SurfExtractorBase& other );

    /**
     * @brief Change parameters for SURF algorithm.
     */
    void setSamplingStep( int newValue );
    void setOctaves( int newValue );
    void setBlobResponseThreshold( double newValue );
    void setInitLobeSize( int newValue );
    void setRotationInvariance( bool newValue );
    void setExtended( bool newValue );

    /** @return list of all parameters and values */
    virtual std::string getDescription();

  protected:

    // Parameters for the SURF algorithm

    /** @brief Square size of descriptor window (determines feature vector length) */
    int m_IndexSize;
    /** @brief Initial sampling step */
    int m_SamplingStep;
    /** @brief Number of analyzed octaves */
    int m_Octaves;
    /** @brief Blob response treshold */
    double m_BlobResponseThreshold;
    /** @brief Initial lobe size */
    int m_InitLobeSize;
    /** @brief Upright SURF or rotation invariant */
    bool m_RotationInvariance;
    /** @brief If the extended flag is turned on, SURF 128 is used */
    bool m_Extended;
};


#endif

