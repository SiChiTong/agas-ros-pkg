/*******************************************************************************
 *  SurfExtractor.h
 *
 *  (C) 2008 AG Aktives Sehen <agas@uni-koblenz.de>
 *           Universitaet Koblenz-Landau
 *
 *******************************************************************************/

#ifndef SurfExtractor_H
#define SurfExtractor_H

#include <vector>
#include <string>


#include "Workers/Math/Box2D.h"
#include "Workers/Math/Point2D.h"

#include "KeyPoint.h"
#include "SurfExtractorBase.h"

#include "../ParallelSurf/KeyPointDetector.h"

#include "threadpool.hpp"



/**
 * @class  SurfExtractor
 * @author David Gossow (RX)
 * @brief  Extracts keypoints and their SURF features from an image
 */
class ParallelSurfExtractor: public SurfExtractorBase
{
  public:

    /** @brief Default constructor. Reads the SURF parameters from the configuration file. */
   ParallelSurfExtractor( int numThreads=0 );

    /** @brief Copy constructor */
   ParallelSurfExtractor( const ParallelSurfExtractor& other );

    /** @brief The destructor */
    virtual ~ParallelSurfExtractor();

    /** @brief Assignment operator */
    ParallelSurfExtractor& operator=( const ParallelSurfExtractor& other );

    /** @brief Copy the given image for later feature extraction */
    virtual void setImage( const cv::Mat &image );

    /// @brief detect keypoints and assign orientation and descriptor
    virtual void getKeyPoints ( std::vector< KeyPoint >& keyPoints );
    
    virtual std::string getName();

  private:

    parallelsurf::Image* m_IntegralImage;
    
    boost::threadpool::pool *m_ThreadPool;

    // define a Keypoint insertor
    class KeyPointVectInsertor : public parallelsurf::KeyPointInsertor
    {
      public:

        KeyPointVectInsertor ( std::vector<parallelsurf::KeyPoint>& keyPoints ) : m_KeyPoints ( keyPoints ) {};

        inline virtual void operator() ( const parallelsurf::KeyPoint &panoKeyPoint )
        {
//           std::cout << ".";
          m_KeyPoints.push_back ( panoKeyPoint );
        }

      private:

        std::vector<parallelsurf::KeyPoint>& m_KeyPoints;
    };
};


#endif

