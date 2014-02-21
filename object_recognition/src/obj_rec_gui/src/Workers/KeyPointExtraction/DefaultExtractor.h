/*******************************************************************************
 *  DefaultExtractor.h
 *
 *  (C) 2008 AG Aktives Sehen <agas@uni-koblenz.de>
 *           Universitaet Koblenz-Landau
 *
 *******************************************************************************/

#ifndef DefaultExtractor_H
#define DefaultExtractor_H

#include "KeyPointExtractor.h"

/**
 * @class  DefaultExtractor
 * @author David Gossow
 * @brief  Extracts keypoints from an image using the default method set in the config
 */
class DefaultExtractor
{
  public:

    enum ExtractorType {
      ExtParallelSurf=0,
      ExtOrigSurf=1
    };

    /// @brief create new instance of the default keypoint extractor. The caller takes ownership of the pointer.
    static KeyPointExtractor* createInstance();

};


#endif

