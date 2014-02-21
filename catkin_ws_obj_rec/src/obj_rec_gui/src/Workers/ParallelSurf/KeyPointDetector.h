/*
* This file is part of Parallel SURF, which implements the SURF algorithm
* using multi-threading.
*
* Copyright (C) 2010 David Gossow
*
* It is based on the SURF implementation included in Pan-o-matic 0.9.4,
* written by Anael Orlinski.
*
* Parallel SURF is free software; you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation; either version 3 of the License, or
* (at your option) any later version.
*
* Parallel SURF is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef __parallelsurf_keypointdetector_h
#define __parallelsurf_keypointdetector_h

#include "Image.h"
#include "KeyPoint.h"

#include <boost/thread.hpp>
#include "threadpool.hpp"

namespace parallelsurf
{

/// @brief Function object which collects KeyPoints
class KeyPointInsertor
{
  public:
    virtual void operator() ( const KeyPoint &k ) = 0;
};

class KeyPointDetector
{
  public:
    /**
     * @brief default constructor
     * @param iImage integral image to use
     * @param iThreadPool Thread pool to use for computation
     */
    KeyPointDetector ( Image& iImage, boost::threadpool::pool &iThreadPool );

    /// @brief set number of scales per octave
    inline void setMaxScales ( int iMaxScales ) { _maxScales = iMaxScales; }

    /// @brief set number of octaves to search
    inline void setMaxOctaves ( int iMaxOctaves ) { _maxOctaves = iMaxOctaves; }

    /// @brief set minimum threshold on determinant of hessian for detected maxima
    inline void setScoreThreshold ( double iThreshold ) { _scoreThreshold = iThreshold; }

    /**
     * @brief detect and store keypoints
     * @param iImage integral image to use
     * @param iInsertor function object used for storing the keypoints
     */
    void detectKeyPoints ( KeyPointInsertor& iInsertor );

  private:

    int getFilterSize ( int iOctave, int iScale );
    int getBorderSize ( int iOctave, int iScale );

    // internal values of the keypoint detector

    // number of scales
    int _maxScales;

    // number of octaves
    int _maxOctaves;

    // detection score threshold
    double _scoreThreshold;

    // initial box filter size
    int _initialBoxFilterSize;

    // scale overlapping : how many filter sizes to overlap
    // with default value 3 : [3,5,7,9,11][7,11,15,19,23][...
    int _scaleOverlap;

    // some default values.
    const static double kBaseSigma;
    
    Image &_image;

    boost::threadpool::pool &_threadPool;

    /**
     * @brief called by the thread pool for doing the computations
     */
    struct ComputeHelper
    {
      Image& _image;
      double ***_scaleHessian;
      const int _octave;
      const int _octaveWidth;
      const int _octaveHeight;
      const int _pixelStep;
      const int * _borderSize;
      const double _scoreThreshold;
      const int _initialBoxFilterSize;
      const int _maxScales;

      boost::mutex &_insertorMutex;

      /// @brief calculate and store determinant of hessian for given scale
      void calcDet ( int s, int filterSize );
      
      /// @brief detect maxima and store as keypoints
      void detect ( int s, KeyPointInsertor& iInsertor );

      /// @brief improve keypoint localization by interpolation
      bool fineTuneExtrema ( double *** iSH, int iX, int iY, int iS,
                             double& oX, double& oY, double& oS, double& oScore,
                             int iOctaveWidth, int iOctaveHeight, int iBorder );

      /// @brief calculate sign of the trace of the hessian matrix at given position
      bool calcTrace ( Image& iImage, double iX, double iY, double iScale, int& oTrace );
    };

};

}

#endif //__parallelsurf_keypointdetector_h
