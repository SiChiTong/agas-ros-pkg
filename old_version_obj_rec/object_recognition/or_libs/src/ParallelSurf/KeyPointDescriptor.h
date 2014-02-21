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

#ifndef __parallelsurf_keypointdescriptor_h
#define __parallelsurf_keypointdescriptor_h

// #include "Image.h"

#include "threadpool.hpp"

namespace parallelsurf
{
class KeyPoint;
class Image;

class KeyPointDescriptorContext
{
  public:
    KeyPointDescriptorContext ( int iSubRegions, int iVecLen, double iOrientation );
    ~KeyPointDescriptorContext();

    int    _subRegions; // number of square subregions (1 direction)

    double   _sin;   // sinus of orientation
    double   _cos;   // cosinus of orientation

    double***  _cmp;   // descriptor components

    void   placeInIndex ( double iMag1, int iOri1,
                          double iMag2, int iOri2, double iUIdx, double iVIdx );

    void   placeInIndex2 ( double iMag1, int iOri1, double iUIdx, double iVIdx );

  private:
    // disallow stupid things
    KeyPointDescriptorContext();
    KeyPointDescriptorContext ( const KeyPointDescriptorContext& );
    KeyPointDescriptorContext& operator= ( KeyPointDescriptorContext& ) throw();

};

class KeyPointDescriptor
{
  public:

    /**
     * @brief default constructor
     * @param iImage integral image to use
     * @param iThreadPool thread pool for parallelizing the computation
     * @param iExtended calculate extended 128-dimensional descriptor
     */
    KeyPointDescriptor ( Image& iImage, boost::threadpool::pool &iThreadPool, bool iExtended = false );

    /// @brief assign orientation to given keypoint
    void assignOrientation ( KeyPoint& ioKeyPoint ) const;

    /**
     * @brief assign orientations to given keypoints
     * @param iBegin iterator to first keypoint
     * @param iEnd iterator to keypoint where to stop computation (first one after the last)
     */
    template< class IteratorT >
    void assignOrientations ( IteratorT iBegin, IteratorT iEnd );

    /// @brief compute descriptor for single keypoint
    void makeDescriptor ( KeyPoint& ioKeyPoint ) const;

    /**
    * @brief compute descriptors for given keypoints
    * @param iBegin iterator to first keypoint
    * @param iEnd iterator to keypoint where to stop computation (first one after the last)
    */
    template< class IteratorT >
    void makeDescriptors ( IteratorT iBegin, IteratorT iEnd );

    /// @return length of descriptor resulting from current parameters
    int getDescriptorLength() const;

  private:

    // disallow stupid things
    KeyPointDescriptor();
    KeyPointDescriptor ( const KeyPointDescriptor& );
    KeyPointDescriptor& operator= ( KeyPointDescriptor& ) throw();

    // do the actual descriptor computation
    void   createDescriptor ( KeyPointDescriptorContext& iCtx, KeyPoint& ioKeyPoint ) const;

    // orig image info
    Image&   _image;

    // info about the descriptor
    bool   _extended;   // use parallelsurf64 or parallelsurf128
    int    _subRegions;  // number of square subregions. default = 4
    int    _vecLen;   // length of the vector. 4 for parallelsurf 64, 8 for parallelsurf 128
    double   _magFactor;

    boost::threadpool::pool &_threadPool;
};

//polar representation of wavelet response (for orientation assignment)
struct response
{
  float orientation;
  float magnitude;
};

//compares the orientation of two responses
bool operator < ( const response a, const response b );

}


template< class IteratorT >
void parallelsurf::KeyPointDescriptor::assignOrientations ( IteratorT iBegin, IteratorT iEnd )
{
  IteratorT aCurrent = iBegin;
  while ( aCurrent != iEnd )
  {
    _threadPool.schedule ( boost::bind ( &parallelsurf::KeyPointDescriptor::assignOrientation, this, boost::ref ( *aCurrent ) ) );
    aCurrent++;
  }
  _threadPool.wait();
}


template< class IteratorT >
void parallelsurf::KeyPointDescriptor::makeDescriptors ( IteratorT iBegin, IteratorT iEnd )
{
  IteratorT aCurrent = iBegin;
  while ( aCurrent != iEnd )
  {
    _threadPool.schedule ( boost::bind ( &parallelsurf::KeyPointDescriptor::makeDescriptor, this, boost::ref ( *aCurrent ) ) );
    aCurrent++;
  }
  _threadPool.wait();
}


#endif //__parallelsurf_keypointdescriptor_h
