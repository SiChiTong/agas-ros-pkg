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

#include <iostream>
#include <cfloat>

#include <boost/thread/thread.hpp>

#include <stdlib.h>

#include "KeyPoint.h"
#include "KeyPointDetector.h"
#include "BoxFilter.h"
#include "MathStuff.h"

#include "threadpool.hpp"

using namespace parallelsurf;

const double parallelsurf::KeyPointDetector::kBaseSigma = 1.2;

KeyPointDetector::KeyPointDetector ( Image& iImage, boost::threadpool::pool &iThreadPool ) : _image( iImage ), _threadPool ( iThreadPool )
{
  // initialize default values
  _maxScales = 5;  // number of scales per octave (9x9, 15x15, 21x21, 27x27, ...)
  _maxOctaves = 4; // number of octaves

  _scoreThreshold = 0.2;

  _initialBoxFilterSize = 3;
  _scaleOverlap = 3;
}


void KeyPointDetector::detectKeyPoints ( KeyPointInsertor& iInsertor )
{
  // allocate lots of memory for the scales
  double *** aSH = new double**[_maxScales];

  for ( int s = 0; s < _maxScales; ++s )
    aSH[s] = Image::AllocateImage ( _image.getWidth(), _image.getHeight() );

  // init the border size
  int * aBorderSize = new int[_maxScales];

  boost::mutex aInsertorMutex;

  // go through all the octaves
  for ( int o = 0; o < _maxOctaves; ++o )
  {
    // calculate the pixel step on the image, and the image size
    int aPixelStep = 1 << o; // 2^aOctaveIt
    int aOctaveWidth = _image.getWidth() / aPixelStep; // integer division
    int aOctaveHeight = _image.getHeight() / aPixelStep; // integer division

    // calculate the border sizes
    for ( int s = 0; s < _maxScales; ++s )
    {
      aBorderSize[s] = getBorderSize ( o, s );
    }

    ComputeHelper helper = { _image, aSH, o, aOctaveWidth, aOctaveHeight, aPixelStep, aBorderSize, _scoreThreshold, _initialBoxFilterSize, _maxScales, aInsertorMutex };

    // fill scale matrices
    for ( int s = 0; s < _maxScales; ++s )
    {
      _threadPool.schedule ( boost::bind ( &ComputeHelper::calcDet, boost::ref ( helper ), s, getFilterSize ( o, s ) ) );
    }
    _threadPool.wait();

    // detect the feature points with a 3x3x3 neighborhood non-maxima suppression
    for ( int s = 1; s < ( _maxScales - 1 ); s += 2 )
    {
      _threadPool.schedule ( boost::bind ( &ComputeHelper::detect, boost::ref ( helper ), s, boost::ref ( iInsertor ) ) );
    }
    _threadPool.wait();
  }
  
  // deallocate memory of the scale images
  for ( int s = 0; s < _maxScales; ++s )
    Image::DeallocateImage ( aSH[s], _image.getHeight() );
}

bool KeyPointDetector::ComputeHelper::fineTuneExtrema ( double *** iSH, int iX, int iY, int iS,
    double& oX, double& oY, double& oS, double& oScore,
    int iOctaveWidth, int iOctaveHeight, int iBorder )
{
  // maximum fine tune iterations
  const int kMaxFineTuneIters = 6;

  // shift from the initial position for X and Y (only -1 or + 1 during the iterations).
  int aX = iX;
  int aY = iY;
  int aS = iS;

  int aShiftX = 0;
  int aShiftY = 0;

  // current deviations
  double aDx = 0, aDy = 0, aDs = 0;

  //result vector
  double aV[3]; //(x,y,s)

  for ( int aIter = 0; aIter < kMaxFineTuneIters; ++aIter )
  {
    // update the extrema position
    aX += aShiftX;
    aY += aShiftY;

    // create the problem matrix
    double aM[3][3]; //symetric, no ordering problem.

    // fill the result vector with gradient from pixels differences (negate to prepare system solve)
    aDx = ( iSH[aS  ][aY  ][aX+1] - iSH[aS  ][aY  ][aX-1] ) * 0.5;
    aDy = ( iSH[aS  ][aY+1][aX  ] - iSH[aS  ][aY-1][aX  ] ) * 0.5;
    aDs = ( iSH[aS+1][aY  ][aX  ] - iSH[aS-1][aY  ][aX  ] ) * 0.5;

    aV[0] = - aDx;
    aV[1] = - aDy;
    aV[2] = - aDs;

    // fill the matrix with values of the hessian from pixel differences
    aM[0][0] = iSH[aS  ][aY  ][aX-1] - 2.0 * iSH[aS][aY][aX] + iSH[aS  ][aY  ][aX+1];
    aM[1][1] = iSH[aS  ][aY-1][aX  ] - 2.0 * iSH[aS][aY][aX] + iSH[aS  ][aY+1][aX  ];
    aM[2][2] = iSH[aS-1][aY  ][aX  ] - 2.0 * iSH[aS][aY][aX] + iSH[aS+1][aY  ][aX  ];

    aM[0][1] = aM[1][0] = ( iSH[aS  ][aY+1][aX+1] + iSH[aS  ][aY-1][aX-1] - iSH[aS  ][aY+1][aX-1] - iSH[aS  ][aY-1][aX+1] ) * 0.25;
    aM[0][2] = aM[2][0] = ( iSH[aS+1][aY  ][aX+1] + iSH[aS-1][aY  ][aX-1] - iSH[aS+1][aY  ][aX-1] - iSH[aS-1][aY  ][aX+1] ) * 0.25;
    aM[1][2] = aM[2][1] = ( iSH[aS+1][aY+1][aX  ] + iSH[aS-1][aY-1][aX  ] - iSH[aS+1][aY-1][aX  ] - iSH[aS-1][aY+1][aX  ] ) * 0.25;

    // solve the linear system. results are in aV. exit with error if a problem happened
    if ( !Math::SolveLinearSystem33 ( aV, aM ) )
    {
      return false;
    }

    // ajust the shifts with the results and stop if no significant change
    if ( aIter < kMaxFineTuneIters - 1 )
    {
      aShiftX = 0;
      aShiftY = 0;

      if ( aV[0] > 0.6 && aX < ( int ) ( iOctaveWidth - iBorder - 2 ) )
        aShiftX++;
      else if ( aV[0] < -0.6 && aX > ( int ) iBorder + 1 )
        aShiftX--;

      if ( aV[1] > 0.6 && aY < ( int ) ( iOctaveHeight - iBorder - 2 ) )
        aShiftY++;
      else if ( aV[1] < -0.6 && aY > ( int ) iBorder + 1 )
        aShiftY--;

      if ( aShiftX == 0 && aShiftY == 0 )
        break;
    }
  }

  // update the score
  oScore = iSH[aS][aY][aX] + 0.5 * ( aDx * aV[0] + aDy * aV[1] + aDs * aV[2] );

  // reject too big deviation in last step (unfinished job).
  if ( Math::Abs ( aV[0] ) > 1.5 || Math::Abs ( aV[1] ) > 1.5  || Math::Abs ( aV[2] ) > 1.5 )
    return false;

  // put the last deviation (not integer :) to the output
  oX = aX + aV[0];

  oY = aY + aV[1];

  oS = iS + aV[2];

  return true;
}

int  KeyPointDetector::getFilterSize ( int iOctave, int iScale )
{
  // base size + 3 times first increment for step back
  // for the first octave 9x9, 15x15, 21x21, 27x27, 33x33
  // for the second 21x21, 33x33, 45x45 ...
  int aScaleShift = 2 << iOctave;
  return _initialBoxFilterSize + ( aScaleShift - 2 ) * ( _maxScales - _scaleOverlap ) + aScaleShift * iScale;
}

int  KeyPointDetector::getBorderSize ( int iOctave, int iScale )
{
  int aScaleShift = 2 << iOctave;

  if ( iScale <= 2 )
  {
    int aMult = ( iOctave == 0 ? 1 : 2 );
    return ( getFilterSize ( iOctave, 1 ) + aMult * aScaleShift ) * 3 / aScaleShift + 1;
  }

  return getFilterSize ( iOctave, iScale ) * 3 / aScaleShift + 1;
}

bool KeyPointDetector::ComputeHelper::calcTrace ( Image& iImage,
    double iX,
    double iY,
    double iScale,
    int& oTrace )
{
  int aRX = Math::Round ( iX );
  int aRY = Math::Round ( iY );

  BoxFilter aBox ( 3*iScale, iImage );

  if ( !aBox.checkBounds ( aRX, aRY ) )
    return false;

  aBox.setY ( aRY );

  double aTrace = aBox.getDxxWithX ( aRX ) + aBox.getDyyWithX ( aRX );

  oTrace = ( aTrace <= 0.0 ? -1 : 1 );

  return true;
}


void KeyPointDetector::ComputeHelper::calcDet ( int s, int filterSize )
{
  // create a box filter of the correct size.
  BoxFilter aBoxFilter ( filterSize, _image );

  // calculate the border for this scale
  const int aBS = _borderSize[ s ];

  // fill the hessians
  int aEy = _octaveHeight - aBS;
  int aEx = _octaveWidth - aBS;

  int aYPS = aBS * _pixelStep;

  for ( int y = aBS; y < aEy; ++y )
  {
    aBoxFilter.setY ( aYPS );
    int aXPS = aBS * _pixelStep;

    for ( int x = aBS; x < aEx; ++x )
    {
      _scaleHessian[s][y][x] = aBoxFilter.getDetWithX ( aXPS );
      aXPS += _pixelStep;
    }

    aYPS += _pixelStep;
  }
}

void KeyPointDetector::ComputeHelper::detect ( int s, KeyPointInsertor& iInsertor )
{
  int aMaxReachableScale = s + 2;
  if ( aMaxReachableScale >= _maxScales )
  {
    aMaxReachableScale = _maxScales - 1;
  }
  const int aBS = _borderSize[ aMaxReachableScale ];

  double aTab[8];
  
//   if ( ( _octaveHeight - aBS - 2 < 0 )
    

  for ( int aYIt = aBS + 1; aYIt < _octaveHeight - aBS - 2; aYIt += 2 )
  {
    for ( int aXIt = aBS + 1; aXIt < _octaveWidth - aBS - 2; aXIt += 2 )
    {
      // find the maximum in the 2x2x2 cube

      // get the values in a
      aTab[0] = _scaleHessian[s]  [aYIt]  [aXIt];
      aTab[1] = _scaleHessian[s]  [aYIt]  [aXIt+1];
      aTab[2] = _scaleHessian[s]  [aYIt+1][aXIt];
      aTab[3] = _scaleHessian[s]  [aYIt+1][aXIt+1];
      aTab[4] = _scaleHessian[s+1][aYIt]  [aXIt];
      aTab[5] = _scaleHessian[s+1][aYIt]  [aXIt+1];
      aTab[6] = _scaleHessian[s+1][aYIt+1][aXIt];
      aTab[7] = _scaleHessian[s+1][aYIt+1][aXIt+1];
      
      // find the max index without using a loop.
      int a04 = ( aTab[0] > aTab[4] ? 0 : 4 );
      int a15 = ( aTab[1] > aTab[5] ? 1 : 5 );
      int a26 = ( aTab[2] > aTab[6] ? 2 : 6 );
      int a37 = ( aTab[3] > aTab[7] ? 3 : 7 );
      int a0426 = ( aTab[a04] > aTab[a26] ? a04 : a26 );
      int a1537 = ( aTab[a15] > aTab[a37] ? a15 : a37 );
      int aMaxIdx = ( aTab[a0426] > aTab[a1537] ? a0426 : a1537 );
      
      // calculate approximate threshold
      double aApproxThres = _scoreThreshold * 0.8;

      double aScore = aTab[aMaxIdx];

      // check found point against threshold
      if ( aScore < aApproxThres )
        continue;

      // verify that other missing points in the 3x3x3 cube are also below treshold

      // aXShift: 2*0-1 = -1; 2*1-1=1
      // aXAdj = aXIt (no adjustment)      -->  aXShift = -1
      // aXAdj = aXIt+1 (with adjustment)  -->  aXShift =  1
      int aXShift = 2 * ( aMaxIdx & 1 ) - 1;
      int aXAdj = aXIt + ( aMaxIdx & 1 );
      
      aMaxIdx >>= 1;

      int aYShift = 2 * ( aMaxIdx & 1 ) - 1;
      int aYAdj = aYIt + ( aMaxIdx & 1 ); 
      
      aMaxIdx >>= 1;

      int aSShift = 2 * ( aMaxIdx & 1 ) - 1;
      int aSAdj = s + ( aMaxIdx & 1 );

      // skip too high scale adjusting
      if ( aSAdj == ( int ) _maxScales - 1 )
        continue;

      if ( ( _scaleHessian[aSAdj + aSShift][aYAdj - aYShift][aXAdj - 1] > aScore ) ||
           ( _scaleHessian[aSAdj + aSShift][aYAdj - aYShift][aXAdj    ] > aScore ) ||
           ( _scaleHessian[aSAdj + aSShift][aYAdj - aYShift][aXAdj + 1] > aScore ) ||
           ( _scaleHessian[aSAdj + aSShift][aYAdj   ][aXAdj - 1] > aScore ) ||
           ( _scaleHessian[aSAdj + aSShift][aYAdj   ][aXAdj    ] > aScore ) ||
           ( _scaleHessian[aSAdj + aSShift][aYAdj   ][aXAdj + 1] > aScore ) ||
           ( _scaleHessian[aSAdj + aSShift][aYAdj + aYShift][aXAdj - 1] > aScore ) ||
           ( _scaleHessian[aSAdj + aSShift][aYAdj + aYShift][aXAdj    ] > aScore ) ||
           ( _scaleHessian[aSAdj + aSShift][aYAdj + aYShift][aXAdj + 1] > aScore ) ||

           ( _scaleHessian[aSAdj][  aYAdj + aYShift ][aXAdj - 1] > aScore ) ||
           ( _scaleHessian[aSAdj][   aYAdj + aYShift ][aXAdj] > aScore ) ||
           ( _scaleHessian[aSAdj][  aYAdj + aYShift ][aXAdj + 1] > aScore ) ||
           ( _scaleHessian[aSAdj][ aYAdj   ][aXAdj + aXShift] > aScore ) ||
           ( _scaleHessian[aSAdj][ aYAdj - aYShift ][aXAdj + aXShift] > aScore ) ||

           ( _scaleHessian[aSAdj - aSShift][  aYAdj + aYShift ][aXAdj - 1] > aScore ) ||
           ( _scaleHessian[aSAdj - aSShift][   aYAdj + aYShift ][aXAdj] > aScore ) ||
           ( _scaleHessian[aSAdj - aSShift][  aYAdj + aYShift ][aXAdj + 1] > aScore ) ||
           ( _scaleHessian[aSAdj - aSShift][ aYAdj   ][aXAdj + aXShift] > aScore ) ||
           ( _scaleHessian[aSAdj - aSShift][ aYAdj - aYShift ][aXAdj + aXShift] > aScore )
         )
        continue;

      // fine tune the location
      double aX = aXAdj;
      double aY = aYAdj;
      double aS = aSAdj;

      // try to fine tune, restore the values if it failed
      // if the returned value is true,  keep the point, else drop it.
      if ( !fineTuneExtrema ( _scaleHessian, aXAdj, aYAdj, aSAdj, aX, aY, aS, aScore, _octaveWidth, _octaveHeight, _borderSize[aSAdj] ) )
      {
        continue;
      }

      // recheck the updated score
      if ( aScore < _scoreThreshold )
      {
        continue;
      }

      // adjust the values
      aX *= _pixelStep;

      aY *= _pixelStep;

      aS = ( ( 2 * aS * _pixelStep ) + _initialBoxFilterSize + ( _pixelStep - 1 ) * _maxScales ) / 3.0; // this one was hard to guess...

      // store the point
      int aTrace;

      if ( !calcTrace ( _image, aX, aY, aS, aTrace ) )
      {
        continue;
      }

      // do something with the keypoint depending on the insertor
      _insertorMutex.lock();
      iInsertor ( KeyPoint ( aX, aY, aS * kBaseSigma, aScore, aTrace ) );
      _insertorMutex.unlock();
    }
  }
}
