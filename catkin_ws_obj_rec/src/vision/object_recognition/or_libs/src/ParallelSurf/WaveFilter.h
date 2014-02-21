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

#ifndef __parallelsurf_wavefilter_h
#define __parallelsurf_wavefilter_h

#include "Image.h"

namespace parallelsurf
{

class WaveFilter
{
  public:
    
    WaveFilter ( double iBaseSize, Image& iImage );

    double getWx ( unsigned int x, unsigned int y );
    double getWy ( unsigned int x, unsigned int y );

    bool checkBounds ( int x, int y ) const;

  private:

    // orig image info
    double**  _ii;
    unsigned int _im_width;
    unsigned int _im_height;

    // internal values
    int _wave_1;
};


inline WaveFilter::WaveFilter ( double iBaseSize, Image& iImage )
{
  _ii = iImage.getIntegralImage();
  _im_width = iImage.getWidth();
  _im_height = iImage.getHeight();

  _wave_1 = ( int ) iBaseSize;
}


#define CALC_INTEGRAL_SURFACE(II, STARTX, ENDX, STARTY, ENDY) \
  (II[ENDY+1][ENDX+1] + II[STARTY][STARTX] - II[ENDY+1][STARTX] - II[STARTY][ENDX+1])


inline double WaveFilter::getWx ( unsigned int x, unsigned int y )
{
  return - CALC_INTEGRAL_SURFACE ( _ii, x - _wave_1, x,    y - _wave_1, y + _wave_1 )
         + CALC_INTEGRAL_SURFACE ( _ii, x,    x + _wave_1, y - _wave_1, y + _wave_1 );
}


inline double WaveFilter::getWy ( unsigned int x, unsigned int y )
{
  return + CALC_INTEGRAL_SURFACE ( _ii, x - _wave_1, x + _wave_1, y - _wave_1, y )
         - CALC_INTEGRAL_SURFACE ( _ii, x - _wave_1, x + _wave_1, y,    y + _wave_1 );
}

inline bool WaveFilter::checkBounds ( int x, int y ) const
{
  return ( x > _wave_1 && x + _wave_1 < ( int ) _im_width - 1
           && y > _wave_1 && y + _wave_1 < ( int ) _im_height - 1 );
}

} // namespace parallelsurf

#endif //__parallelsurf_wavefilter_h
