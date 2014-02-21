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

#ifndef __parallelsurf_math_h
#define __parallelsurf_math_h

#include <vector>

#define PI 3.14159

namespace parallelsurf
{

struct Math
{
  static bool SolveLinearSystem33 ( double *solution, double sq[3][3] );
  static inline double Abs ( const double iD ) { return ( iD > 0.0 ? iD : -iD ); }
  static inline int  Round ( const double iD ) { return ( int ) ( iD + 0.5 ); }
  static bool Normalize ( std::vector<double> &iVec );
};

template < int LBound = -128, int UBound = 127, class TResult = double, class TArg = double >
class LUT
{
  public:
    explicit LUT ( TResult ( *f ) ( TArg ), double coeffadd = 0, double coeffmul = 1 )
    {
      lut = lut_array - LBound;
      for ( int i = LBound; i <= UBound; i++ )
      {
        lut[i] = f ( coeffmul * ( i + coeffadd ) );
      }
    }

    const TResult & operator() ( int i ) const
    {
      return lut[i];
    }
  private:
    TResult lut_array[UBound - LBound + 1];
    TResult * lut;
};

}

#endif //__parallelsurf_math_h
