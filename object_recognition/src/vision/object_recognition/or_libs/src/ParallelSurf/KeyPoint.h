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

#ifndef __parallelsurf_keypoint_h
#define __parallelsurf_keypoint_h

#include <vector>

namespace parallelsurf {

class KeyPoint
{
public:
	KeyPoint();
	KeyPoint(double x, double y, double s, double score, int trace);
	
	double		_x, _y;
	double		_scale;
	double		_score;
	int			_trace;
	double		_ori;

	std::vector<double> _vec;
};

inline KeyPoint::KeyPoint()
{

}

inline KeyPoint::KeyPoint(double x, double y, double s, double score, int trace) : 
	_x(x), _y(y), _scale(s), _score(score), _trace(trace), _vec(0)
{

}

inline bool operator < (const KeyPoint & iA, const KeyPoint & iB)
{
	return (iA._score < iB._score);
}

}

#endif //__parallelsurf_keypoint_h

