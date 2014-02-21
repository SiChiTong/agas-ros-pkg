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

#include "MathStuff.h"
#include <math.h>

using namespace parallelsurf;

bool Math::SolveLinearSystem33(double *solution, double sq[3][3])
{
	const int size = 3;
	int row, col, c, pivot = 0, i;
	double maxc, coef, temp, mult, val;

	/* Triangularize the matrix. */
	for (col = 0; col < size - 1; col++) 
	{
		/* Pivot row with largest coefficient to top. */
		maxc = -1.0;
		for (row = col; row < size; row++) 
		{
			coef = sq[row][col];
			coef = (coef < 0.0 ? - coef : coef);
			if (coef > maxc)
			{
				maxc = coef;
				pivot = row;
			}
		}
		if (pivot != col)
		{
			/* Exchange "pivot" with "col" row (this is no less efficient
			than having to perform all array accesses indirectly). */
			for (i = 0; i < size; i++) 
			{
				temp = sq[pivot][i];
				sq[pivot][i] = sq[col][i];
				sq[col][i] = temp;
			}
			temp = solution[pivot];
			solution[pivot] = solution[col];
			solution[col] = temp;
		}
		/* Do reduction for this column. */
		for (row = col + 1; row < size; row++) 
		{
			mult = sq[row][col] / sq[col][col];
			for (c = col; c < size; c++)	/* Could start with c=col+1. */
				sq[row][c] -= mult * sq[col][c];
			solution[row] -= mult * solution[col];
		}
	}

	/* Do back substitution.  Pivoting does not affect solution order. */
	for (row = size - 1; row >= 0; row--) {
		val = solution[row];
		for (col = size - 1; col > row; col--)
			val -= solution[col] * sq[row][col];
		solution[row] = val / sq[row][row];
	}
	return true;
}

bool Math::Normalize( std::vector<double> &iVec )
{
	int i;
	double val, fac, sqlen = 0.0;

	for (i = 0; i < iVec.size(); i++) {
		val = iVec[i];
		sqlen += val * val;
	}
	if (sqlen == 0.0)
		return false;

	fac = 1.0 / sqrt(sqlen);
	for (i = 0; i < iVec.size(); i++)
		iVec[i] *= fac;

	return true;
}

