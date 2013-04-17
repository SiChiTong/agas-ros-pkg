#ifndef _OBB2D_H
#define _OBB_H

#include "vec2.h"
#include <vector>

/**
 * Obolete. Used by Robbie 9 for line clipping.
 * Bad documentation. For questions ask F. Neuhaus.
 */

class OBB2D 
{
  public:

	OBB2D();
	~OBB2D();
	
	CVec2& operator[](int i) { return mPoints[i]; };
	const CVec2& operator[](int i) const { return mPoints[i]; };
	
	float computeClippedArea(const OBB2D& clipPoly);
	
	std::pair<CVec2,CVec2> computeAABB() const;


  private:

	CVec2 mPoints[4];
};

#endif

