/*******************************************************************************
 *  misc.h
 *
 *  (C) 2006 AG Aktives Sehen <agas@uni-koblenz.de>
 *           Universitaet Koblenz-Landau
 *
 * Author: Frank Neuhaus
 *******************************************************************************/

#ifndef MISC_H
#define MISC_H

template<class T>
T sqr(T f)
{
	return f*f;
}

bool intersectRayCircle(const CVec2& m, float r, const CVec2& x, const CVec2& t, float& f);
bool intersectRayLineSegment(const CVec2& a, const CVec2& b, const CVec2& x, const CVec2& t, float&f);
float shortestDistanceToLineSegment(const CVec2& a, const CVec2& b, const CVec2& x);
bool intersectRayCapsule(const CVec2& x, const CVec2& t, const CVec2& a, const CVec2& b, float radius, float& f);

bool intersectPathCircle(const CVec2& c, const CVec2& p, float r0, const CVec2& hd, const CVec2& m1, float r1, float& angle);
bool intersectPathLine(const CVec2& c, const CVec2& p, float r0, const CVec2& hd, const CVec2& p1, const CVec2& p2, float& angle);

bool intersectPathCapsule(const CVec2& c, const CVec2& p, float r0, const CVec2& hd, const CVec2& a, const CVec2& b, float radius, float& f);

float computeOBBIntersection(const CVec2& a, const CVec2& b, const CVec2& c, const CVec2& d, float size);

#endif

