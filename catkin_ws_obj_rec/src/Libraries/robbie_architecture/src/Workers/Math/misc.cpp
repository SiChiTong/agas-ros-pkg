/*******************************************************************************
 *  misc.cpp
 *
 *  (C) 2006 AG Aktives Sehen <agas@uni-koblenz.de>
 *           Universitaet Koblenz-Landau
 *
 * Author: Frank Neuhaus
 *******************************************************************************/

#include "vec2.h"
#include "misc.h"
#include <iostream>
#include "math.h"
#include <assert.h>
#include "Obb2D.h"

/**
*	m - center of the circle
*	r - radius of the circle
*	x - starting point of the ray
*	t - ray
*	f - param - return value
*/
bool intersectRayCircle(const CVec2& m, float r, const CVec2& x, const CVec2& t, float& f)
{
	//|x-m|=r
	//|x+r*t-m|=r
	//(x+r*t-m)^2=r^2
	//((x-m)+r*t)^2=r^2
	//(x-m)^2+2*(x-m)*r*t+r^2*t^2=r^2
	
	//r^2*t^2+r*2*(x-m)*t+(x-m)^2-r^2
	
	
	float invtSqr=1.0f/t.sqr();
	float p = 2.0f*((x-m)*t)*invtSqr;
	float q = ((m-x).sqr() - r*r)*invtSqr;	
	float diskr = p*p*0.25f - q;
	
	if (diskr < 0)
	{
		return false;
	}

	diskr=sqrtf(diskr);
	f = -0.5f*p - diskr;
	
	if (f<0)
	{
		f = -0.5f*p + diskr;
		if (f>0)
		{
			return true;
		}
		else
		{
			return false;
		}
	}
/*	
	if (f<0)
	{
		std::cout << "smaller"  << f << std::endl;
		if (t*(m-x)>0)
		{
			f=0.001;
			return true;
		}
		else
			return false;
	}
	*/
	return true;

}


bool intersectRayLineSegment(const CVec2& a, const CVec2& b, const CVec2& x, const CVec2& t, float&f)
{
	CVec2 n=(b-a).getNormal();
	
	float denom=t*n;
		
	if (fabs(denom)<0.000001f)
		return false;
		
	f=(n*a-n*x)/denom;
	
	if (f<0)
	{
		return false;
	}
		
	CVec2 pt=x+t*f;
	if ((a-pt)*(b-pt)<0)
		return true;	
	return false;
}

float shortestDistanceToLineSegment(const CVec2& a, const CVec2& b, const CVec2& x)
{
	CVec2 dir=b-a;
	
	if ( (dir*dir) < 0.00001 )
	{
		float m1=(x-a).magnitude();
		float m2=(x-b).magnitude();
		if (m1<m2) return m1;
		else return m2;
	}
		
	float r=(dir*x-dir*a)/(dir*dir);
	if (r<=0.0f)
		return (x-a).magnitude();
	if (r>=1.0f)
		return (x-b).magnitude();
	
	return (a+r*dir-x).magnitude();
	
}

bool intersectRayCapsule(const CVec2& x, const CVec2& t, const CVec2& a, const CVec2& b, float radius, float& f)
{
	f=99999999;
	float r;
	bool hadInt=false;
	if (intersectRayCircle(a,radius,x,t,r))
	{
		if (r<f)
			f=r;
		hadInt=true;
	}
	if (intersectRayCircle(b,radius,x,t,r))
	{
		if (r<f)
			f=r;
		hadInt=true;
	}
	CVec2 n=normalize(b-a).getNormal();
	if (intersectRayLineSegment(a+radius*n,b+radius*n,x,t,r))
	{
		if (r<f)
			f=r;
		hadInt=true;
	}
	if (intersectRayLineSegment(a-radius*n,b-radius*n,x,t,r))
	{
		if (r<f)
			f=r;
		hadInt=true;
	}
	return hadInt;
}

// c - punkt wo arc startet
// p - zentrum des arcs
// r0 - arc radius
// hd - heading
// m1 - kreiszentrum
// r1 - kreisradius
// angle - output
bool intersectPathCircle(const CVec2& c, const CVec2& p, float r0, const CVec2& hd, const CVec2& m1, float r1, float& angle)
{
	CVec2 vec=p-m1;
	float s=vec.sqr();
	if (s>sqr(fabs(r0)+r1))	
		return false;
	
	if (s<sqr(fabs(r0)-r1))
		return false;
	
	float d=sqrtf(s);
	
	float b=(r0*r0-r1*r1+s)/(2*d);
	
	CVec2 mid=p-vec*(b/d);
	
	float h=sqrtf(r0*r0-b*b);
	//std::cout << " h :  " << h << std::endl;
	
	CVec2 n=(vec*(h/d)).getNormal();
	
	
	//CVec2 pm=p-m1;
	
	CVec2 p1=(mid+n-p);
	CVec2 p2=(mid-n-p);
	
	CVec2 pc=normalize(c-p);
	
	float angle0=acosf(normalize(p1)*pc);
	float angle1=acosf(normalize(p2)*pc);
	
	//std::cout << "angle0: " << angle0 << std::endl;
	
	if ((p1*hd)<0) angle0=2*M_PI-angle0;
	if ((p2*hd)<0) angle1=2*M_PI-angle1;
	
	if (angle0<angle1) angle=angle0;
	else angle=angle1;
	
	//angle=angle0;
	
	return true;
	

}

bool intersectPathLine(const CVec2& c, const CVec2& p, float r0, const CVec2& hd, const CVec2& p1, const CVec2& p2, float& angle)
{
	CVec2 vec=p2-p1;

	
	/*
	float invtSqr=1.0f/t.sqr();
	float p = 2.0f*((x-m)*t)*invtSqr;
	float q = ((m-x).sqr() - r*r)*invtSqr;	
	float diskr = p*p*0.25f - q;
	*/
	
	if (((p1-p).sqr()<r0*r0)&&((p2-p).sqr()<r0*r0))
		return false;
	
	
	float invtSqr=1.0f/vec.sqr();
	float pa = 2.0f*((p1-p)*vec)*invtSqr;
	float qi = ((p-p1).sqr() - r0*r0)*invtSqr;	
	float diskr = pa*pa*0.25f - qi;
	
	if (diskr < 0)
	{
		return false;
	}
	
	float sqrtfdiskr=sqrtf(diskr);

	float f1 = -0.5f*pa - sqrtfdiskr;
	float f2 = -0.5f*pa + sqrtfdiskr;
	
	if (f1>1) return false;
	if (f2<0) return false;
	
	//std::cout << "f1: " << f1 << " f2: " << f2 << std::endl;
	
	CVec2 int1=p1+f1*vec;
	CVec2 int2=p1+f2*vec;
	
	//assert((int2-int1).magnitude()<(p1-p2).magnitude());
	
	CVec2 pc=normalize(c-p);
	
	float angle0=acosf(normalize(int1-p)*pc);
	float angle1=acosf(normalize(int2-p)*pc);
	
	
	if ((int1-p)*hd<0)
	{
		angle0=2*M_PI-angle0;
	}
	if ((int2-p)*hd<0)
	{
		angle1=2*M_PI-angle1;
	}
	
	
	if (angle0<angle1) angle=angle0;
	else angle=angle1;
	
	if (f1<0) angle=angle1;
	if (f2>1) angle=angle0;

	
	
	return true;
	
}

// c - punkt wo arc startet
// p - arc zentrum
// r0 - radius des arcs
// hd - heading
// a - linienanfang
// b - linienende
// radius - linienradius
// r [out] - wie weit ging der arc?

bool intersectPathCapsule(const CVec2& c, const CVec2& p, float r0, const CVec2& hd, const CVec2& a, const CVec2& b, float radius, float& f)
{
	f=99999999;
	float r;
	bool hadInt=false;
	if (intersectPathCircle(c,p,r0,hd,a,radius,r))
	{
		if (r<f)
			f=r;
		hadInt=true;
	}
	if (intersectPathCircle(c,p,r0,hd,b,radius,r))
	{
		if (r<f)
			f=r;
		hadInt=true;
	}
	CVec2 n=normalize(b-a).getNormal();
	if (intersectPathLine(c,p,r0,hd,a+radius*n,b+radius*n,r))
	{
		if (r<f)
			f=r;
		hadInt=true;
	}
	if (intersectPathLine(c,p,r0,hd,a-radius*n,b-radius*n,r))
	{
		if (r<f)
			f=r;
		hadInt=true;
	}
	return hadInt;
}


bool isInAABB(const std::pair<CVec2,CVec2>& aabb, const CVec2& p)
{
	const CVec2& mins=aabb.first;
	const CVec2& maxs=aabb.second;
	if ((p[0]>mins[0]) &&(p[1]>mins[1])&&(p[0]<maxs[0]) &&(p[1]<maxs[1]))
		return true;
	return false;
}

bool testAABBOverlap(const std::pair<CVec2,CVec2>& a, const std::pair<CVec2,CVec2>& b)
{
	const CVec2& vMins=b.first;
	const CVec2& vMaxs=b.second;
	CVec2 B=(vMins+vMaxs)*0.5f;
	CVec2 A=(a.first+a.second)*0.5f;
	CVec2 E=a.second-A;
	CVec2 bE=vMaxs-B;

	const CVec2 T = B - A;//vector from A to B
	return fabs(T[0])  <= (E[0] + bE[0]) 
			&&
			fabs(T[1]) <= (E[1] + bE[1]);

}

float computeOBBIntersection(const CVec2& a, const CVec2& b, const CVec2& c, const CVec2& d, float size)
{
	OBB2D g;
	CVec2 dab=normalize(b-a)*size;
	CVec2 nab=dab.getNormal();
	g[0]=a+nab-dab;
	g[1]=a-nab-dab;
	g[2]=b-nab+dab;
	g[3]=b+nab+dab;
	
	
	OBB2D h;
	CVec2 dcd=normalize(d-c)*size;
	CVec2 ncd=dcd.getNormal();
	h[0]=c+ncd-dcd;
	h[1]=c-ncd-dcd;
	h[2]=d-ncd+dcd;
	h[3]=d+ncd+dcd;
	
	std::pair<CVec2,CVec2> aabb=g.computeAABB();
	std::pair<CVec2,CVec2> aabb2=h.computeAABB();
	
	//static int all=0;
	//all++;
	
	
	if (!testAABBOverlap(aabb,aabb2))
	{
		return 0;
	}
	
	//h.clip(g);
	/*
	static int clips=0;
	clips++;
	if (clips%10000==0)
		std::cout<<"clips: " << clips << " rate: " << float(clips)/float(all) << std::endl;
	*/
	return h.computeClippedArea(g)/(2*size*((a-b).magnitude()+2*size));
	
}




