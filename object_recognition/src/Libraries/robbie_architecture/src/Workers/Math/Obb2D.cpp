#include <stdlib.h>


#include "Obb2D.h"
#include <iostream>

//----------------------------------------------------------------------------------------------------------
OBB2D::OBB2D()
{
}
//----------------------------------------------------------------------------------------------------------
OBB2D::~OBB2D()
{
}
//----------------------------------------------------------------------------------------------------------
std::pair<CVec2,CVec2> OBB2D::computeAABB() const
{
	CVec2 mins(999999,999999);
	CVec2 maxs(-999999,-999999);
	for (int i=0;i<4;i++)
	{
		if (mPoints[i][0]<mins[0]) mins[0]=mPoints[i][0];
		if (mPoints[i][1]<mins[1]) mins[1]=mPoints[i][1];
		if (mPoints[i][0]>maxs[0]) maxs[0]=mPoints[i][0];
		if (mPoints[i][1]>maxs[1]) maxs[1]=mPoints[i][1];
	}
	return std::make_pair(mins,maxs);
}
//----------------------------------------------------------------------------------------------------------
// Polygon an Kante (clipStart, clipEnd) clippen, Originalpunkte werden ueberschrieben
int clipEdge(const CVec2* points, int numPoints, const CVec2& clipStart, const CVec2& clipEnd, CVec2* dest)
{
	int result=0;

	// Normale zeigt nach aussen
	CVec2 n(clipEnd[1] - clipStart[1],
		       clipStart[0] - clipEnd[0]);

	//n=n*(-1);

	float d0=clipStart*n;

	int i=numPoints-1;
	for (int iNext=0;iNext<numPoints;iNext++)
	{
		const CVec2& a=points[i];
		const CVec2& b=points[iNext];

		float adotn=a*n;
		float bdotn=b*n;
		float da=adotn-d0;
		float db=bdotn-d0;

		if (da<=0)
		{
			if (db<=0)
			{
				// beide innerhalb
				//outputPoints.push_back(b);
				dest[result++]=b;
			}
			else
			{
				// a drinnen, b draussen => schnittpunkt
				float t=-da/(bdotn-adotn);
				//outputPoints.push_back(a+t*(b-a));
				dest[result++]=a+t*(b-a);
			}
		}
		else
		{
			if (db<=0)
			{
				// a draussen, b drinnen => schnittpunkt
				float t=-da/(bdotn-adotn);
				//outputPoints.push_back(a+t*(b-a));
				//outputPoints.push_back(b);
				dest[result++]=a+t*(b-a);
				dest[result++]=b;
			}
			else
			{
				// beide ausserhalb
			}
		}

		i=iNext;
	}

	return result;
}
//----------------------------------------------------------------------------------------------------------
float OBB2D::computeClippedArea(const OBB2D& clipPoly)
{
	CVec2* tmp1=(CVec2*)alloca(20*sizeof(CVec2));
	CVec2* tmp2=(CVec2*)alloca(20*sizeof(CVec2));

	int res=0;

	res=clipEdge(mPoints,4,clipPoly[3],clipPoly[0],tmp1);
	res=clipEdge(tmp1,res,clipPoly[0],clipPoly[1],tmp2);
	res=clipEdge(tmp2,res,clipPoly[1],clipPoly[2],tmp1);
	res=clipEdge(tmp1,res,clipPoly[2],clipPoly[3],tmp2);

	/*
	static int maxres=0;
	if (res>=maxres)
	{
		std::cout << "res: " << res << std::endl;
		maxres=res;
	}*/


	float area=0;
	for (int i=0;i<res;i++)
	{
		area+=(tmp2[i][1]+tmp2[(i+1)%res][1])*(tmp2[i][0]-tmp2[(i+1)%res][0]);
	}
	area=0.5f*fabs(area);
	return area;
}
//----------------------------------------------------------------------------------------------------------
/*float Polygon2D::computeArea() const
{
#if 0
	float area=0;
	CVec2 last=mPoints[1];
	for (int i=2;i<mPoints.size();i++)
	{
		CVec2 a=last-mPoints[0];
		CVec2 b=mPoints[i]-mPoints[0];
		area+=0.5f*fabs(a.x*b.y-a.y*b[0]);

		last=mPoints[i];
	}
	return area;
#else
	float area2=0;
	for (int i=0;i<mPoints.size();i++)
	{
		area2+=(mPoints[i][1]+mPoints[(i+1)%mPoints.size()][1])*(mPoints[i][0]-mPoints[(i+1)%mPoints.size()][0]);//mPoints[i].x*mPoints[i+1].y-mPoints[i+1].x*mPoints[i][1];
	}
	area2=0.5f*fabs(area2);
	//std::cout << "area: " << area << " " << area2 << std::endl;
	return area2;
#endif
}*/
//----------------------------------------------------------------------------------------------------------

