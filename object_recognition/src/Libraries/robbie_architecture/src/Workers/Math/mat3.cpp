/*******************************************************************************
 *  mat3.h
 *
 *  (C) 2007 AG Aktives Sehen <agas@uni-koblenz.de>
 *           Universitaet Koblenz-Landau
 *
 * Author: Frank Neuhaus, Susanne Maur
 *******************************************************************************/

#include "mat3.h"
#include "math.h"

#define THIS CMat3

//----------------------------------------------------------------------------------------------------------
THIS::~CMat3()
{
}
//----------------------------------------------------------------------------------------------------------
void THIS::transpose()
{
	float temp;

	temp=fMatrix[1];
	fMatrix[1]=fMatrix[3];
	fMatrix[3]=temp;

	temp=fMatrix[2];
	fMatrix[2]=fMatrix[6];
	fMatrix[6]=temp;

	temp=fMatrix[5];
	fMatrix[5]=fMatrix[7];

	fMatrix[7]=temp;
}

//----------------------------------------------------------------------------------------------------------

void THIS::reverse() 
{
  CMat3 temp;

  temp[0]=fMatrix[4]*fMatrix[8] - fMatrix[5]*fMatrix[7]; 
  temp[1]=fMatrix[2]*fMatrix[7] - fMatrix[1]*fMatrix[8];    
  temp[2]=fMatrix[1]*fMatrix[5] - fMatrix[2]*fMatrix[4];
  temp[3]=fMatrix[5]*fMatrix[6] - fMatrix[3]*fMatrix[8]; 
  temp[4]=fMatrix[0]*fMatrix[8] - fMatrix[2]*fMatrix[6];    
  temp[5]=fMatrix[2]*fMatrix[3] - fMatrix[0]*fMatrix[5];
  temp[6]=fMatrix[3]*fMatrix[7] - fMatrix[4]*fMatrix[6]; 
  temp[7]=fMatrix[1]*fMatrix[6] - fMatrix[0]*fMatrix[7];    
  temp[8]=fMatrix[0]*fMatrix[4] - fMatrix[1]*fMatrix[3];

  *this = (temp) * (1.0/determinant());
}

//----------------------------------------------------------------------------------------------------------
void THIS::loadIdentity()
{
	fMatrix[0]=1.0f; fMatrix[1]=0;    fMatrix[2]=0;
	fMatrix[3]=0;    fMatrix[4]=1.0f; fMatrix[5]=0;
	fMatrix[6]=0;    fMatrix[7]=0;    fMatrix[8]=1.0f;
}
//----------------------------------------------------------------------------------------------------------
void THIS::makeRotationX(float fA)
{
	// 1  0  0  0
	// 0  c -s  0
	// 0  s  c  0
	// 0  0  0  1
	float c=cosf(fA);
	float s=sinf(fA);
	fMatrix[0]=1; fMatrix[1]=0; fMatrix[2]=0; 
	fMatrix[3]=0; fMatrix[4]=c; fMatrix[5]=s; 
	fMatrix[6]=0; fMatrix[7]=-s; fMatrix[8]=c;

}
//----------------------------------------------------------------------------------------------------------
void THIS::makeRotationY(float fA)
{
	// c  0  s  0
	// 0  1  0  0
	//-s  0  c  0
	// 0  0  0  1
	float c=cosf(fA);
	float s=sinf(fA);
	fMatrix[0]=c; fMatrix[1]=0; fMatrix[2]=-s; 
	fMatrix[3]=0; fMatrix[4]=1; fMatrix[5]=0; 
	fMatrix[6]=s; fMatrix[7]=0; fMatrix[8]=c; 
}
//----------------------------------------------------------------------------------------------------------
void THIS::makeRotationZ(float fA)
{
	// c -s  0  0
	// s  c  0  0
	// 0  0  1  0
	// 0  0  0  1

	float c=cosf(fA);
	float s=sinf(fA);
	fMatrix[0]=c; fMatrix[1]=s; fMatrix[2]=0; 
	fMatrix[3]=-s; fMatrix[4]=c; fMatrix[5]=0; 
	fMatrix[6]=0; fMatrix[7]=0; fMatrix[8]=1; 
}
//----------------------------------------------------------------------------------------------------------
void THIS::makeScale(const Vector3D& vScale)
{
	fMatrix[0]=vScale[0]; fMatrix[1]=0; fMatrix[2]=0; 
	fMatrix[3]=0; fMatrix[4]=vScale[1]; fMatrix[5]=0; 
	fMatrix[6]=0; fMatrix[7]=0; fMatrix[8]=vScale[2]; 
}
//----------------------------------------------------------------------------------------------------------
CMat3 THIS::operator*(const CMat3& mat) const
{
	CMat3 r;
	float* r0=(float*)&r.fMatrix;
	float* m2=(float*)&fMatrix;
	float* m1=(float*)&mat.fMatrix;

	r0[0]=m1[0]*m2[0]+ m1[1]*m2[3]+ m1[2]*m2[6];
	r0[1]=m1[0]*m2[1]+ m1[1]*m2[4]+ m1[2]*m2[7];
	r0[2]=m1[0]*m2[2]+ m1[1]*m2[5]+ m1[2]*m2[8];
	
	r0[3]=m1[3]*m2[0]+ m1[4]*m2[3]+ m1[5]*m2[6];
	r0[4]=m1[3]*m2[1]+ m1[4]*m2[4]+ m1[5]*m2[7];
	r0[5]=m1[3]*m2[2]+ m1[4]*m2[5]+ m1[5]*m2[8];
	
	r0[6]=m1[6]*m2[0]+ m1[7]*m2[3]+ m1[8]*m2[6];
	r0[7]=m1[6]*m2[1]+ m1[7]*m2[4]+ m1[8]*m2[7];
	r0[8]=m1[6]*m2[2]+ m1[7]*m2[5]+ m1[8]*m2[8];
	

	return r;
}
//----------------------------------------------------------------------------------------------------------
CMat3 THIS::operator *(float f) const {
  CMat3 newMatrix;
  for (unsigned i = 0; i < 9; i++) {
      newMatrix[i] = fMatrix[i] * f;
  }
  return newMatrix;
}
//-------------------------------------------------------------------------------------------------------------------------------------------------------------------------

CMat3& THIS::operator *=(float f) {
  for (unsigned i = 0; i < 9; i++) {
      fMatrix[i] *= f;
  }
  return *this;
}
//----------------------------------------------------------------------------------------------------------
Vector3D THIS::operator *(const Vector3D& v) const {
  Vector3D temp;
  temp[0]=fMatrix[0]*v[0]+fMatrix[1]*v[1]+fMatrix[2]*v[2];
  temp[1]=fMatrix[3]*v[0]+fMatrix[4]*v[1]+fMatrix[5]*v[2];
  temp[2]=fMatrix[6]*v[0]+fMatrix[7]*v[1]+fMatrix[8]*v[2];
  return temp;
}
//----------------------------------------------------------------------------------------------------------
CMat3 THIS::operator +(const CMat3& mat) {
  CMat3 newMatrix;
  for (unsigned i = 0; i < 9; i++) {
      newMatrix[i] = mat.valueAt(i) + valueAt(i);
  }
  return newMatrix;
}

//----------------------------------------------------------------------------------------------------------
std::string THIS::toString() const
{
  std::ostringstream st;
  for (int i=0;i<3;i++)
  {
    for (int j=0;j<3;j++)
    {
      st<<m[j][i]<<" ";
    }
    st<<"\n";
  }
  return st.str();
}
//----------------------------------------------------------------------------------------------------------
float THIS::determinant() const {
  return fMatrix[0]*fMatrix[4]*fMatrix[8] + fMatrix[1]*fMatrix[5]*fMatrix[6 ] + fMatrix[2]*fMatrix[3]*fMatrix[7]
      - fMatrix[2]*fMatrix[4]*fMatrix[6] - fMatrix[1]*fMatrix[3]*fMatrix[8] - fMatrix[0]*fMatrix[5]*fMatrix[7];
}
//----------------------------------------------------------------------------------------------------------

#undef THIS
