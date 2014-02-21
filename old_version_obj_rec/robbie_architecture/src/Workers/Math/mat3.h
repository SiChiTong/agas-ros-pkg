/*******************************************************************************
 *  mat3.h
 *
 *  (C) 2007 AG Aktives Sehen <agas@uni-koblenz.de>
 *           Universitaet Koblenz-Landau
 *
 * Author: Frank Neuhaus, Susanne Maur
 *******************************************************************************/
#ifndef MAT3_H
#define MAT3_H

#include <iostream>
#include <sstream>
#include <assert.h>
#include "Vector3D.h"

class CMat3 {
  public:
	  CMat3();
    CMat3( float xx, float xy, float xz, float yx, float yy, float yz, float zx, float zy, float zz );
	  ~CMat3();
  
    /** overwritten operator**/
	  CMat3 operator *(const CMat3 &mat) const;  
    CMat3 operator *(float f) const;  
    CMat3& operator *=(float f);  
    Vector3D operator *(const Vector3D& v) const;      
    float& operator [](const unsigned value);    
    CMat3 operator +(const CMat3& mat1);
    
    /** @return value at position **/
    float valueAt(unsigned i) const;
   
    /** set value at position **/
    void setValue(unsigned line, unsigned column, float value);
  
    /** @return determinant of matrix **/
    float determinant() const;
      
    /** transpose matrix **/
	  void transpose();
    /** reverse matrix **/
    void reverse();

    /** create identity matrix **/
	  void loadIdentity();
  
    /** create rotation matrix **/
	  void makeRotationX(float fA);
	  void makeRotationY(float fA);
	  void makeRotationZ(float fA);

    /** create scale matrix **/
	  void makeScale(const Vector3D& vScale);
	  //void BuildRPY(float fRoll, float fPitch, float fYaw);
      
	  std::string toString() const;

  private:
	  union
	  {
		  float fMatrix[9];
		  float m[3][3];
		  struct
		  {
			  float xx, xy, xz;
			  float yx, yy, yz;
			  float zx, zy, zz;
		  };
	  };
};

#include "mat3.inl"

#endif

