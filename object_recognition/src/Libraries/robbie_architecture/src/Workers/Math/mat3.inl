/*******************************************************************************
 *  mat3.h
 *
 *  (C) 2007 AG Aktives Sehen <agas@uni-koblenz.de>
 *           Universitaet Koblenz-Landau
 *
 * Author: Frank Neuhaus, Susanne Maur
 *******************************************************************************/

#define THIS CMat3

//----------------------------------------------------------------------------------------------------------
inline THIS::CMat3()
{
	for(int i=0;i<9;i++) fMatrix[i]=0.0f;
}
//----------------------------------------------------------------------------------------------------------
inline THIS::CMat3( float xx, float xy, float xz, float yx, float yy, float yz, float zx, float zy, float zz )
{
  fMatrix[0] = xx;
  fMatrix[1] = xy;
  fMatrix[2] = xz;
  fMatrix[3] = yx;
  fMatrix[4] = yy;
  fMatrix[5] = yz;
  fMatrix[6] = zx;
  fMatrix[7] = zy;
  fMatrix[8] = zz;
}
//----------------------------------------------------------------------------------------------------------
inline float& THIS::operator [](const unsigned value) {
  return fMatrix[value];
}
//----------------------------------------------------------------------------------------------------------
inline float THIS::valueAt(unsigned i) const {
  return fMatrix[i];
}
//----------------------------------------------------------------------------------------------------------
inline void THIS::setValue(unsigned line, unsigned column, float value) {
  m[line][column] = value;
}
//----------------------------------------------------------------------------------------------------------

#undef THIS
