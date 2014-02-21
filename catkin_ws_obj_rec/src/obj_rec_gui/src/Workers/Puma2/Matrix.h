#ifndef Matrix_H
#define Matrix_H
#include <sys/types.h>
#include <assert.h>
#include <iostream>
#include "Vector.h"

namespace puma2 {

/**
 * @class	Matrix
 * @brief	Represents a matrix template class.
 *
 * This template class offers basic matrix functions for image classes.
 * It is used in conjuction with the vector template class to provide a fast
 * storage for raw image data.
 */

template <class T>
  class Matrix {
    /**
	  * local typedef to provide give an alias for the
	  * template parameter T
	  */
  	typedef T ElemType;
  public:
    /**
	  * copy given matrix
	  */
    Matrix(const Matrix&);
    /**
	  * create matrix (default 0x0)
	  */
    explicit Matrix(int =0, int =0);
	/**
	  * create Matrix of given size and make it
	  * a submatrix of another matrix with offset
	  * @parm xo and @param yo .
	  */
    Matrix(int, int, Matrix*, int xo, int yo);
	/**
	  * delete matrix
	  */
    virtual ~Matrix();
	/**
	  * assign matrix
	  */
    void operator= (const Matrix&);
	/**
	  * assign a given value to each element
	  */
    void operator= (const T& v);
	/**
	  * return size of matrix
	  */
    int getWidth() const {return mXSize;}
	/**
	  * return size of matrix
	  */
    int getHeight() const {return mYSize;}
#if 1
	/**
	  * resize a zero sized matrix to the given size
	  */
	void resize(int x, int y) ;
	/**
	  * resize a zero sized matrix to the given size making it a
	  * submatrix of the matrix given as parameter with
	  * offset @parm xo and @param yo .
	  */
	void resize(int x, int y, Matrix* m, int xo, int yo) ;
#else
	/**
	  * resize a zero sized matrix to the given size making it a
	  * submatrix of the matrix given as parameter with
	  * offset @parm xo and @param yo .
	  */
	void resize(int x, int y, Matrix* m = 0, int xo = 0, int yo = 0) ;
#endif
    /**
	  * read / write access by operator []
	  */
    inline Vector<T>& operator[] (int);
    /**
	  * read access by operator []
	  */
    inline const Vector<T>& operator[] (int) const;
    /**
	  * provide direct access to matrix (read / write)
	  */
    operator T**(){ return matrix; }
    /**
	  * provide direct access to matrix (read only)
	  */
    operator const T**() const { return (const T**) matrix; }
    /**
	  * returns true if matrix is a submatrix of another matrix
	  */
    bool isSubMatrix() const { return master != NULL; }
  private:
    int mXSize;
    int mYSize;
    T ** matrix;
    Vector<T> ** vp;
    Matrix<T> * master;
    int rcount;
};

template <class T>
  inline Vector<T>& Matrix<T>::operator[] (int i){
    assert((i>=0) && (i<mYSize));
    return *vp[i];
}


#if 1
// ????
// should we unite the two resize functions to
// void Matrix<T>::resize(int x, int y, Matrix* m = 0, int xo = 0, int yo = 0)
// ????

template <class T>
  void Matrix<T>::resize(int x, int y, Matrix* m, int xo, int yo)
{
    if ((getWidth() != 0) || (getHeight() != 0))
      throw "cannot resize allocated matrix";
    mXSize = x;
    mYSize = y;
    master = m;
    m->rcount++;
    rcount = 0;
    T ** array = m->matrix;
    vp = new Vector<T>*[y];
    matrix = new T*[y];
    for (int i = 0; i < y; ++i) {
      T* cp = matrix[i] = & (array[i+yo][xo]);
      vp[i] = new Vector<T>(cp,x);
    }
}

template <class T>
  void Matrix<T>::resize(int x, int y)
{
    if ((getWidth() == x) && (getHeight() == y))
      return;	// redundant call -- just ignore it.
    if ((getWidth() != 0) || (getHeight() != 0))
      throw "cannot resize allocated matrix";
    mXSize = x;
    mYSize = y;
    rcount = 0;
    master = NULL;
    T * array = new T[x*y];
    vp = new Vector<T>*[y];
    matrix = new T*[y];
    for (int i = 0; i < y; ++i) {
      T* cp = matrix[i] = & (array[i*x]);
      vp[i] = new Vector<T>(cp,x);
    }
}

#else

// not correct, yet
template <class T>
  void Matrix<T>::resize(int x, int y, Matrix* m, int xo, int yo)
{
    if ((getWidth() == x) && (getHeight() == y))
      return;	// redundant call -- just ignore it.
    if ((getWidth() != 0) || (getHeight() != 0))
      throw "cannot resize allocated matrix";
    mXSize = x;
    mYSize = y;
    master = m;
    rcount = 0;
    T ** array;
	if (master == NULL) {
	   assert(xo == 0);
	   assert(yo == 0);
       array = new T[x*y];
	} else {
       m->rcount++;
       array = m->matrix;
    }
    vp = new Vector<T>*[y];
    matrix = new T*[y];
    for (int i = 0; i < y; ++i) {
      T* cp = matrix[i] = & (array[i+yo][xo]);
      vp[i] = new Vector<T>(cp,x);
    }
}
#endif

template <class T>
  const Vector<T>& Matrix<T>::operator[](int i) const{
    assert((i>=0) && (i<mYSize));
    return *vp[i];
}

template <class T>
  Matrix<T> operator*(const Matrix<T>&, const Matrix<T>&);

template <class T>
  Vector<T> operator*(const Matrix<T>&, const Vector<T>&);

template <class T>
  Vector<T> operator*(const T&, const Vector<T>&);

template <class T>
  T operator*(const Vector<T>&, const Vector<T>&);

template <class T>
  Matrix<T>::Matrix(int x, int y){
    mXSize = x;
    mYSize = y;
    rcount = 0;
    master = NULL;
    T * array = new T[x*y];
    vp = new Vector<T>*[y];
    matrix = new T*[y];
    for (int i = 0; i < y; ++i) {
      T* cp = matrix[i] = & (array[i*x]);
      vp[i] = new Vector<T>(cp,x);
    }
}

template <class T>
  Matrix<T>::~Matrix(){
    if (rcount > 0) throw "try to delete referenced matrix";
    if (master == NULL) {
      if (mYSize>0) delete [] matrix[0];
    } else {
      master->rcount--;
    }
	// should also work for 0 x 0 matrices

    delete[] matrix;
    for (int i = 0; i < mYSize; ++i) delete vp[i];
    delete[] vp;
}

template <class T>
  Matrix<T>::Matrix(int x, int y, Matrix* m, int xo, int yo)
{
    mXSize = x;
    mYSize = y;
    master = m;
    m->rcount++;
    rcount = 0;
    T ** array = m->matrix;
    vp = new Vector<T>*[y];
    matrix = new T*[y];
    for (int i = 0; i < y; ++i) {
      T* cp = matrix[i] = & (array[i+yo][xo]);
      vp[i] = new Vector<T>(cp,x);
    }
}

template <class T>
  Matrix<T>::Matrix(const Matrix& m)
{
    u_int x = mXSize = m.mXSize;
    u_int y = mYSize = m.mYSize;
    master = NULL;
    rcount = 0;
    T * array = new T[x*y];
    matrix = new T*[y];
    vp = new Vector<T>*[y];
    for (u_int i = 0; i < y; ++i) {
      T* cp = matrix[i] = & (array[i*x]);
      vp[i] = new Vector<T>(cp,x);
      for (u_int j = 0; j < x; ++j)
        cp[j] = m.matrix[i][j];
        //Vector<T>::copy(m.matrix[i][j],cp[j]);
    }
}

template <class T>
  void Matrix<T>::operator= (const Matrix& m){
    //assert(mYSize == m.mYSize && mXSize == m.mXSize);
		resize( m.mXSize, m.mYSize);
    for (int i = 0; i < mYSize; ++i){
      for (int j = 0; j < mXSize; ++j){
        matrix[i][j] = m.matrix[i][j];
      }
    }
}

}

#endif
