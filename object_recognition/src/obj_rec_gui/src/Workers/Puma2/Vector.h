#ifndef Vector_H
#define Vector_H
#include <cassert>
#include <iostream>

namespace puma2 {

template <class T> 
  class Matrix;  

/**
 * @class	Vector
 * @brief	Represents a vector template class.
 *
 * This class provides a template for vectors.
 */

template <class T> 
  class Vector { 
  public:
    explicit Vector(int = 0);       
    Vector(const Vector&);          
    virtual ~Vector();                      
    int length() const {return size;}           
    inline T& operator[] (int i) ;              
    inline const T& operator[] (int i) const;   
    void operator=(const Vector &);             
    void operator=(T v);                        
    operator T* () { return vec; }              
    operator const T* () const { return vec; }  
	static inline void copy(const T&, T&);
  private:
    int size;
    T *vec;
    bool allocated;
    friend class Matrix<T>;  
    Vector(T*,int);          
};                         


template <class T> inline void Vector<T>::copy(const T & f, T& t) { t = f; }

template <class T> 
  Vector<T>::Vector(const Vector & v) : size(v.size){
    allocated = true;       
    vec = new T[size];      
    for (int i = 0; i<size; ++i){
      copy(v.vec[i],vec[i]); 
    }
}

template <class T> 
  Vector<T>::~Vector(){ 
    if (allocated){
      delete [] vec; 
    }
} 

template <class T> 
  Vector<T>::Vector(int s) : size(s){
    vec = new T[s]; 
    allocated = true;  
}

template <class T> 
  Vector<T>::Vector(T* m, int s){ 
    allocated = false; 
    vec = m; 
    size = s;  
}

template <class T> 
  void Vector<T>::operator=(const Vector & v){
    assert(v.size == size);      
    for (int i = 0; i<size; ++i){
      // vec[i]=v.vec[i];
      copy(v.vec[i],vec[i]); 
    }   
}
template <class T> 
  inline const T& Vector<T>::operator[] (int i) const{ 
    assert((i<size) && (i>=0)); 
    return vec[i]; 
}

template <class T> 
  inline T& Vector<T>::operator[] (int i){ 
    assert(i<size); 
    assert(i>=0);
    return vec[i]; 
}

}

#endif
