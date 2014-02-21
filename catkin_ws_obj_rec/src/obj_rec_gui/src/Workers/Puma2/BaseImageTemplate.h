/**
 * @file    BaseImageTemplate.h
 * @brief   Basic template for image data storage.
 *
 * (C) 2007 AG Aktives Sehen <agas@uni-koblenz.de>, Universitaet Koblenz-Landau
 * $Id: $
 */

#ifndef BaseImageTemplate_H
#define BaseImageTemplate_H

#include "Matrix.h"
#include "Vector.h"

#include <cstring>
#include <stdio.h>

namespace puma2
{

/**
 * @class   TBaseImg
 * @brief   abstract Template base for all image templates
 *
 * (detailed descripton forthcomming)
 *
 * @see     ImageCore
 * @author  Dietrich Paulus <paulus@uni-koblenz.de>
 * @date    January 2007
 */

template <class T> class TBaseImg
{
  public:
    /**
     * Retuns the vertical size of the image.
     * @return Vertical size of the image
     */
    unsigned getHeight() const { return c0.getHeight(); }

    /**
     * Retuns the horizontal size of the image.
     * @return Horizontal size of the image
     */
    unsigned getWidth() const { return c0.getWidth(); }

    /// local typedef for Pixel
    typedef T Pixel;

    /// access operators for data rows (const)
    const Vector<T> & operator[] ( int i ) const { return c0[i]; }

    /// access operators for data rows (mutable)
    Vector<T> & operator[] ( int i ) { return c0[i]; }

    /// explicit definition of assignment operator
    void operator= ( const TBaseImg & o ) {
      this->c0 = o.c0;
    }
    /// cast operator
    operator T**() { return c0; }
    /// cast operator for constant object
    operator const T**() const { return ( const T** ) c0; }
    /// verbose cast via method call
    T** unsafeRowPointerArray() { return c0; }
    /// verbose cast via method call
    const T** unsafeRowPointerArray() const { return ( const T ** ) c0; }

    /// delegate resize operation to matrix
    void resize ( int width, int height ) { c0.resize ( width, height ); }

    /// delegate resize operation to matrix to create a subimage
    void resize ( int width, int height, TBaseImg* master, int xOffset, int yOffset ) {
      c0.resize ( width, height, & ( master->c0 ), xOffset, yOffset );
    }

    /// check if image is a sub image
    bool isSubImage() const { return c0.isSubMatrix(); }

    void readFromFile(const char * fileName)
    {
      char buffer [1024];
      sprintf (buffer,"Try to read from %s -- not implemeted for current class", fileName);
      throw buffer;
    }

    void writeToFile(const char * fileName) const
    {
      char buffer [1024];
      sprintf (buffer,"Try to write to %s -- not implemeted for current class", fileName);
      throw buffer;
    }

    void setupImageBaseVariables ()
    {
      mValueRangeMinimum = this->getElementTypeMinimum();
      mValueRangeMaximum = this->getElementTypeMaximum();
    }

    /**
     * Tell about the minimal value which can be stored by an element (a sample)
     * which is the base type of this image class.lementTypeM
     * (This should be some kind of constant, individually set for each
     *  subclass, but C++ allows such overloading only for functions,
     *  so we have to implement it as a constant function.)
     */
    virtual double getElementTypeMinimum() const { return 0; };

    /**
     * Tell about the maximal value which can be stored by an element (a sample)
     * which is the base type of this image class.
     * (see getElementTypeMaximum().)
     */
    virtual double getElementTypeMaximum() const { return 0; };

    /** set mValueRangeMinimum to value
     * @param[in] value new minimum */
    void setValueRangeMinimum(double value) { mValueRangeMinimum = value; };
    /** set mValueRangeMaximum to value
     * @param[in] value new maximum */
    void setValueRangeMaximum(double value) { mValueRangeMaximum = value; };
    /** get mValueRangeMinimum */
    double getValueRangeMinimum() const { return mValueRangeMinimum; };
    /** get mValueRangeMaximum */
    double getValueRangeMaximum() const { return mValueRangeMaximum; };

    // all constructors are protected!
  protected:
    /// image matrix (raw data)
    Matrix<T> c0;   //< matrix for all image types

    /// also works as default constructor
    TBaseImg ( int width = 0, int height = 0 )
        : c0 ( width, height ) {}

    /// subimage constructor
    TBaseImg ( int width, int height, TBaseImg* master, int xOffset, int yOffset )
        : c0 ( width, height, & ( master->c0 ), xOffset, yOffset ) {}

    /**
     * Holds the intended minimum value any sample of this image should have.
     * This is not the current minimal value of all samples, but the value
     * according to the range expected, like an 8-bit gray level image usually
     * has an expected range from 0 (mValueRangeMinimum) to 255 (mValueRangeMaximum),
     * whereas a 4-bit image has a range of 0..15, 12-bit has 0..4095 and so on.
     */
    double mValueRangeMinimum;

    /**
     * Holds the intended maximum value any sample of this image should have.
     * This is not the current minimal value of all samples, but the value
     * according to the range expected, like an 8-bit gray level image usually
     * has an expected range from 0 (mValueRangeMinimum) to 255 (mValueRangeMaximum),
     * whereas a 4-bit image has a range of 0..15, 12-bit has 0..4095 and so on.
     */
    double mValueRangeMaximum;
};

}

#endif
