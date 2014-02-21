/**
 * @file    MultiElementImage.h
 * @brief   Basic template for image data storage.
 *
 * (C) 2007 AG Aktives Sehen <agas@uni-koblenz.de>, Universitaet Koblenz-Landau
 * $Id: $
 */

#ifndef MultiElementImageTemplate_H
#define MultiElementImageTemplate_H

#include "BaseImageTemplate.h"

namespace puma2
{

//Class acts exactly like an array, only that it supports the assignment operator.
template <class T, int N> class MultiElementPixel
{
  public:

    T data[N];

    inline operator T*() { return data; }
    inline operator const T*() const { return data; }

    inline MultiElementPixel< T, N >& operator=( MultiElementPixel< T, N > other ) {
      for ( unsigned i=0; i < N; i++ )
      {
        data[i] = other.data[i];
      }
      return *this;
    }
};

template <class T, int N> class MultiElementImageIter;
template <class T, int N> class MultiElementImageIterChannel;

/**
 * @class   MultiElementImage
 * @brief   N-channel image template
 *
 * (detailed descripton forthcomming)
 *
 * @see     TBaseImg
 * @author  Dietrich Paulus <paulus@uni-koblenz.de>
 * @date    January 2007
 */

template <class T, int N> class MultiElementImage : public TBaseImg < MultiElementPixel< T, N > >
{
    friend class MultiElementImageIter<T, N>;
    friend class MultiElementImageIterChannel<T, N>;
  public:
    typedef T ElementType;     /// element type, type of samples a pixel is composed of
    typedef MultiElementPixel< T, N > PixelType;    /// pixel type
  public:
    static int numberOfChannels() { return N; }
    MultiElementImage ( int width = 0, int height = 0 ) : TBaseImg < MultiElementPixel< T, N > > ( width, height ) {}
    /// subimage constructor
    MultiElementImage ( int width, int height, MultiElementImage* master, int xOffset, int yOffset )
        : TBaseImg < MultiElementPixel< T, N > > ( width, height, master, xOffset, yOffset ) {}
    const ElementType sample ( int x, int y, int n ) const {
      assert ( n < N );
      return TBaseImg < MultiElementPixel< T, N > >::c0[y][x][n];
    }
    ElementType& sample ( int x, int y, int n ) {
      assert ( n < N );
      return TBaseImg < MultiElementPixel< T, N > >::c0[y][x][n];
    }

    /// initialize iterator to start of image data - will point to pixels
    MultiElementImageIter<T, N> begin() {
      MultiElementImageIter<T, N> iter;
      iter.start = & this->c0[0][0];
      iter.end = iter.start + this->getHeight() * this->getWidth();
      iter.curr = iter.start;
      // cerr << "Begin Curr: " << reinterpret_cast<unsigned long>(iter.curr) << endl;
      return iter;
    }
    /// initialize iterator to start of image data - will point to channel @param n
    MultiElementImageIterChannel<T, N> begin ( int n ) {
      MultiElementImageIterChannel<T, N> iter;
      iter.start = & this->c0[0][0];
      iter.end = iter.start + this->getHeight() * this->getWidth();
      iter.curr = iter.start;
      // cerr << "Begin Curr: " << reinterpret_cast<unsigned long>(iter.curr) << endl;
      iter.c = n;
      return iter;
    }
    /// initialize iterator to end of image data - will point to pixels (multi-channel)
    MultiElementImageIter<T, N> end() {
      MultiElementImageIter<T, N> iter;
      iter.start = & this->c0[0][0];
      iter.end = iter.start + this->getHeight() * this->getWidth();
      iter.curr = iter.end;
      // cerr << "End Curr: " << reinterpret_cast<unsigned long>(iter.curr) << endl;
      return iter;
    }
    /// initialize iterator to end of image data - will point to channel @param n
    MultiElementImageIterChannel<T, N> end ( int n ) {
      MultiElementImageIterChannel<T, N> iter;
      iter.start = & this->c0[0][0];
      iter.end = iter.start + this->getHeight() * this->getWidth();
      iter.curr = iter.end;
      // cerr << "End Curr: " << reinterpret_cast<unsigned long>(iter.curr) << endl;
      iter.c = n;
      return iter;
    }
    typedef MultiElementImageIter<T, N> PixelIterator;           //< Simplify notation
    typedef MultiElementImageIterChannel<T, N> ChannelIterator;  //< Simplify notation
};

/// Iterator class that will iteratate over all pixels (multi-channel)
template <class T, int N> class MultiElementImageIter
{
    friend class MultiElementImage<T, N>;
    typedef T P[N];   ///< simplify notation: P is pixel type (multi-channel)
    P * start;
    P * curr;
    P * end;
  public:
    bool operator== ( const MultiElementImageIter& i ) const {
      // cerr << "Compare: " << reinterpret_cast<unsigned long>(i.curr) << " to "
      //                    << reinterpret_cast<unsigned long>(curr) << endl;
      return i.curr == curr;
    }
    bool operator!= ( const MultiElementImageIter& i ) const
    { return ! operator== ( i ); }
    /// in contrast to STL, the operators are not const to allow write access to pixels
    MultiElementImageIter& operator++ ()
    { curr++; return *this; }
    MultiElementImageIter  operator++ ( int )
    { MultiElementImageIter old = *this; ++curr; return old; }
    MultiElementImageIter& operator-- ()
    { curr--; return *this; }
    MultiElementImageIter  operator-- ( int )
    { MultiElementImageIter old = *this; --curr; return old; }
    P& operator * ()
    { return * curr; }
    P * operator-> ()
    { return curr; }
    operator P* ()
    { return curr; }
};

/// Iterator class that will access channels in a n-channel image
template <class T, int N> class MultiElementImageIterChannel
{
    friend class MultiElementImage<T, N>;
    typedef T P[N];
    P * start;
    P * curr;
    P * end;
    int c;
  public:
    bool operator== ( const MultiElementImageIterChannel& i ) const
    { return i.curr == curr; }
    bool operator!= ( const MultiElementImageIterChannel& i ) const
    { return ! operator== ( i ); }
    MultiElementImageIterChannel& operator++ ()
    { curr++; return *this; }
    MultiElementImageIterChannel  operator++ ( int )
    { MultiElementImageIterChannel old = *this; ++curr; return old; }
    MultiElementImageIterChannel& operator-- ()
    { curr--; return *this; }
    MultiElementImageIterChannel  operator-- ( int )
    { MultiElementImageIterChannel old = *this; --curr; return old; }
    T& operator * ()
    { return * curr[c]; }
    T ** operator-> ()
    { return curr[c]; }
    operator P** ()
    { return curr[c]; }
};

}

#endif
