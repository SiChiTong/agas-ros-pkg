/**
 * @file    Mirror.h
 * @brief   Contains generic class Mirror.
 *
 * (c) 2006 AG Aktives Sehen <agas@uni-koblenz.de>, Universitaet Koblenz-Landau
 * $Id: $ 
 */

#ifndef MIRROR_H
#define MIRROR_H

#include "ImageToImageOperator.h"

// #include <cmath>
// #include <typeinfo>

namespace puma2 {

/**
 * @class   Mirror
 * @brief   Vertical and horizontal mirror for any image.
 * @author  Tobias Feldmann, Peter Decker
 * @date    Februar 2007
 */

template <class T> class Mirror : public ImageToImageOperator<T,T>
{
  private:
    bool mVertical;
    bool mHorizontal;

  public:

    /**
     * Do the real work of the operator
     * @param iImg The input image.
     * @param oImg The resulting outpout image.
     */
    virtual void apply(const T & iImg, T & oImg);

    /**
     * Default constructor.
     */
    Mirror(const bool iVertical = true, const bool iHorizontal=true);

    /**
     * Destructor
     */
    virtual ~Mirror();

    /**
     * @param iEnable  Enables vertical mirroring (default true)
     */
    void setVertical(const bool iEnable=true){
      mVertical=iEnable;
    };

    /**
     * @param iEnable  Enables horizontal mirroring (default true)
     */
    void setHorizontal(const bool iEnable=true){
      mHorizontal=iEnable;
    };

    /**
     * Returns the mirror status for the horizontal axis
     * @return true, if horizontal mirroring is enabled.
     */
    bool getHorizontalEnabled() const {
      return mHorizontal;
    };

    /**
     * Returns the mirror status for the vertical axis
     * @return true, if vertical mirroring is enabled.
     */
    bool getVerticalEnabled()  const {
      return mVertical;
    };
};

template <class T> Mirror<T>::Mirror(const bool iVertical,
                                     const bool iHorizontal){
      mVertical   = iVertical;
      mHorizontal = iHorizontal;
}

template <class T> Mirror<T>::~Mirror(){
}

template <class T> void Mirror<T>::apply(const T & iImg, T & oImg){

  // make sure iImg and oImg is not the same image
  if (iImg == oImg)
    throw "No inplace operator";

  // get the number of channels
  int numOfChannels = T::numberOfChannels() - 1;

  // start mirroring
  if ( mVertical == true && mHorizontal == true ) {
    for (int y = iImg.getHeight() - 1 ; y >= 0; --y){
      for (int x = iImg.getWidth() - 1 ; x >= 0; --x){
        for (int n = numOfChannels ; n >= 0; --n) {
          oImg.sample(iImg.getWidth()  - x - 1, iImg.getHeight() - y - 1, n) =
                      iImg.sample(x, y, n);
          }
      }
    }
  } else {
    if ( mVertical == true ) {
      for (int y = iImg.getHeight() - 1 ; y >= 0; --y){
        for (int x = iImg.getWidth() - 1 ; x >= 0; --x){
          for (int n = numOfChannels ; n >= 0; --n) {
            oImg.sample(x, iImg.getHeight() - y - 1, n) = iImg.sample(x, y, n);
          }
        }
      }
    }

    if ( mHorizontal == true ) {
      for (int y = iImg.getHeight() - 1 ; y >= 0; --y){
        for (int x = iImg.getWidth() - 1 ; x >= 0; --x){
          for (int n = numOfChannels ; n >= 0; --n) {
            oImg.sample(iImg.getWidth() - x - 1, y, n) = iImg.sample(x, y, n);
          }
        }
      }
    }

    if ( mVertical == false && mHorizontal == false ) {
      // oImg=iImg; // TODO DP -> m�ge das fixen

      // Workaround:
      for (int y = iImg.getHeight() - 1 ; y >= 0; --y){
        for (int x = iImg.getWidth() - 1 ; x >= 0; --x){
          for (int n = numOfChannels ; n >= 0; --n) {
            oImg.sample(x,y,n) = iImg.sample(x,y,n);
          }
        }
      }
    }
  }
}

}

#endif /* MIRROR_H */
