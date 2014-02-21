#ifndef MEDIAN_FILTER_H
#define MEDIAN_FILTER_H

#include "ImageToImageOperator.h"
#include "PumaException.h"

namespace puma2 {

/**
 * @class MedianFilter
 * @brief This class implements a basic median filter for images.
 *
 * @author Richard Arndt
 * @author Stephan Wirth
 *
 * This class implements a basic median filter for single channel images.
 * If it is used for multi-channel images, it will be applied per channel and
 * a warning will be produced. The median filter will only be applied to image
 * elements where the filter window fits completely into the image, i.e. if the
 * window has width and height of five, the first and the last two rows and colums
 * will not be filtered.
 *
 */
template <class T>
class MedianFilter : public ImageToImageOperator<T,T> {
  public:

    /**
     * Constructor. Sets width and height of the median filter window.
     * @param windowWidth Width of the window of the median filter
     * @param windowHeight Height of the window of the median filter
     */
    MedianFilter(int windowWidth, int windowHeight);

    /**
     * Constructor for sqare windows. Sets height and width of the median
     * filter window to windowSize.
     * @param windowSize Size of the median filter window (width and height)
     */
    MedianFilter(int windowSize);

    /**
     * Destructor deletes dynamically allocated memory.
     */
    virtual ~MedianFilter();

    /**
     * Do the real work of the operator
     */
	  virtual void apply(const T& iImg, T& oImg);

    /**
     * @return Width of the median filter window
     */
    int getWindowWidth() const { return mWindowWidth; }

    /**
     * @return Height of the median filter window
     */
    int getWindowHeight() const { return mWindowHeight; }

  private:

    /**
     * Computes the helpers for the apply() function. This function is called in the constructors.
     * The members mHalfWindowWidth, mHalfWindowHeight, mNumWindowElements and mMedianIndex
     * will be computed and the memory for mWindowElements will be allocated.
     */
    void computeHelpers();

    /**
     * Stores the width of the filter window
     */
    int mWindowWidth;

    /**
     * Stores the height of the filter window
     */
    int mWindowHeight;

    /**
     * Stores (mWindowWidth - 1) / 2
     */
    int mHalfWindowWidth;

    /**
     * Stores (mWindowHeight - 1) / 2
     */
    int mHalfWindowHeight;

    /**
     * Stores the number of elements in the window (mWindowHeight * mWindowWidth).
     */
    int mNumWindowElements;

    /**
     * Stores the window elements.
     */
    typename T::ElementType* mWindowElements;

    /**
     * Stores the index of the median in mWindowElements (mNumWindowElements / 2)
     */
    int mMedianIndex;
};


template <class T>
MedianFilter<T>::MedianFilter(int windowSize) {
  if (windowSize < 1 || windowSize % 2 == 0) {
    std::string msg("Illegal window size for median filter (has to be odd and positive)");
    throw PumaException(PumaException::intolerable, msg);
  }

  if (T::numberOfChannels() > 1) {
    //TODO throw warning
  }

  mWindowHeight = windowSize;
  mWindowWidth = windowSize;
  computeHelpers();
}


template <class T>
MedianFilter<T>::MedianFilter(int windowWidth, int windowHeight) {
  if (windowWidth < 1 || windowWidth % 2 == 0) {
    std::string msg("Illegal window width for median filter (has to be odd and positive)");
    throw PumaException(PumaException::intolerable, msg);
  }
  if (windowHeight < 1 || windowHeight % 2 == 0) {
    std::string msg("Illegal window height for median filter (has to be odd and positive)");
    throw PumaException(PumaException::intolerable, msg);
  }

  if (T::numberOfChannels() > 1) {
    //TODO throw warning
  }

  mWindowHeight = windowHeight;
  mWindowWidth = windowWidth;
  computeHelpers();
}

template <class T>
void MedianFilter<T>::computeHelpers() {
  mHalfWindowHeight = (mWindowHeight - 1) / 2;
  mHalfWindowWidth = (mWindowWidth - 1) / 2;
  mNumWindowElements = mWindowWidth * mWindowHeight;
  mWindowElements = new typename T::ElementType[mNumWindowElements];
  mMedianIndex = (mNumWindowElements - 1) / 2;
}

template <class T>
MedianFilter<T>::~MedianFilter() { 
  delete mWindowElements;
};


// NB: 
// - This is the first working implementation
// - much fuster algorithmes for computing the median exist
//   wich are even (almost) independent of the window size.
//   These algorithms will be implemented later

template <class T>
void MedianFilter<T>::apply(const T& iImg, T& oImg) {
  
  int minX = mHalfWindowWidth;
  int maxX = oImg.getWidth() - mHalfWindowWidth - 1;
  int minY = mHalfWindowHeight;
  int maxY = oImg.getHeight() - mHalfWindowHeight - 1;
  int numChannelsM1 = T::numberOfChannels() - 1;

  for (int y = minY; y <= maxY; y++) {
    for (int x = minX; x <= maxX; x++) {
      for (int n = 0; n <= numChannelsM1; n++) {
        int listIndex = 0;
        for (int wy = -mHalfWindowHeight; wy <= mHalfWindowHeight; wy++) {
          for (int wx = -mHalfWindowWidth; wx <= mHalfWindowWidth; wx++) {
            mWindowElements[listIndex++] = iImg.sample(x + wx, y + wy, n);
          }
        }
        // bubblesort mWindowElements
        for (int k = 1; k <= mNumWindowElements - 1; k++) {
          for (int l = 0; l < mNumWindowElements - k; l++) {
            if (mWindowElements[l] > mWindowElements[l + 1]) {
              typename T::ElementType temp = mWindowElements[l];
              mWindowElements[l] = mWindowElements[l + 1];
              mWindowElements[l + 1] = temp;
            }
          }
        }
        oImg.sample(x, y, n) = mWindowElements[mMedianIndex];
      }
    }
  }
}

}

#endif
