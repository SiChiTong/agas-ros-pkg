/*******************************************************************************
 *  HistogramUV.h
 *
 *  (C) 2007 AG Aktives Sehen <agas@uni-koblenz.de>
 *           Universitaet Koblenz-Landau
 *
 *  Additional information:
 *  $Id: HistogramUV.h 24221 2008-04-12 12:13:04Z dgossow $
 *******************************************************************************/


#ifndef UVHISTOGRAM_H
#define UVHISTOGRAM_H

#include "../../Workers/Puma2/ColorImageRGB8.h"
#include "../../Workers/Puma2/GrayLevelImage8.h"
#include "Matrix.h"

#include "../../Workers/Puma2/ColorImageUV8.h"
#include "../../Workers/Puma2/ImageMask.h"

// #include "Architecture/Config/Config.h" // TODO

#include "Workers/Math/Box2D.h"


using namespace puma2;


/** @note Either float or double */
typedef double EntryT;

/**
 * @class  HistogramUV
 * @brief  Fast UV histogram supporting back projection
 * @note   Representation of colors by U/Y and V/Y quotients for luminance independency
 * @author Patric Lambrecht (RX), David Gossow (RX)
 */
class HistogramUV
{

  public:

    /**
     * @brief The constructor
     * @param binSize determines how much neighbored u and v values are treated as identical
     */
    HistogramUV ( unsigned binSize );

    HistogramUV( );

    /** @brief Copy constructor */
    HistogramUV ( const HistogramUV& other );

    /** @brief The destructor */
    ~HistogramUV();

    /** @brief Assign operator */
    HistogramUV& operator= ( const HistogramUV& other );

    /** @brief Set all histogram values to 0 */
    void clear();

    /**
     * @brief Add the values of an image within the bounding box to the histogram
     * @param imageUV,imageY images to be added
     * @param minY,maxY range of valid Luminance values. Pixels with an Y value outside of these boundaries are ignored.
     * @param bBox bounding box in the image
     */
    void addImage ( ColorImageUV8 &imageUV, GrayLevelImage8 &graimageY, Box2D<int> bBox, unsigned minY=1, unsigned maxY=254 );

    /**
      * @brief Add the values of an image to the histogram
      * @see addImage (above)
      */
    void addImage ( ColorImageUV8 &imageUV, GrayLevelImage8 &graimageY, unsigned minY=1, unsigned maxY=254 );

    /** @brief Substract a constant value, clipping at 0 */
    void substract ( float value );

    /** @brief Entry-wise division by the values of divident */
    void divideBy ( const HistogramUV& divident );

    void add ( const HistogramUV& other );

    /** @brief Normalize the histogram to max=1 */
    void normalizeMax();

    /** @brief Sets all entries below thresholdFactor*meanValue to zero and applies a dilation filter */
    void applyThreshold ( float thresholdFactor=1.0, float dilationRadius=0.0 );

    /** @brief Erase the center of the histogram with the given range */
    void clearCenter ( unsigned int range );

    /** @return A relative range for erasing the histogram center */
    float getDeviation();

    /** @brief Calculate a binary image mask. If the color of the pixel is contained in the histogram, the mask is set to 255, 0 otherwise. */
    ImageMask* getMask ( ColorImageUV8 &imageUV, GrayLevelImage8 &imageY, unsigned minY=20, unsigned maxY=235 ) const;

    /** @return the maximal value */
    EntryT getMaxValue() const;

    /** @return the mean value */
    EntryT getMeanValue() const;

    /** @brief Calculates an RGB image representing the histogram values
     *  @param imageRGB output image
     *  @param exponent if not 1, perform non-linear value scaling
    */
    void getImage ( ColorImageRGB8& imageRGB, float exponent=0.5 ) const;

    /** @brief Compares two histogram and return the mean squared difference */
    EntryT distance ( HistogramUV& other ) const;

    /** @return matrix containing the histogram data */
    const Matrix<EntryT>& getMatrix() const { return m_Matrix; }

    /** @return matrix containing the histogram data */
    const EntryT* getData() const { return m_Data; }

    unsigned getBinSize() const { return m_BinSize; }

    // TODO kann wahrscheinlich einfach weg
//    /** @brief Deserialize from stream */
//    HistogramUV ( ExtendedInStream& extStrm );
//    /** @brief Serialize to stream */
//    void storer ( ExtendedOutStream& extStrm );

    /** @brief Print object information */
    void printOn ( std::ostream& strm );

  private:

    bool checkInit() const;

    /** Convert to internal u/v representation */
    inline int correct ( int val, int y ) const;

    /**
     * @brief Size of one bin
     * @see constructor
     */
    unsigned m_BinSize;

    /** @brief Number of bins per channel */
    unsigned m_NumBins;

    /** @brief Holds the histogram data */
    Matrix<EntryT> m_Matrix;

    /** @brief Pointer to the raw data of m_Matrix */
    EntryT* m_Data;

    /** @brief Byte length of the histogram data */
    unsigned m_DataLength;

};



#endif
