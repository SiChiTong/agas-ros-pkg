/*******************************************************************************
 *  ImageProperties.h
 *
 *  (C) 2007 AG Aktives Sehen <agas@uni-koblenz.de>
 *           Universitaet Koblenz-Landau
 *
 *  Additional information:
 *  $Id: $
 *******************************************************************************/

#ifndef ImageProperties_H
#define ImageProperties_H

#include <vector>

#include "Workers/Math/Point2D.h"

#include "Workers/KeyPointExtraction/KeyPoint.h"
#include "Workers/Puma2/HistogramUV.h"
#include "Workers/Puma2/ColorImageUV8.h"
#include "Workers/Puma2/GrayLevelImage8.h"
#include "Workers/Puma2/ImageMask.h"

#include "Workers/Math/Point2D.h"


/**
 * @class  ImageProperties
 * @brief  Extracts and holds features (histogram, keypoints) of an image
 * @author David Gossow (RX)
 */
class ImageProperties
{
  public:

    ImageProperties();

    /** @brief The constructor. Takes ownership of the given data */
    ImageProperties ( std::string name, puma2::GrayLevelImage8* imageY, puma2::ColorImageUV8* imageUV, ImageMask* imageMask=0 );

    ImageProperties ( const ImageProperties& other );

    /** @brief The destructor */
    ~ImageProperties();

    ImageProperties& operator= ( const ImageProperties& other );

    /// @brief data analysis. Unless the object was deserialized, you have to call these before using the getter functions.
    void applyMask();
    void traceOutline();
    void extractKeyPoints();
    void calcHistogram();

    /// @brief calls all extraction functions (applyMask, extractKeyPoints etc.)
    void calculateProperties();

    // GETTER FUNCTIONS

    std::string getName() const { return m_Name; }

    puma2::GrayLevelImage8* getImageY() const { return m_ImageY; }

    puma2::ColorImageUV8* getImageUV() const { return m_ImageUV; }

    puma2::GrayLevelImage8* getMaskedImageY() const { return m_MaskedImageY; }

    puma2::ColorImageUV8* getMaskedImageUV() const { return m_MaskedImageUV; }

    ImageMask* getImageMask() const { return m_ImageMask; }

    HistogramUV* getHistogram() const { return m_Histogram; }

    std::vector< KeyPoint >* getKeyPoints() const { return m_KeyPoints; }

    std::vector<Point2D>* getOutline() const { return m_Outline; }

    std::vector<Point2D> getBoundingBox() const;

    Point2D getCenter() const { return m_Center; }

    // TODO just remove?

//    /** @brief Serialize to stream */
//    void storer ( ExtendedOutStream& extStrm );

//    /** @brief Deserialize from stream */
//    ImageProperties ( ExtendedInStream& extStrm );

  private:

    void clear();
    void deleteAll();

    /// @brief base Data
    std::string m_Name;
    puma2::GrayLevelImage8* m_ImageY;
    puma2::ColorImageUV8* m_ImageUV;
    ImageMask* m_ImageMask;
    ImageMask* m_ImageMaskWithBorder;

    /// @brief extracted data
    puma2::GrayLevelImage8* m_MaskedImageY;
    puma2::ColorImageUV8* m_MaskedImageUV;
    std::vector< KeyPoint >* m_KeyPoints;
    HistogramUV* m_Histogram;
    std::vector<Point2D>* m_Outline;
    Point2D m_Center;

    int m_Border;
};

#endif
