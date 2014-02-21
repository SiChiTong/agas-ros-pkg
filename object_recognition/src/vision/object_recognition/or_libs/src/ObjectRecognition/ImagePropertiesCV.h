/*******************************************************************************
 *  ImagePropertiesCV.h
 *
 *  (C) 2007 AG Aktives Sehen <agas@uni-koblenz.de>
 *           Universitaet Koblenz-Landau
 *
 *  Additional information:
 *  $Id: $
 *******************************************************************************/

#ifndef ImagePropertiesCV_H
#define ImagePropertiesCV_H

#include <vector>
#include <opencv2/highgui/highgui.hpp>
#include <ros/ros.h>

#include "../KeyPointExtraction/KeyPoint.h"
//#include "Workers/ImageHelpers/ImageMaskCV.h"
#include "../KeyPointExtraction/ImageMaskCV.h"

#include "Workers/Math/Point2D.h"

// include headers that implement an archive in binary format
#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/text_oarchive.hpp>

#include "Architecture/Serializer/cvmat_serialization.h"

/**
 * @class  ImageProperties
 * @brief  Extracts and holds features (histogram, keypoints) of an image
 * @author David Gossow (RX), Viktor Seib (R20)
 */
class ImagePropertiesCV
{
public:

    ImagePropertiesCV();

    /** @brief The constructor. Takes ownership of the given data */
    ImagePropertiesCV ( std::string name, cv::Mat* imageY, cv::Mat* imageUV, ImageMaskCV* imageMask=0 );

    ImagePropertiesCV ( const ImagePropertiesCV& other );

    /** @brief The destructor */
    ~ImagePropertiesCV();

    ImagePropertiesCV& operator= ( const ImagePropertiesCV& other );

    /// @brief data analysis. Unless the object was deserialized, you have to call these before using the getter functions.
    void applyMask();
    void traceOutline();
    void extractKeyPoints();

    /// @brief calls all extraction functions (applyMask, extractKeyPoints etc.)
    void calculateProperties();

    // GETTER FUNCTIONS

    std::string getName() const { return m_Name; }

    cv::Mat* getImageY() const { return m_ImageY; }

    cv::Mat* getImageUV() const { return m_ImageUV; }

    cv::Mat* getMaskedImageY() const { return m_MaskedImageY; }

    cv::Mat* getMaskedImageUV() const { return m_MaskedImageUV; }

    ImageMaskCV* getImageMask() const { return m_ImageMask; }

    std::vector< KeyPoint >* getKeyPoints() const { return m_KeyPoints; }

    std::vector<Point2D>* getOutline() const { return m_Outline; }

    std::vector<Point2D> getBoundingBox() const;

    Point2D getCenter() const { return m_Center; }

    // SERIALIZATION

    template<class Archive>
    void save(Archive & ar, const unsigned int version) const
    {
        unsigned x = 12;
        ar & x;
        ar & m_Name;

        ar & m_ImageY;
        ar & m_ImageUV;

        if ( m_ImageMask )
        {
            bool b = true;
            ar & b;
            int width = m_ImageMask->getWidth();
            ar & width;
            int height = m_ImageMask->getHeight();
            ar & height;

            ar & boost::serialization::make_array(m_ImageMask->getData(), m_ImageMask->getWidth()*m_ImageMask->getHeight());
        }
        else
        {
            bool b = false;
            ar & b;
        }
    }

    template<class Archive>
    void load(Archive & ar, const unsigned int version_b)
    {
        clear();

        unsigned version;
        ar & version;

        if ( version != 12 )
        {
          throw "File has wrong version number.";
        }

        ar & m_Name;

        m_ImageY = new cv::Mat();
        m_ImageUV = new cv::Mat();

        ar & m_ImageY;
        ar & m_ImageUV;

        bool hasMask;
        ar & hasMask;

        if ( hasMask )
        {
          int width;
          int height;
          ar & width;
          ar & height;

          // TODO check this for correct behavior
          m_ImageMask = new ImageMaskCV( width, height);
          ar & boost::serialization::make_array(m_ImageMask->getData(), width*height);
        }
        else
        {
          m_ImageMask = NULL;
        }

        calculateProperties();
    }

    BOOST_SERIALIZATION_SPLIT_MEMBER()

    private:

        void clear();
    void deleteAll();

    /// @brief base Data
    std::string m_Name;
    cv::Mat* m_ImageY;
    cv::Mat* m_ImageUV;
    ImageMaskCV* m_ImageMask;
    ImageMaskCV* m_ImageMaskWithBorder;

    /// @brief extracted data
    cv::Mat* m_MaskedImageY;
    cv::Mat* m_MaskedImageUV;
    std::vector< KeyPoint >* m_KeyPoints;
    std::vector<Point2D>* m_Outline;
    Point2D m_Center;

    int m_Border;
};

BOOST_CLASS_VERSION(ImagePropertiesCV, 12)

#endif
