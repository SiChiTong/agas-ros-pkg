/**
 * @file    FeatureImage.h
 * @brief   Contains a superclass for all feature images.
 *
 * Contains a superclass for all feature images. This superclass is abstract
 * and, therefore, it cannot be instaniated directly. For scalar features use
 * the class ScalarFeatureImage instead.
 *
 * (C) 2007 AG Aktives Sehen <agas@uni-koblenz.de>, Universitaet Koblenz-Landau
 * $Id: $
 */

#ifndef FeatureImage_H
#define FeatureImage_H

#include <vector>
#include <cstdlib>

#include "SingleElementImage.h"
#include "Feature.h"
#include "GrayLevelImage8.h"

/**
 * @namespace  Puma2
 * @brief      The PUMA2 standard namespace.
 */
namespace puma2 {

/**
 * @class  FeatureImage
 * @brief  Superclass for all possible FeatureImages.
 *
 * This class provides a superclass for all possible feature images as edge
 * images etc. Although this class is a template class, it is abstract. So for
 * using this class, a subclass has always to be implemented. The reason for
 * this is an abstract method that cannot be generalized for all types of
 * features. For scalar featuare types use the class ScalarFeatureImage instead.
 * 
 * Some methods of this class provide the computing of maxima and minima. These
 * methods  require the implementation of the operator <. for the features that
 * are used. All of these methods return instances of the class Feature.
 *
 * @see     Feature
 * @see     ScalarFeatureImage
 * @author  Andreas Kasten <stultissimum@uni-koblenz.de>
 * @date    Juli 2007
 */
template <typename T> class FeatureImage : public SingleElementImage<T> {

    private:

        /**
         * @brief Creates a binary mask for defining a local neighborhood.
         *
         * This method is just used for creating a radial binary mask of boolean
         * values. The mask itself is used for searching local optima. The size
         * of the mask is specified by the parameter @p radius. The created mask
         * will be returned via call by reference, using the paramter @p mask.
         *
         * @param radius The radius of the mask.
         * @param mask The resulting binary mask.
         */
        void createMask(int radius, bool **(&mask));

    public:

        /**
         * @brief Default constructor.
         * 
         * Default constructor. Creates a new image of the given size. The size
         * is specified by the parameters @p x and @p y. The default values for
         * both parameters are 0.
         * 
         * @p x The horizontal size of the image.
         * @p y The vertical size of the image.
         */
        FeatureImage(int x = 0, int y = 0)
            : SingleElementImage<T>(x,y) {}

        /**
         * @brief Subimage constructor.
         * 
         * This is the subimage constructor. It creates a subimage of the given
         * base image @p m. The size is given by the parameters @p x and @p y,
         * the position of the subimage in the original image is given by the
         * offsets @p xo and @p yo. The subimage will be created as a reference
         * to the base image. The constructor will do all required checks on the
         * sizes of the images during the creation process.
         *
         * @param x: Horizontal size of the subimage.
         * @param y: Vertical size of the subimage.
         * @param m: Pointer to master image of which the image becomes a
         *           subimage.
         * @param xo: offset for horizontal position of subimage in master
         *            image.
         * @param yo: offset for vertical position of subimage in master image.
         */
        FeatureImage(int x, int y, FeatureImage * m, int xo, int yo)
            : SingleElementImage<T>(x,y,m,xo,yo) {}

        /**
         * Destructor
         */
        virtual ~FeatureImage() {}


        /**
         * @brief Returns the maximum feature value.
         *
         * Returns the maximum value of the image, returned as a Feature. For
         * using this method, the current feature type T has to implement the
         * operator <. Beware of using arrays as the feature type T! The
         * operator <. will only compare the memory addresses and NOT the
         * values.
         *
         * @return The maximum value, given as a Feature.
         */
        Feature<T> getGlobalMaximum();


        /**
         * @brief Returns the n maxima of the given image.
         *
         * Returns the @p n maxima of the given image. The maxima will be
         * returnded as a vector of Features. For using this method, the current
         * feature type T has to implement the operator <. Beware of using
         * arrays as the feature type T! The operator <. will only compare the
         * memory addresses and NOT the values.
         *
         * @param n The number of maximum values that have to be returned.
         * @param return A vector of Features.
         */
        std::vector<Feature<T> > getGlobalMaxima(int n);


        /**
         * @brief Returns the minimum feature value of the actual image.
         *
         * Returns the minimum feature value of the actual image. For using this
         * method, the current feature type T has to implement the operator <.
         * Beware of using arrays as the feature type T! The operator <. will
         * only compare the memory addresses and NOT the values.
         *
         * @return The minimum value, given as a Feature.
         */
        Feature<T> getGlobalMinimum();


        /**
         * @brief Returns the n minima of the actual image.
         * 
         * Returns the @p n minima of the actual image. The minima will be
         * returnded as a vector of Features. For using this method, the current
         * feature type T has to implement the operator <. Beware of using
         * arrays as the feature type T! The operator <. will only compare the
         * memory addresses and NOT the values.
         *
         * @param n The number of minimum values that have to be returned.
         * @return A vector of Features.
         */
        std::vector<Feature<T> > getGlobalMinima(int n);


        /**
         * @brief Returns a vector of n local minima of the actual image.
         *
         * Returns a vector of @p n local minima of the actual image. The minima
         * are located within a specified range. The range itself is defined by
         * a given @p radius. A value is a minimum value iff there is no other
         * value within the range that is lower than this value. Furthermore,
         * every minimum has to be lower than a given @p threhold.
         *
         * @param n The number of local minimum values that have to be returned.
         * @param threshold The maximum feature value that has to be reached for
         *                  defining a feature as a local minimum.
         * @param radius The radius of the range that will be used for defining
         *               'local'.
         * @return A vector of Features.
         */
        std::vector<Feature<T> > getLocalMinima(int n, T threhold, int radius);


        /**
         * @brief Returns a vector of all local minima of the actual image.
         *
         * Returns a vector of all local minima of the actual image. The minima
         * are located within a specified range. The range itself is defined by
         * a given @p radius. A value is a minimum value iff there is no other
         * value within the range that is lower than this value. Furthermore,
         * every minimum has to be lower than a given @p threhold.
         *
         * @param threshold The maximum feature value that has to be reached for
         *                  defining a feature as a local minimum.
         * @param radius The radius of the range that will be used for defining
         *               'local'.
         * @return A vector of Features.
         */
        std::vector<Feature<T> > getLocalMinima(T thresh, int radius);


        /**
         * @brief Returns a vector of n local maxima of the actual image.
         *
         * Returns a vector of @p n local maxima of the actual image. The maxima
         * are located within a specified range. The range itself is defined by
         * a given @p radius. A value is a maximum value iff there is no other
         * value within the range that is greater than this value. Furthermore,
         * every maximum has to be greater than a given @p threhold.
         *
         * @param n The number of local maximum values that have to be returned.
         * @param threshold The minimum feature value that has to be reached for
         *              defining a feature as a local maximum.
         * @param radius The radius of the range that will be used for defining
         *               'local'.
         * @return A vector of Features.
         */
        std::vector<Feature<T> > getLocalMaxima(int n, T threshold, int radius);


        /**
         * @brief Returns a vector of all local maxima of the actual image.
         *
         * Returns a vector of all local maxima of the actual image. The maxima
         * are located within a specified range. The range itself is defined by
         * a given @p radius. A value is a maximum value iff there is no other
         * value within the range that is greater than this value. Furthermore,
         * every maximum has to be greater than a given @p threhold.
         *
         * @param threshold The minimum feature value that has to be reached for
         *              defining a feature as a local maximum.
         * @param radius The radius of the range that will be used for defining
         *               'local'.
         * @return A vector of Features.
         */
        std::vector<Feature<T> > getLocalMaxima(T threshold, int radius);


        /**
         * @brief Returns a GrayLevelImage of the actual FeatureImage.
         *
         * @return The resulting GrayLevelImage.
         */
        virtual GrayLevelImage8 getGrayLevelImageRepresentation() = 0;

};


/*******************************************************************************
 * Returns the maximum of the given image.
 ******************************************************************************/
template <typename T> Feature<T> FeatureImage<T>::getGlobalMaximum()
{
    int h = this->getHeight();
    int w = this->getWidth();

    Feature<T> max;

    max.x   = 0;
    max.y   = 0;
    max.val = &(this->c0[0][0]);

    for (int y = 0; y < h; ++y)
    {
        for (int x = 0; x < w; ++x)
        {
            T *tmp = &(this->c0[y][x]);
            if ( (*max.val) < (*tmp) )
            {
                max.x   = x;
                max.y   = y;
                max.val = tmp;
            }
        }
    }

    return max;
}


/*******************************************************************************
 * Returns the n maxima of the given image.
 ******************************************************************************/
template <typename T> std::vector<Feature<T> > FeatureImage<T>::getGlobalMaxima(int n)
{
    int h = this->getHeight();
    int w = this->getWidth();

    assert (w*h >= n);

    std::vector<Feature<T> > maxima;
    maxima.resize(n);

    // Store if the current slot has already been written. This temporary vector
    // replaces an initialization of all slots with maximum elements which in
    // fact cannot be done with such features like edges.
    std::vector<bool> isSet(n,false);

    for (int y = 0; y < h; ++y)
    {
        for (int x = 0; x < w; ++x)
        {
            T *tmp = &(this->c0[y][x]);
            if ( !isSet[0] || ( (*maxima[0].val) < *tmp ) )
            {
                int i=1;
                while ( (i < n) && (!isSet[i] || ( *maxima[i].val) < *tmp ) )
                    ++i;
                --i;
                for (int j=0; j<i; ++j)
                {
                    maxima[j].x   = maxima[j+1].x;
                    maxima[j].y   = maxima[j+1].y;
                    maxima[j].val = maxima[j+1].val;
                }
                maxima[i].x   = x;
                maxima[i].y   = y;
                maxima[i].val = tmp;
                isSet [i]     = true;
            }
        }
    }

    return maxima;
}


/*******************************************************************************
 * Returns the minimum of the given image.
 ******************************************************************************/
template <typename T> Feature<T> FeatureImage<T>::getGlobalMinimum()
{
    int h = this->getHeight();
    int w = this->getWidth();

    Feature<T> min;

    min.x   = 0;
    min.y   = 0;
    min.val = &(this->c0[0][0]);
    for (int y = 0; y < h; ++y) {
        for (int x = 0; x < w; ++x)
        {
            T *tmp = &(this->c0[y][x]);
            if ( *tmp < (*min.val) )
            {
                min.x   = x;
                min.y   = y;
                min.val = tmp;
            }
        }
    }

    return min;
}


/*******************************************************************************
 * Returns the n minima of the given image.
 ******************************************************************************/
template <typename T> std::vector<Feature<T> > FeatureImage<T>::getGlobalMinima(int n)
{
    int h = this->getHeight();
    int w = this->getWidth();

    assert (w*h >= n);

    std::vector<Feature<T> > minima;
    minima.resize(n);

    // Store if the current slot has already been written. This temporary vector
    // replaces an initialization of all slots with minimum elements which in
    // fact cannot be done with such features like edges.
    std::vector<bool> isSet(n,false);

    for (int y=0; y<h; ++y)
    {
        for (int x=0; x<w; ++x)
        {
            T *tmp = &(this->c0[y][x]);
            if ( !isSet[0] || ( *tmp < (*minima[0].val) ) )
            {
                int i=1;
                while ( (i < n) && (!isSet[i] ||  *tmp < (*minima[i].val) ) )
                    ++i;
                for (int j = --i; j > 0; --j)
                {
                    minima[j-1].x   = minima[j].x;
                    minima[j-1].y   = minima[j].y;
                    minima[j-1].val = minima[j].val;
                }
                minima[i].x   = x;
                minima[i].y   = y;
                minima[i].val = tmp;
                isSet [i]     = true;
            }
        }
    }

    return minima;
}



/*******************************************************************************
 * Returns a vector of n local maxima of the actual image.
 ******************************************************************************/
template <typename T> std::vector<Feature<T> > FeatureImage<T>::getLocalMaxima(int n,
    T thresh, int radius)
{
    int h = this->getHeight();
    int w = this->getWidth();

    assert (w*h >= n);

    std::vector<Feature<T> > maxima;
    maxima.resize(n);

    // Create the mask that will be used for defining locality.
    bool **mask;
    createMask(radius, mask);


    // Store if the current slot has already been written. This temporary vector
    // replaces an initialization of all slots with maximum elements which in
    // factcannot be done with such features like edges.
    std::vector<bool> isSet(n,false);


    for (int y = 0; y < h; ++y)
    {
        for (int x=0; x<w; ++x)
        {
            bool isMax = true;
            T * val = &(this->c0[y][x]);
            for (int i = -radius; ( i <= radius ) && isMax; ++i)
            {
                for (int j = -radius; (j <= radius) && isMax; ++j)
                {
                    if ( (mask[i][j]) && ( j!= 0 || i != 0) &&
                         ( y + i >= 0 && y + i < h ) &&
                         ( x + j >= 0 && x + j < w ) )
                        isMax &= ( *val >= this->c0[y+i][x+j] && *val > thresh);
                }
            }
            if (isMax)
            {
                T *tmp = &(this->c0[y][x]);
                if ( !isSet[0] || ( (*maxima[0].val) < *tmp ) )
                {
                    int i=1;
                    while ( (i < n) && (!isSet[i] || (*maxima[i].val) <  *tmp ))
                        ++i;
                    --i;
                    for (int j=0; j<i; ++j)
                    {
                        maxima[j].x   = maxima[j+1].x;
                        maxima[j].y   = maxima[j+1].y;
                        maxima[j].val = maxima[j+1].val;
                    }
                    maxima[i].x   = x;
                    maxima[i].y   = y;
                    maxima[i].val = tmp;
                    isSet [i]     = true;
                }
            }
        }
    }

    return maxima;
}



/*******************************************************************************
 * Returns a vector of all local maxima of the actual image.
 ******************************************************************************/
template <typename T> std::vector<Feature<T> > FeatureImage<T>::getLocalMaxima(T thresh,
    int radius)
{
    int h = this->getHeight();
    int w = this->getWidth();

    // Create the mask that will be used for defining locality.
    bool **mask;
    createMask(radius, mask);

    std::vector<Feature<T> > maxima;

    for (int y = 0; y < h; ++y)
    {
        for (int x = 0; x < w; ++x)
        {
            bool isMax = true;
            T * val = &(this->c0[y][x]);
            for (int i = -radius; (i <= radius) && isMax; ++i)
            {
                for (int j = -radius; (j <= radius) && isMax; ++j)
                {
                    if ( (mask[i][j]) && (j != 0 || i != 0) &&
                         (y + i >= 0 && y + i < h) &&
                         (x + j >= 0 && x + j < w))
                        isMax &= (*val >= this->c0[ y + i ][ x + j ] &&
                                  *val > thresh);
                }
            }
            if (isMax)
            {
                Feature<T> feat;
                feat.x   = x;
                feat.y   = y;
                feat.val = &(this->c0[y][x]);
                maxima.push_back(feat);
            }
        }
    }

    return maxima;
}



/*******************************************************************************
 * Returns a vector of n local minima of the actual image.
 ******************************************************************************/
template <typename T> std::vector<Feature<T> > FeatureImage<T>::getLocalMinima(int n,
    T thresh, int radius)
{
    int h = this->getHeight();
    int w = this->getWidth();

    assert (w*h >= n);

    std::vector<Feature<T> > minima;
    minima.resize(n);

    // Create the mask that will be used for defining locality.
    bool **mask;
    createMask(radius, mask);


    // Store if the current slot has already been written. This temporary vector
    // replaces an initialization of all slots with minimum elements which in
    // fact cannot be done with such features like edges.
    std::vector<bool> isSet(n,false);


    for (int y = 0; y < h; ++y)
    {
        for (int x = 0; x < w; ++x)
        {
            bool isMin = true;
            T * val = &(this->c0[y][x]);
            for (int i = -radius; (i <= radius) && isMin; ++i)
            {
                for (int j = -radius; (j <= radius) && isMin; ++j)
                {
                    if ( (mask[i][j]) && (j != 0 || i != 0) &&
                         (y + i >= 0 && y + i < h) &&
                         (x + j >= 0 && x + j < w))
                        isMin &= (*val <= this->c0[ y + i ][ x + j ] &&
                                  *val < thresh);
                }
            }
            if (isMin) {
                T *tmp = &(this->c0[y][x]);
                if ( !isSet[0] || ( (*minima[0].val) > *tmp ) )
                {
                    int i=1;
                    while ( (i < n) && (!isSet[i] || (*minima[i].val) > *tmp ) )
                        ++i;
                    --i;
                    for (int j = 0; j < i; ++j)
                    {
                        minima[j].x   = minima[ j + 1 ].x;
                        minima[j].y   = minima[ j + 1 ].y;
                        minima[j].val = minima[ j + 1 ].val;
                    }
                    minima[i].x   = x;
                    minima[i].y   = y;
                    minima[i].val = tmp;
                    isSet [i]     = true;
                }
            }
        }
    }

    return minima;
}


/*******************************************************************************
 * Returns a vector of all local minima of the actual image.
 ******************************************************************************/
template <typename T> std::vector<Feature<T> > FeatureImage<T>::getLocalMinima(T thresh,
    int radius)
{
    int h = this->getHeight();
    int w = this->getWidth();

    std::vector<Feature<T> > minima;

    // Create the mask that will be used for defining locality.
    bool **mask;
    createMask(radius, mask);

    for (int y = 0; y < h; ++y)
    {
        for (int x = 0; x < w; ++x)
        {
            bool isMin = true;
            T * val = &(this->c0[y][x]);
            for (int i = -radius; (i <= radius) && isMin; ++i)
            {
                for (int j = -radius; (j <= radius) && isMin; ++j)
                {
                    if ( (mask[i][j]) && (j != 0 || i != 0) &&
                         (y + i>= 0 && y + i < h) && (x + j >= 0 && x + j < w))
                        isMin &= (*val <= this->c0[y+i][x+j] && *val < thresh);
                }
            }
            if (isMin)
            {
                Feature<T> feat;
                feat.x   = x;
                feat.y   = y;
                feat.val = &(this->c0[y][x]);
                minima.push_back(feat);
            }
        }
    }

    return minima;
}



/*******************************************************************************
 * Creates a binary mask for defining a local neighborhood.
 ******************************************************************************/
template <typename T> void FeatureImage<T>::createMask(int radius, bool **(&mask))
{
    int size = 2*radius+1;
    int radius21 = radius * radius + 1;

    mask = (bool**) calloc(size, sizeof(bool*)) + radius;

    for(int y = -radius; y <= radius; ++y)
    {
        *(mask+y) = (bool*) calloc(size, sizeof(bool)) + radius;
        for (int x = -radius; x <= radius; ++x)
        {
            if (radius21 >= x * x + y * y)
                mask[y][x] = true;
            else
                mask[y][x] = false;
        }
    }
}

} // end of namespace

#endif
