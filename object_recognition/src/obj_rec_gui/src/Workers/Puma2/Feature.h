/**
 * @file    Feature.h
 * @brief   Contains a struct, used for computing the extrema of a FeatureImage.
 *
 * Contains a struct, used for computing the extrema of a FeatureImage. Such an
 * image does not consist of Features, they will only be returned after calling
 * some methods of a FeatureImage.
 *
 * (C) 2007 AG Aktives Sehen <agas@uni-koblenz.de>, Universitaet Koblenz-Landau
 * $Id: $
 */

#ifndef Feature_H
#define Feature_H

/**
 * @namespace  Puma2
 * @brief      The PUMA2 standard namespace.
 */
namespace puma2 {

/**
 * @struct  Feature
 * @brief   Struct for obtaining extrema in a FeatureImage.
 *
 * This structure is used for obtaining local and global extrema of an instance
 * of the class FeatureImage. A FeatureImage does not consist of instances of
 * the struct Feature. However, instances of the struct Feature are used to
 * locate the found extrema of a FeatureImage. A Feature consists of a image
 * position and a value. The value is stored as a reference to the corresponding
 * position in the FeatureImage.
 *
 * @see     FeatureImage
 * @author  Andreas Kasten <stultissimum@uni-koblenz.de>
 * @date    Juli 2007
 */
template <class T> struct Feature {

    /**
     * Stores the x position in the FeatureImage of the current feature.
     */
    int x;

    /**
     * Stores the y position in the FeatureImage of the current feature.
     */
    int y;

    /**
     * Stores the a pointer to the position (x,y) in the FeatureImage. The value
     * at this posittion is not copied.
     */
    T *val;
    inline Feature<T>& operator=  (const Feature<T> & f);

    /**
     * Two feature are identical if they have the same position and the same
     * value. Equality only on the value is not sufficient.
     */
    inline bool        operator== (const Feature<T> & f);
    inline bool        operator!= (const Feature<T> & f);
    inline bool        operator<  (const Feature<T> & f);
    inline bool        operator>  (const Feature<T> & f);
    inline bool        operator<= (const Feature<T> & f);
    inline bool        operator>= (const Feature<T> & f);
};

template <class T> inline Feature<T>& Feature<T>::operator= (
        const Feature<T> & f)
{
    x = f.x;
    y = f.y;
    *val = *(f.val);
    return *this;
}

template <class T> inline bool Feature<T>::operator== (const Feature<T> & f)
{
    return (x == f.x) &&
           (y == f.y) &&
           (*val == *(f.val));
}

template <class T> inline bool Feature<T>::operator!= (const Feature<T> & f)
{
    return !(*this == f);
}

template <class T> inline bool Feature<T>::operator< (const Feature<T> & f)
{
    return (*val < *(f.val));
}

template <class T> inline bool Feature<T>::operator> (const Feature<T> & f)
{
    return (*(f.val) < *val );
}

template <class T> inline bool Feature<T>::operator<= (const Feature<T> & f)
{
    return !(f < *this);
}

template <class T> inline bool Feature<T>::operator>= (const Feature<T> & f)
{
    return !(*this < f);
}

} // end of namespace

#endif
