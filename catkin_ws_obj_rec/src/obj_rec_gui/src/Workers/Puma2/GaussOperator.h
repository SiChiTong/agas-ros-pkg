/**
 * @file    GaussOperator.h
 * @brief   Contains the class GaussOperator for blurring an image.
 *
 * Contains the class GaussOperator that is used for performing a convolution
 * with a gauss kernel on all SingleElementImages.
 * The implementation for this class is taken from PUMA.
 *
 * (C) 2007 AG Aktives Sehen <agas@uni-koblenz.de>, Universitaet Koblenz-Landau
 * $Id: $
 */

#ifndef GaussOperator_H
#define GaussOperator_H

#include "ImageToImageOperator.h"
#include "SingleElementImage.h"
#include <cmath>

namespace puma2 {

/**
 * @class GaussOperator
 * @brief Gauss operator for a template image.
 *
 * This class is a image-to-image operator that performs a convolution with a
 * gauss kernel on a SingleElementImage. For using this operator, a variable of
 * the type double has to be casted into the type of the SingleElementImage.
 * Therefore, this type has to implement a corresponding cast operator.
 *
 * @see SingleElementImage
 */
template <class T> class GaussOperator :
    public ImageToImageOperator<SingleElementImage<T>,SingleElementImage<T> >
{
    private:

    /**
     * The size of the gaussian mask. The mask itself is created as a linear
     * mask in the constructor.
     */
    int maskradius;

    /**
     * The standard deviation, used for the gaussian kernel.
     */
    float sigma;

    /**
     * The gaussian kernel, used for the convolution. Since the gaussian kernel
     * is separable, the kernel is stored as a 1D-array.
     */
    double* kernel;


    public:

    /**
     * @brief Creates the kernel for the convolution.
     *
     * The constructor creates a kernel for computing the Gaussian filter. The
     * size of the kernel is given by the optional paramater @p maskSize. If
     * this parameter is not set or has an invalid value, the mask size will be
     * automatically computed, using the given @p sigma. If the standard
     * deviation sigma has an invalid value, it will be put in the valid range.
     *
     * \param sigma used for computing the Gaussian kernel as well as its size.
     *              The minimum value is 0.5, the maximum value is 10.0. If the
     *              parameter is not within this range, it will be corrected.
     * \param maskSize The size of the Gaussian kernel. The minimum value is 3,
     *                 the maximum value is 61. If the set value is not within
     *                 this range (or if it is not set at all), it will be
     *                 computed automatically, according to the value of sigma.
     */
    GaussOperator(float sigma=3.0f, int maskRadius=0);

    /**
     * @brief Deletes the kernel.
     *
     * The destructor deletes the private kernel, that was used for the
     * convolution.
     */
    virtual ~GaussOperator();

    /**
     * @brief Performs a Gaussian filter on the given input image.
     *
     * This method performs a Gaussian filter on the given input image @p iImg.
     * The result is stored via call by reference in the paramater @p oImg.
     *
     * \param iImg The input image.
     * \param oImg The output image.
     */
    virtual void apply(const SingleElementImage<T> & iImg,
                       SingleElementImage<T> & oImg);

    float getSigma() const { return sigma; }
};


/*******************************************************************************
 * Creates the kernel for the convolution.
 ******************************************************************************/
template <class T> GaussOperator<T>::GaussOperator(float sigma, int maskSize)
{
    this->sigma = sigma;


    if (sigma < 0.5f)
        sigma = 0.5f;
    else if (sigma > 10.0f)
        sigma = 10.0f;

    int masksize;

    // If the maskSize is set manually, use this value.
    if (maskSize >= 3 && maskSize <= 61)
        masksize = maskSize;
    // If not or if the value is invalid, compute another radius.
    else
        masksize = (int) (floor(6 * sigma + 0.5));

    if (masksize % 2 == 0)
        ++masksize;

    this->maskradius = masksize / 2;

    kernel = new double[masksize];
    float sigma2 = sigma*sigma;

    // Compute the Gaussian kernel.
    double teiler = sqrt(2 * M_PI) * sigma;
    for(int i = -maskradius; i <= maskradius; i++)
    {
        double distance = (double) (i * i);
        double e = exp( -distance / (2.0 * sigma2) );
        kernel[i + maskradius] = e / teiler;
    }
}

/*******************************************************************************
 * Deletes the kernel.
 ******************************************************************************/
template <class T> GaussOperator<T>::~GaussOperator()
{
    delete[] kernel;
}


/*******************************************************************************
 * Performs a Gaussian filter on the given input image.
 ******************************************************************************/
template <class T> void GaussOperator<T>::apply(
    const SingleElementImage<T> & iImg, SingleElementImage<T> & oImg)
{
    int imageWidth  = iImg.getWidth();
    int imageHeight = iImg.getHeight();

    assert(2 * maskradius + 1 < imageHeight);
    assert(2 * maskradius + 1 < imageWidth);

    SingleElementImage<T> tempImage(imageWidth,imageHeight);

    // Faltung des Bildes mit 1.dim Gaussmaske in x-Richtung
    // Randbehandlung: Spiegelung der Randpixel
    for(int y_x = 0; y_x < imageHeight; ++y_x)
    {
        for(int x_x = 0; x_x < imageWidth; ++x_x)
        {
            double xValue = 0.0;
            for(int m = maskradius; m >= -maskradius; --m)
            {
                int xx = 0;
                if( x_x - m < 0 )
                  xx = 0;//- ( x_x - m );
                else if ( x_x - m > imageWidth - 1)
                  xx = imageWidth - 1;// - ( x_x - m );
                else
                  xx = x_x - m;
                xValue += iImg[y_x][xx] * kernel[m + maskradius];
            }
            tempImage[y_x][x_x] = (T) xValue;
        }
    }

    // Faltung des temporaeren x-Bildes mit 1.dim Gaussmaske in y-Richtung
    // Randbehandlung: Spiegelung der Randpixel
    for(int y_y = 0; y_y < imageHeight; ++y_y)
    {
        for(int x_y = 0; x_y < imageWidth; ++x_y)
        {
            double yValue = 0.0;
            for(int m = maskradius; m >= -maskradius; --m)
            {
                int yy = 0;
/*                if(y_y - m < 0) || y_y - m > imageHeight - 1)
                  yy = y_y + m;*/

                if ( y_y - m < 0 )
                  yy = 0;//- ( y_y - m );
                else if ( y_y - m > imageHeight - 1 )
                  yy = imageHeight - 1;// - ( y_y - m );//y_y - m - ( y_y - m - imageHeight - 1 );
                else
                  yy = y_y - m;
                yValue += tempImage[yy][x_y] * kernel[m + maskradius];
            }
            oImg[y_y][x_y] = (T) yValue;
        }
    }
}

}

#endif
