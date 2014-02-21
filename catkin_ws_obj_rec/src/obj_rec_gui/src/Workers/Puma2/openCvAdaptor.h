#ifndef H_PUMA2OPENCV_H
#define H_PUMA2OPENCV_H

#include "puma2config.h"

#ifdef HAVE_OPENCV 


/* OpenCV stuff */
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <opencv/cxcore.h>

#include "GrayLevelImage8.h"
#include "ColorImageRGB8.h"

namespace puma2 {

/**
 * @class	Adaptor Class to use openCV together with PUMA2
 */

class AdaptorOpenCvToPuma2 
{
	IplImage * iplImage;
public:
    AdaptorOpenCvToPuma2 (const ColorImageRGB8&);
    // AdaptorOpenCvToPuma2 (ColorImageRGB8&);
    AdaptorOpenCvToPuma2 (const GrayLevelImage8&);
    // AdaptorOpenCvToPuma2 (GrayLevelImage8&);
    ~AdaptorOpenCvToPuma2 ();
	IplImage * asIplImage();
	const IplImage * asIplImage() const;
protected:
public:
private:
void initIpl(void * data, const char * colorModel, int w, int h, int nChannels);
};

}

#endif
#endif
