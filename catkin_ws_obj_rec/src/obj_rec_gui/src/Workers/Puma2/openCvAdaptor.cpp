#include "puma2config.h"
#include "openCvAdaptor.h"

using namespace puma2;

#ifdef HAVE_OPENCV

AdaptorOpenCvToPuma2::~AdaptorOpenCvToPuma2()
{
  delete iplImage;
}

#if 0
AdaptorOpenCvToPuma2::AdaptorOpenCvToPuma2(ColorImageRGB8& img)
{
    if (img.getWidth() * img.getHeight() == 0) throw "Cannot convert to openCV image";
    initIpl((void*)&(img[0][0]), (const char*) "RGB", img.getWidth(), img.getHeight(), 3);
    iplImage->widthStep = ((byte*)&(img[1][0])) - ((byte*)&(img[0][0])); /* works for subimages too ! */
}
#endif

AdaptorOpenCvToPuma2::AdaptorOpenCvToPuma2(const ColorImageRGB8& img)
{
    if (img.getWidth() * img.getHeight() == 0) throw "Cannot convert to openCV image";
    initIpl((void*)&(img[0][0]), (const char*) "RGB", img.getWidth(), img.getHeight(), 3);
    iplImage->widthStep = ((byte*)&(img[1][0])) - ((byte*)&(img[0][0])); /* works for subimages too ! */
}

#if 0
AdaptorOpenCvToPuma2::AdaptorOpenCvToPuma2(GrayLevelImage8& img)
{
    if (img.getWidth() * img.getHeight() == 0) throw "Cannot convert to openCV image";
    initIpl((void*)&(img[0][0]), (const char*) "Y", img.getWidth(), img.getHeight(), 1);
    iplImage->widthStep = ((byte*)&(img[1][0])) - ((byte*)&(img[0][0])); /* works for subimages too ! */
}
#endif

AdaptorOpenCvToPuma2::AdaptorOpenCvToPuma2(const GrayLevelImage8& img)
{
    if (img.getWidth() * img.getHeight() == 0) throw "Cannot convert to openCV image";
    initIpl((void*)&(img[0][0]), (const char*) "Y", img.getWidth(), img.getHeight(), 1);
    iplImage->widthStep = ((byte*)&(img[1][0])) - ((byte*)&(img[0][0])); /* works for subimages too ! */
}

IplImage * AdaptorOpenCvToPuma2::asIplImage()
{
  return iplImage;
}

const IplImage * AdaptorOpenCvToPuma2::asIplImage() const
{
  return iplImage;
}

void AdaptorOpenCvToPuma2::initIpl(void * data, const char * colorModel,
  int w, int h, int nChannels)
{
   // std::cerr << "Initialize GVI to IPL" << std::endl;
   char channelSeq[4] = "Y";  /// ***** is this correct????
   // char * channelSeq = colorModel;
   int width  = w;
   int height = h;
   // IplImage * ipli = (IplImage*) this;
   // IplImage *& ipli = this->iplImage;
   IplImage * ipli;
   this->iplImage = ipli = new IplImage;

   // IplImage* mask ;
   // IplImage* mask = (Cv_iplCreateImageHeader) (
   //     1, 0, IPL_DEPTH_1U, "GRAY", "GRAY",
   //     IPL_DATA_ORDER_PIXEL, IPL_ORIGIN_TL,
   //     IPL_ALIGN_DWORD, width, height, NULL, NULL,
   //      NULL, NULL);

   ipli->nSize     = sizeof(IplImage);
   ipli->nChannels = nChannels;
   ipli->alphaChannel  = 0;
   ipli->depth = IPL_DEPTH_8U;  /* pixel depth in bits: */
   ipli->dataOrder = 0;  /* 0 - interleaved , 1 - separate color channels. */
   ipli->origin = 0;        /* 0 - top-left origin */
   ipli->width     = width;
   ipli->height    = height;
   ipli->align     = IPL_ALIGN_QWORD; /* Alignment of image rows (4 or 8).  */
   ipli->ID        = 0; /* version (=0)*/
   ipli->roi = NULL;  /* image ROI. if NULL, the whole image is selected */
   ipli->maskROI = NULL ; /* must be NULL */
   ipli->imageId = NULL ;     /* ditto */
   ipli->tileInfo = NULL ; /* ditto */

  *((int*)(*ipli).colorModel) = *((int*) colorModel);
  *((int*)(*ipli).channelSeq) = *((int*) channelSeq);

    ipli->imageSize = ipli->width * ipli->height * nChannels ;
    ipli->imageData = (char*) data ;  /* pointer to aligned image data */
    ipli->imageDataOrigin = ipli->imageData;
    ipli->imageDataOrigin = NULL;

    // this code works for normal images only, not for subimages:
    ipli->widthStep = ipli->width*nChannels;   /* size of aligned image row in bytes */
}

#endif
