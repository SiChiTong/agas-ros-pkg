/*******************************************************************************
 *  ImageGrabber.h
 *
 *  (C) 2006 AG Aktives Sehen <agas@uni-koblenz.de>
 *           Universitaet Koblenz-Landau
 *
 *  Additional information:
 *  $Id: ImageGrabber.h 44313 2011-04-06 22:46:28Z agas $
 ******************************************************************************/

#ifndef ImageGrabber_H
#define ImageGrabber_H

//#include "Camera.h"
//#include "../../Workers/Puma2/ColorImageUV8.h"

//#include "GrabbingDevice.h"
#include "../../Workers/Puma2/ColorImageRGB8.h"
#include "../../Workers/Puma2/GrayLevelImage8.h"

#include "../../Workers/ImageSources/ImageSources.h"

//#include <map>
//#include <string>

/**
 * @brief This class grabs mono and stereo images from the connected cameras
 * @author David Gossow (RX)
 */
class ImageGrabber
{
  public:

    /** @brief specifies color spaces & depths */
    enum ColorFormat{
      UYVY8=0, //Pixel format of YUV 4:2:2
      RGB8=1,
      GRAY8=2,
      Y8UV8=3 //Y and UV as separate images
    };

    /** @brief specifies image qualities (e.g. when scaling) */
    enum ImageQuality{
      LOW=0,
      HIGH=1
    };

    /** @brief specifies down scaling factors for images */
    enum ScaleFactor{
      FULL=1,
      HALF=2,
      QUARTER=4
    };

    ImageGrabber() {};

//    /**
//     * @brief The constructor
//     * @param parentName Used as main name for the timers */
    ImageGrabber(std::string parentName);

//    /** The destructor. */
    virtual ~ImageGrabber();

//    /**
//     * @brief Assigns an ID to a camera, initializes it and starts grabbing process
//     * @param sourceId ID to be assigned to the cameras
//     * @param cam contains description & parameters of the camera
//     * @return false if an error has occured
//     */
//    bool setCamera( ImageSources::SourceId sourceId, Camera cam );

//    /**
//     * @brief Grabs a single RGB image from a camera.
//     * @param sourceId ID of the camera to be used
//     * @param image target image
//     * @return true if grabbing was successfull
//     *         false otherwise
//     */
//    bool grabImageRgb( ImageSources::SourceId sourceId, ScaleFactor scaling, ImageQuality quality, puma2::ColorImageRGB8 &image );

//    /**
//     * @brief Grabs a single image from a YUV422 camera and stores it in a GrayLevelmage8 and a ColorImageUV8
//     * @param sourceId ID of the camera to be used
//     * @param grayImage,uvImage target images
//     * @return true if grabbing was successfull
//     *         false otherwise
//     */
//    bool grabImageYuv( ImageSources::SourceId sourceId, ScaleFactor scaling, ImageQuality quality,
//                       puma2::GrayLevelImage8 &grayImage, puma2::ColorImageUV8 &uvImage );

//    /**
//     * @brief Does whitebalancing.
//     * @return True if whitebalancing is done without errors.
//    */
//    bool doWhiteBalance( ImageSources::SourceId sourceId );

//    /**
//     * @brief Sets camera proptery either absolute or relative.
//     * @return True if setting of camera proptery is done without errors.
//     */
//    bool setCameraProperty(ImageSources::SourceId sourceId, string param, double value, bool isAbsolute=true);

//    /**
//     * @brief Gets camera proptery
//     * @return value of camera property
//     */
//    double getCameraProperty(ImageSources::SourceId sourceId, string param);

//    /**
//     * @brief Sets camera zoom proptery either absolute or relative wth zoomfactorvalues in one dimension.
//     * @brief Needed to convert zoomvalues to zoomfactors.
//     * @return True if setting of camera proptery is done without errors.
//     */
//    bool setCameraZoomProperty(ImageSources::SourceId sourceId, float factor, bool isAbsolute=true);

//    /**
//     * @brief Clean the buffer queue. Should be called at least as often as the highest frame rate
//     * @brief of all cameras.
//     */
//    void cleanBuffers();

//    /**
//     * @brief Calculate viewangle from zoomvalue (range: 40-1432)
//     * @brief CAUTION!: Due to missing real viewangle the values might be wrong. Calibration PI * thumb (5 mesasures, cubic polynom).
//     */
//    static double zoomToAngle(double zoom);

//    /**
//     * @brief Calculate zoomvalue (range: 40-1432) from viewangle
//     * @brief CAUTION!: inverse function to zoomToAngle (see description)
//     */
//    static double angleToZoom(double angle);

//    /*
//    double factorToAngle(double factor);

//    double angleToFactor(double arc);*/

//    void applySobel(puma2::GrayLevelImage8* image);

//    long measureSharpness(puma2::GrayLevelImage8* image);

//  private:

//    /**
//     * @brief Initializes a camera.
//     * @return True if initialization was successful.
//     */
//    bool initCamera( ImageSources::SourceId sourceId );

//    /**
//     * @brief Stops a camera.
//     * @return True if camera could be stopped without errors.
//     */
//    bool stopCamera( ImageSources::SourceId sourceId );

//    /** @brief checks if the given camera is available */
//    bool checkCamera( ImageSources::SourceId sourceId );

//    /** @brief grab raw YUV data */
//    bool grabDataYuv( ImageSources::SourceId sourceId, unsigned char* buffer );

//    /** @brief Convert & scale raw YUV422 data
//     *  @note  In YUV422, the U and V channel have only half the horizontal resolution
//     *         Data format: UY,VY,UY,VY,..
//     */
//    bool yuv422To444( ImageQuality quality, int width, int height, ScaleFactor scaling, unsigned char* buffer,
//                      puma2::GrayLevelImage8 &yImage, puma2::ColorImageUV8 &uvImage);

//    /** @brief maps an id to a camera / its state / the associated grabbing device */
//    map<ImageSources::SourceId,Camera> m_Cameras;
//    map<ImageSources::SourceId,bool> m_Initialized;
//    map<ImageSources::SourceId,puma2::GrabbingDevice*> m_GrabbingDevices;

//    /** @brief maps format description strings to format IDs */
//    map<std::string,ColorFormat> m_ColorFormatId;


};

#endif
