/*******************************************************************************
 *  Camera.h
 *
 *  (C) 2005 AG Aktives Sehen <agas@uni-koblenz.de>
 *           Universitaet Koblenz-Landau
 *
 *  Information on Code Review state:
 *  §Author:R5; DevelTest: 10.04.2006; Reviewer:MS; Review: Date; State: NOK§
 *
 *  Additional information:
 *  $Id: Camera.h 44313 2011-04-06 22:46:28Z agas $
 ******************************************************************************/

#ifndef Camera_H
#define Camera_H

#include <string>
#include "../../Workers/ImageSources/ImageSources.h"

/**
 * @class Camera
 * @brief This container class holds properties and settings of a camera
 * @author David Gossow (RX)
 */
class Camera{

    public:

        /**
        * @brief The constructor
        * @param deviceDescription Used to identify the camera
        * @param imageWidth,imageHeight Size of the camera image
        * @param horizontalViewAngle opening angle of the camera
				* @param rotateImage If true, the image will be rotated 90° counter-clockwise
        */
        Camera( std::string deviceDescription="", int formatId=0, int subFormatId=0, float horizontalViewAngle=0, std::string customProperties="", bool rotateImage=false );

        ~Camera() {};

        /** @return Device description (needed for Puma2Grabber) */
        std::string getDeviceDescription() { return m_DeviceDescription; }

        float getHorizontalViewAngle() { return m_HorizontalViewAngle; }

        /** @brief calculates the horizontal view angle from the diagonal view angle, width and height */
        float getHorizontalViewAngle( float diagonalViewAngle );

        /** @return Format indices as needed by setFormat() */
        int getFormatId() { return m_FormatId; }
        int getSubFormatId() { return m_SubFormatId; }
				
        int getRotateImage() { return m_RotateImage; }

        std::map<std::string,double> getCustomProperties() { return m_CustomProperties; }

    private:

        /** @brief Member variables holding data (see above) */
        std::string m_DeviceDescription;

        int m_FormatId;
        int m_SubFormatId;

        std::map<std::string,double> m_CustomProperties;

        /** @brief Stores the horizontal view angle of the cam */
        float m_HorizontalViewAngle;
				
				bool m_RotateImage;

};

#endif
