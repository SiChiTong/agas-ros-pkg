/*******************************************************************************
 *  ORLearningModule.h
 *
 *  (C) 2005, 2013 AG Aktives Sehen <agas@uni-koblenz.de>
 *                 Universitaet Koblenz-Landau
 *
 ******************************************************************************/

#ifndef ORLearningModule_H
#define ORLearningModule_H

#include <string>
#include <map>

#include "ros/ros.h"
#include <or_msgs/OrLearnCommand.h>
#include <or_msgs/OrImage.h>
#include <sensor_msgs/Image.h>

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "Architecture/StateMachine/StateMachine.h"

class ObjectProperties;
class ImagePropertiesCV;

/**
 * @class  ORLearningModule
 * @brief  This module learns objects with camera images by color and SURF-features
 * @author Patric Lambrecht, Jan Bornemeier (RX), David Gossow (RX), Viktor Seib (R20)
 */
class ORLearningModule
{

  public:

    enum CommandId {
      SetDifferenceThreshold,
      SetOpenRadius,
      SetIsolateLargestSegment,
      SetBorderSize,
      SetObjectType,
      GrabBackgroundImage,
      GrabForegroundImage,
      LoadBackgroundImage,
      LoadForegroundImage,
      DisplayImage,
      SaveImage,
      DeleteImage,
      LoadObject,
      SaveObject
    };

    enum ValueT {
      NoValue,
      FloatValue,
      StringValue,
      IntValue
    };

    /** @brief Define an enum type for the states of each machine here */
    enum ModuleState
    {
      IDLE,
      WAITING_FOR_BACKGROUND,
      WAITING_FOR_FOREGROUND
    };

    ORLearningModule(ros::NodeHandle *nh, std::string inputTopic);

    virtual ~ORLearningModule();

  private:

    void callbackOrLearnCommand( const or_msgs::OrLearnCommand::ConstPtr& msg );
    void callbackImage( const sensor_msgs::Image::ConstPtr& msg );
    void processImageMessage( const sensor_msgs::Image::ConstPtr& msg );

    void loadImage(std::string path);

    /**
    * @brief This method stores an objectProperties object to disk
     **/
    void saveObject ( std::string objectName );

    void loadObject ( std::string objectName );

    void displayImage ( int index );

    void deleteImage ( int index );

    void saveImage ( std::string name );

    void setBackground ( cv_bridge::CvImagePtr gray_image, cv_bridge::CvImagePtr color_image );

    void setForeground ( cv_bridge::CvImagePtr gray_image, cv_bridge::CvImagePtr color_image );

    void previewIsolatedImage();

    ImagePropertiesCV* makeImageProperties( std::string name="", bool crop=true );

    /** @brief Configuration parameters */
    std::string m_PathForSaving;
    unsigned m_HistogramBinSize;
    unsigned m_HistogramMinY;
    unsigned m_HistogramMaxY;

    float m_HistogramClearRange;

    cv::Mat* m_BackgroundImageGray;
    cv::Mat* m_BackgroundImageColor;
    cv::Mat* m_ForegroundImageGray;
    cv::Mat* m_ForegroundImageColor;

    ObjectProperties* m_ObjectProperties;

    std::string m_ObjectType;

    float m_DifferenceThreshold;
    float m_OpenRadius;
    float m_BorderSize;
    bool  m_IsolateLargestSegment;

    StateMachine<ModuleState> m_ModuleMachine;

    std::string m_SimpleImagePath;

    bool m_ImageRequested;

    ros::Subscriber m_ORLearnCommandSubscriber;
    ros::Subscriber m_ORImageSubscriber;
    ros::Subscriber m_ImageSubscriber;

    ros::Publisher m_OLPrimaryImagePublisher;
    ros::Publisher m_OLDebugImagePublisher;
    ros::Publisher m_ORLearningStatusPublisher;
};

#endif
