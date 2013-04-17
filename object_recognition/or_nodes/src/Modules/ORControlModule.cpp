/*******************************************************************************
 *  ORControlModule.cpp
 *
 *  (C) 2006 AG Aktives Sehen <agas@uni-koblenz.de>
 *           Universitaet Koblenz-Landau
 *
 *******************************************************************************/

#include <sstream>

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
//#include <sensor_msgs/Image.h>
#include <opencv2/highgui/highgui.hpp>

#include <or_msgs/BoundingBox2D.h>
#include <or_msgs/ExtractKeyPoints.h>
#include <or_msgs/OrImage.h>

#include "Architecture/Config/Config.h"

#include "ORControlModule.h"
#include "ORMatchingModule.h"

#define THIS ORControlModule

THIS::THIS(ros::NodeHandle *nh, ORMatchingModule* objRecMatchingModule)
{
    m_ORMatchingModule = objRecMatchingModule;

    m_ImagesInPipeline = 0;
    m_MaxImagesInPipeline = Config::getInt( "ObjectRecognition.iMaxImagesInPipeline" );

    m_Continuous = false;

    // subscribe to topics
    m_ORCommandSubscriber = nh->subscribe<or_msgs::OrCommand>("/or/commands", 10, &ORControlModule::callbackOrCommand, this);
    m_ORMatchResultSubscriber = nh->subscribe<or_msgs::OrMatchResult>("or/match_result", 10, &ORControlModule::callbackOrMatchResult, this);

    // advertise topics
    m_ExtractKeyPointsPublisher = nh->advertise<or_msgs::ExtractKeyPoints>("/or/extract", 10);
    m_DebugImagePublisher = nh->advertise<or_msgs::OrImage>("or/debug_image", 10);
}


THIS::~THIS()
{}


void THIS::callbackOrCommand( const or_msgs::OrCommand::ConstPtr& or_command_msg )
{
    ROS_DEBUG_STREAM("or_command message received");

    std::vector<or_msgs::BoundingBox2D> m_BoundingBoxes = or_command_msg->bounding_boxes;

    switch (or_command_msg->command)
    {
    case ORControlModule::GrabSingleImage:
    {
        m_SourceId = or_command_msg->int_value; // TODO  m_SourceId = ImageSources::SourceId(message->getInt());
        // send message for keypoint extraction
        or_msgs::ExtractKeyPoints extract_msg;
        extract_msg.img_source = m_SourceId;
        extract_msg.bounding_boxes = m_BoundingBoxes;
        m_ExtractKeyPointsPublisher.publish(extract_msg);

        m_ImagesInPipeline++;
        m_Continuous = false;
        break;
    }

    case ORControlModule::LoadSingleImage:
    {
        std::string fileName = or_command_msg->string_value;
        ROS_INFO_STREAM("Loading image file: " << fileName);

        // send message for keypoint extraction
        or_msgs::ExtractKeyPoints extract_msg;
        extract_msg.img_source = 0; // TODO ImageSources::SourceId::None
        extract_msg.bounding_boxes = m_BoundingBoxes;
        m_ExtractKeyPointsPublisher.publish(extract_msg);

        //read and publish image
        cv_bridge::CvImage gray_image, color_image;
        gray_image.image = cv::imread(fileName.c_str(), CV_LOAD_IMAGE_GRAYSCALE);
        gray_image.encoding = sensor_msgs::image_encodings::TYPE_8UC1;
        color_image.image = cv::imread(fileName.c_str(), CV_LOAD_IMAGE_COLOR);
        color_image.encoding = sensor_msgs::image_encodings::TYPE_8UC3;

        or_msgs::OrImage or_image_msg;
        or_image_msg.image_gray = *(gray_image.toImageMsg());
        or_image_msg.image_color = *(color_image.toImageMsg());
        or_image_msg.filename = fileName;
        m_DebugImagePublisher.publish(or_image_msg);

        m_ImagesInPipeline++;
        m_Continuous = false;

        // pass loaded image directly to the matching module
        m_ORMatchingModule->processImageMessage(color_image.toImageMsg());

        break;
    }

    case ORControlModule::StartRecognitionLoop:
    {
        m_SourceId = or_command_msg->int_value; // TODO ImageSources::SourceId(message->getInt());
        for ( int i=0; i<m_MaxImagesInPipeline; i++ )
        {
            // send message for keypoint extraction
            or_msgs::ExtractKeyPoints extract_msg;
            extract_msg.img_source = m_SourceId;
            extract_msg.bounding_boxes = m_BoundingBoxes;
            m_ExtractKeyPointsPublisher.publish(extract_msg);
            m_ImagesInPipeline++;
        }
        m_Continuous = true;
        break;
    }

    case ORControlModule::StopRecognitionLoop:
    {
        m_Continuous = false;
        break;
    }

    default:
        break;
    }
}

void THIS::callbackOrMatchResult( const or_msgs::OrMatchResult::ConstPtr& or_match_result_msg)
{
    // TODO only for debug
    if(or_match_result_msg->match_results.size() == 0)
        ROS_ERROR_STREAM("no objects recognized");
    else
    {
           ROS_ERROR_STREAM("recognized objects: ");
    for(unsigned i = 0; i < or_match_result_msg->match_results.size(); i++)
        ROS_ERROR_STREAM(or_match_result_msg->match_results.at(i).object_name);
    }


    if ( m_Continuous )
    {
        // send message for keypoint extraction
        or_msgs::ExtractKeyPoints extract_msg;
        extract_msg.img_source = m_SourceId;
        extract_msg.bounding_boxes = m_BoundingBoxes;
        m_ExtractKeyPointsPublisher.publish(extract_msg);
        m_ImagesInPipeline++;
    }
    m_ImagesInPipeline--;
}


#undef THIS
