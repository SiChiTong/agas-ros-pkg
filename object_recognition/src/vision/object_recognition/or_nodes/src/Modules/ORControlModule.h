/*******************************************************************************
 *  ORControlModule.h
 *
 *  (C) 2006 AG Aktives Sehen <agas@uni-koblenz.de>
 *           Universitaet Koblenz-Landau
 *
 *******************************************************************************/

#ifndef ORControlModule_H
#define ORControlModule_H

#include <ros/ros.h>
#include <or_msgs/OrCommand.h>
#include <or_msgs/OrMatchResult.h>

class ORMatchingModule;

/**
 * @class  ORControlModule
 * @brief  Controls the Object Recognition Process
 * @author David Gossow (RX), Viktor Seib (R20)
 */
class ORControlModule
{
  public:

    enum CommandId {
      LoadObject,
      UnloadObject,
      LoadSingleImage,
      GrabSingleImage,
      StartRecognitionLoop,
      StopRecognitionLoop
    };

    enum ValueT {
      NoValue,
      FloatValue,
      StringValue,
      IntValue
    };

    /** @brief The constructor. */
    ORControlModule(ros::NodeHandle *nh, ORMatchingModule* objRecMatchingModule);

    /** @brief The destructor. */
    virtual ~ORControlModule();

  private:

    void callbackOrCommand( const or_msgs::OrCommand::ConstPtr& msg );
    void callbackOrMatchResult( const or_msgs::OrMatchResult::ConstPtr& msg);

    int m_ImagesInPipeline;
    int m_MaxImagesInPipeline;

    bool m_Continuous;

    ORMatchingModule* m_ORMatchingModule;

    int m_SourceId; // TODO ImageSources::SourceId m_SourceId;
    std::vector<or_msgs::BoundingBox2D> m_BoundingBoxes;

    ros::Subscriber m_ORCommandSubscriber;
    ros::Subscriber m_ORMatchResultSubscriber;

    ros::Publisher m_ExtractKeyPointsPublisher;
    ros::Publisher m_DebugImagePublisher;

    ros::Publisher m_DebugImagePublisherGray;
    ros::Publisher m_DebugImagePublisherColor;
};

#endif

