/*******************************************************************************
 *  ORMatchingModule.h
 *
 *  (C) 2006 AG Aktives Sehen <agas@uni-koblenz.de>
 *           Universitaet Koblenz-Landau
 *
 *******************************************************************************/

#ifndef ORMatchingModule_H
#define ORMatchingModule_H

#include <deque>

#include <ros/ros.h>
#include <or_msgs/OrCommand.h>
#include <or_msgs/ExtractKeyPoints.h>
#include <or_msgs/OrImage.h>

#include "KeyPointExtraction/KeyPoint.h"
#include "ObjectRecognition/ObjectProperties.h"
#include "ObjectRecognition/MatchResult.h"
#include "KeyPointExtraction/KeyPointMatch.h"

#include "ObjectRecognition/ImagePropertiesCV.h"

#include "Workers/Math/Box2D.h"

class FLANNMatcher;

class KeyPointExtractor;

/**
 * @class  ORMatchingModule
 * @brief  Default module template
 * @author David Gossow (RX), Viktor Seib (R20)
 */
class ORMatchingModule
{
  public:

    /** @brief The constructor. */
    ORMatchingModule(ros::NodeHandle *nh, std::string inputTopic);

    /** @brief The destructor. */
    virtual ~ORMatchingModule();

    void addObjectProperties(ObjectProperties* newProperties);

    void callbackOrCommand( const or_msgs::OrCommand::ConstPtr& msg );
    void callbackExtractKeyPoints( const or_msgs::ExtractKeyPoints::ConstPtr& msg );
    void callbackImage( const sensor_msgs::Image::ConstPtr& message );

    void processImageMessage( const sensor_msgs::Image::ConstPtr& message );

  private:

    enum Stage1MatcherT
    {
      NearestNeighbor=1,
      Flann=2
    };

    enum Stage2MatcherT
    {
      SimpleHoughClustering=1,
      HoughClustering=2
    };

        /// @brief extract all keypoints, then do object recognition on all bounding boxes separately
        /// @param boundingBoxes bounding boxes to use. if empty, the full image is used.
    void processImages(  cv::Mat* image, int seqNum, std::vector< Box2D<int> > boundingBoxes=std::vector< Box2D<int> >() );

        /// @brief search for objects on all given keypoints
    void processKeyPoints( cv::Mat* image, std::vector< KeyPoint > *rawKeyPoints, std::vector< Box2D<int> > boundingBoxes, unsigned seqNum );

        /// @brief try to match keypoints to one specific object
    bool matchObject( std::vector< KeyPoint >* sceneKeyPoints, ObjectProperties& objectProperties, MatchResult& matchResult );

    /* Match keypoints by nearest neighbor ratio */
    std::list< KeyPointMatch > matchStage1( std::vector< KeyPoint > *sceneKeyPoints, ImagePropertiesCV *objectImageProperties );

    /* Match keypoints with flann */
    std::list< KeyPointMatch > matchStage1Flan(ImagePropertiesCV *objectImageProperties );

    /* Hough-Transform-Clustering */
    std::vector< std::list< KeyPointMatch> > matchStage2( std::vector< KeyPoint > *sceneKeyPoints, ImagePropertiesCV *objectImageProperties, std::list< KeyPointMatch > &stage1Matches );

    /* Homography */
    std::list< KeyPointMatch > matchStage3( std::vector< KeyPoint > *sceneKeyPoints,
        ImagePropertiesCV *objImageProperties,
        std::vector< std::list< KeyPointMatch> > &stage2Matches,
        Homography &homography );

    /* helper function */
    std::vector<KeyPoint> getSceneKeyPointsWithinOutline(std::vector< KeyPoint >* sceneKeyPoints, Homography& homography, std::list< KeyPointMatch >& stage3Matches);

    void removeObjectProperties(std::string name);
    void sendObjectNames();

    std::deque<ObjectProperties> m_ObjectList;

    int m_ImagesRequested;
    std::vector< Box2D<double> > m_BoundingBoxes;

    Stage1MatcherT m_Stage1Matcher;
    Stage2MatcherT m_Stage2Matcher;

    int m_ImageWidth;
    int m_ImageHeight;

    KeyPointExtractor* m_Extractor;

    std::list<double> m_ProcessingTimes;

    FLANNMatcher* m_FlannMatcher;

    ros::Subscriber m_OrCommandSubscriber;
    ros::Subscriber m_ExtractKeyPointsSubscriber;
    ros::Subscriber m_ImageSubscriber;

    ros::Publisher m_ORMatchResultPublisher;
    ros::Publisher m_ORObjectNamesPublisher;
    ros::Publisher m_DebugImagePublisherColor;

};


#endif
