#ifndef QT_ROS_NODE_H
#define QT_ROS_NODE_H


#include <QThread>
#include <QObject>
#include <QApplication>

//messages
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <map_messages/PointsOfInterest.h>
#include <sensor_msgs/LaserScan.h>

#include "Workers/PointOfInterest/PointOfInterest.h"
#include "Architecture/StateMachine/StateMachine.h"
#include <vector>

//#include <or_msgs/OrImage.h>
#include <or_msgs/OrLearningStatus.h>
#include <or_msgs/OrObjectNames.h>

#include <ros/ros.h>
//#include <ros/callback_queue.h>

class MainWindow;

class QtRosNode : public QThread {

  Q_OBJECT

  public:
    ///Note: The constructor will block until connected with roscore
    ///Instead of ros::spin(), start this thread with the start() method
    ///to run the event loop of ros
    QtRosNode(int argc, char *argv[], const char* node_name, MainWindow* mainWindow);

    ~QtRosNode();

    ros::NodeHandle* getNodeHandle(){ return m_NodeHandle; }

    /// This method contains the ROS event loop. Feel free to modify
    void run();

    enum ModuleStateT {
      IDLE,
      WAITING,
      DOOR_IS_OPEN,
    };


  public slots:
    ///Connect to aboutToQuit signals, to stop the thread
    void quitNow();

  signals:
    ///Triggered if ros::ok() != true
    void rosShutdown();
    void poseUpdated(double x, double y, double theta);
    void mapUpdated(unsigned char * pMap, unsigned size,
                    double resolution, geometry_msgs::Pose origin,
                    int minX, int minY, int maxX, int maxY);
    void poiListUpdated(std::vector < map_messages::PointOfInterest > poiList);
    void pathUpdated(std::vector<geometry_msgs::PoseStamped> path);
    void learningStatus(std::vector<std::string> filenames, std::string objType);
    void objectNames(std::vector<std::string> names, std::vector<std::string> types);

  private:

    bool quitfromgui;

    ros::NodeHandle* m_NodeHandle;
    MainWindow* m_MainWindow;
    QApplication* m_Application;

    ros::Subscriber m_TextSubscriber;
    ros::Subscriber m_LaserSubscriber;
    ros::Subscriber m_RgbImageSubscriber;

    ros::Subscriber m_PoseStampedSubscriber;
    ros::Subscriber m_MapDataSubscriber;
    ros::Subscriber m_POIsSubscriber;
    ros::Subscriber m_PathSubscriber;

    ros::Subscriber m_OLPrimaryImageSubscriber;
    ros::Subscriber m_ORLearningStatusSubscriber;
    ros::Subscriber m_ORObjectNamesSubscriber;

    void subscribeToTopics();
    void advertiseTopics();

    //callbacks
    void callbackPoseStamped(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void callbackOccupancyGrid(const nav_msgs::OccupancyGrid::ConstPtr& msg);
    void callbackPOIList(const map_messages::PointsOfInterest::ConstPtr& msg);
    void callbackLaserScan(const sensor_msgs::LaserScan::ConstPtr& msg);

    //void callbackPrimaryImage(const or_msgs::OrImage::ConstPtr& msg );
    //void callbackGUIImage(const sensor_msgs::Image::ConstPtr &msg);
    void callbackLearningStatus(const or_msgs::OrLearningStatusConstPtr &msg);
    void callbackObjectNames(const or_msgs::OrObjectNamesConstPtr &msg);

    StateMachine<ModuleStateT> m_ModuleMachine;

    // door detection
    std::string m_ExtraStatusInfo;
    std::vector<int> m_Ranges;
    unsigned int m_InitTimestamp;
    unsigned int m_CurrentTimestamp;
    unsigned int m_NewTimestamp;
    int m_InitRange;
    int m_CurrentRange;
    int m_NewRange;
    int m_GameTime;
    void compareRanges();
    //end door detection


};
#endif
