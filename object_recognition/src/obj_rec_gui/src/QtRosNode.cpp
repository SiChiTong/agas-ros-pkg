
#include <tf/tf.h>
#include <std_msgs/String.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <map_messages/PointOfInterest.h>
#include <map_messages/DeletePointOfInterest.h>
#include <map_messages/PointsOfInterest.h>

#include "QtRosNode.h"
#include "Workers/PointOfInterest/PointOfInterest.h"

QtRosNode::QtRosNode(int argc, char *argv[], const char* node_name, MainWindow* mainWindow)
{
  ros::init(argc, argv, node_name);
  ros::start();
  m_NodeHandle = new ros::NodeHandle;
  m_MainWindow = mainWindow;

  //m_Application = app;
  quitfromgui = false;


  advertiseTopics();
  subscribeToTopics();

  start();


  ADD_MACHINE_STATE( m_ModuleMachine, IDLE );
  ADD_MACHINE_STATE( m_ModuleMachine, WAITING );
  ADD_MACHINE_STATE( m_ModuleMachine, DOOR_IS_OPEN );


  m_ModuleMachine.setName( "Module State" );
  m_ModuleMachine.setState( IDLE );

}

QtRosNode::~QtRosNode()
{
    if( m_NodeHandle ) delete m_NodeHandle;
    if(ros::isStarted()) {
      ros::shutdown(); // explicitly needed since we use ros::start();
      ros::waitForShutdown();
    }
    wait();
}


void QtRosNode::compareRanges()
{
  //switch (m_ModuleMachine.state() )
  //{
    //case WAITING:
    //{
      //int ranges_center = ( m_Ranges.size() / 2 ) + 1;
      ////- m_NewRange = m_Ranges.at( ranges_center );
      //m_NewRange = (m_Ranges.at( ranges_center -1 )        // mittelwert der mittleren 3 laser, noiseresistent
          //+ m_Ranges.at( ranges_center )
          //+ m_Ranges.at( ranges_center +1 ))/3;
      //TRACE_INFO( "NewRange: " << m_NewRange );
      //m_CurrentTimestamp = m_NewTimestamp;

      //if (( m_NewRange > m_CurrentRange + 1000 ) && ( m_CurrentRange <= 2000)) // Tür nicht weiter als 2m weg, um hintergrundbewegungen zu ignorieren
      //{
        //usleep( 1000000 );
        //ROS_INFO_STREAM( "The door is open!" );
        ////sendMessage( new SpeechOutM( "The door is now open" ) );
        ////sendMessage( new StartGameM( m_GameTime ) );
        //m_ModuleMachine.setState( DOOR_IS_OPEN );
      //}
      //else if (m_NewRange < m_CurrentRange)   // falls tür nach dem startbutton geschlossen wird
      //{
        //m_CurrentRange = m_NewRange;
      //}
      //break;
    //}
    //default:
      //break;
  //}
}


void QtRosNode::subscribeToTopics()
{
//    // subscribe to all topics here
    m_LaserSubscriber = m_NodeHandle->subscribe<sensor_msgs::LaserScan>("/scan", 10, &QtRosNode::callbackLaserScan, this);
//   // m_RgbImageSubscriber = m_NodeHandle->subscribe<sensor_msgs::Image>("/camera/rgb/image_color", 10, &MainWindow::callbackRgbImage, m_MainWindow);
    m_PoseStampedSubscriber = m_NodeHandle->subscribe<geometry_msgs::PoseStamped>("/pose", 10, &QtRosNode::callbackPoseStamped, this);
    m_MapDataSubscriber = m_NodeHandle->subscribe<nav_msgs::OccupancyGrid>("/map", 10, &QtRosNode::callbackOccupancyGrid, this);
    m_POIsSubscriber = m_NodeHandle->subscribe<map_messages::PointsOfInterest>("/map_manager/poi_list", 10, &QtRosNode::callbackPOIList, this);

    //m_OLPrimaryImageSubscriber = m_NodeHandle->subscribe<map_messages::PointsOfInterest>("/or/obj_learn_primary", 10, &QtRosNode::callbackPOIList, this);
    //m_OLPrimaryImageSubscriber = m_NodeHandle->subscribe<sensor_msgs::Image>("/or/obj_learn_primary", 10, &QtRosNode::callbackGUIImage, this);
    //m_OLPrimaryImageSubscriber = m_NodeHandle->subscribe<or_msgs::OrImage>("/or/obj_learn_primary", 10, &QtRosNode::callbackPrimaryImage, this);
    m_ORLearningStatusSubscriber = m_NodeHandle->subscribe<or_msgs::OrLearningStatus>("/or/learning_status", 10, &QtRosNode::callbackLearningStatus, this);
    m_ORObjectNamesSubscriber = m_NodeHandle->subscribe<or_msgs::OrObjectNames>("or/obj_names", 10, &QtRosNode::callbackObjectNames, this);
}

void QtRosNode::advertiseTopics()
{
//    // TODO advertise topics here or find other solution
//    ros::Publisher* text_publisher = new ros::Publisher(m_NodeHandle->advertise<std_msgs::String>("chatter", 10));
//    m_MainWindow->setTextPublisher(text_publisher);
//    ros::Publisher* pose2D_publisher = new ros::Publisher(m_NodeHandle->advertise<geometry_msgs::Pose2D>("/userdef_pose", 10));
//    m_MainWindow->setPose2DPublisher(pose2D_publisher);
//    ros::Publisher* addPointOfInterest_publisher = new ros::Publisher(m_NodeHandle->advertise<map_messages::PointOfInterest>("/add_POI", 10));
//    m_MainWindow->setAddPOIPublisher(addPointOfInterest_publisher);
//    ros::Publisher* modifyPointOfInterest_publisher = new ros::Publisher(m_NodeHandle->advertise<map_messages::PointOfInterest>("/modify_POI", 10));
//    m_MainWindow->setModifyPOIPublisher(modifyPointOfInterest_publisher);
//    ros::Publisher* deletePointOfInterest_publisher = new ros::Publisher(m_NodeHandle->advertise<map_messages::DeletePointOfInterest>("/delete_POI", 10));
//    m_MainWindow->setDeletePOIPublisher(deletePointOfInterest_publisher);
}


void QtRosNode::quitNow()
{
  quitfromgui = true;
}


void QtRosNode::run()
{
  ros::Rate loop_rate(10);

  while (ros::ok())
  {
    ros::spinOnce();
    loop_rate.sleep();
  }
  emit rosShutdown();

}

void QtRosNode::callbackPoseStamped(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    double theta = tf::getYaw(msg->pose.orientation);
    emit poseUpdated(msg->pose.position.x, msg->pose.position.y, theta);
}

void QtRosNode::callbackOccupancyGrid(const nav_msgs::OccupancyGrid::ConstPtr &msg)
{

   unsigned char* map = new unsigned char[msg->data.size()];
    //generate exploredRegion
   // TODO bounding box should not be MIN/MAX in first round
    int minX = INT_MIN;
    int minY = INT_MIN;
    int maxX = INT_MAX;
    int maxY = INT_MAX;
    for(int x = 0; x < msg->info.height; x++)
    {
        int xOffset = msg->info.height * x;
        for(int y = 0; y < msg->info.width; y++)
        {
            int i = xOffset + y;
            map[i] = msg->data[i] == -1 ? 50 : msg->data[i];
            if(map[i]!=-1) {
                if(minX==INT_MIN || minX > x)
                    minX = x;
                if(minY==INT_MIN || minY > y)
                    minY = y;
                if(maxX==INT_MAX || maxX < x)
                    maxX = x;
                if(maxY==INT_MAX || maxY < y)
                    maxY = y;
            }
        }
    }

    emit mapUpdated(map, msg->info.width, msg->info.resolution, msg->info.origin, minX, minY, maxX, maxY);
}

void QtRosNode::callbackPOIList(const map_messages::PointsOfInterest::ConstPtr& msg)
{
    emit poiListUpdated(msg->pois);

}



//void QtRosNode::callbackPrimaryImage(const or_msgs::OrImage::ConstPtr &msg ) {
//    ROS_INFO_STREAM("callbackPrimaryImage");
//}

//void QtRosNode::callbackGUIImage(const sensor_msgs::Image::ConstPtr& msg)
//{

//}


void QtRosNode::callbackLaserScan(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    //m_Ranges.clear();
    //switch ( m_ModuleMachine.state() )
    //{
        //case IDLE:
            //{
                //if ( laserDataM->getScannerConfig()->getName() == "SickLMS100" )
                //{
                    //m_Ranges = laserDataM->getRangeVector();
                    //int ranges_center = ( m_Ranges.size() / 2 ) + 1;
                    ////- m_CurrentRange = m_Ranges.at( ranges_center );
                    //m_CurrentRange = (m_Ranges.at( ranges_center -1 )        // mittelwert der mittleren 3 laser, noiseresistent
                            //+ m_Ranges.at( ranges_center )
                            //+ m_Ranges.at( ranges_center +1 ))/3;

                //}
                //break;
            //}
        //case WAITING:
            //{
                //string LRFName = laserDataM->getScannerConfig()->getName();
                //m_NewTimestamp = laserDataM->getTimestamp();
                //if ( LRFName == "SickLMS100" && m_NewTimestamp > m_CurrentTimestamp + 500 )
                //{
                    //m_Ranges = laserDataM->getRangeVector();
                    //compareRanges();
                //}
                //break;
            //}
        //default:
            //break;
    //}





    //    if(m_SensorDataDisplay)
    //    {
    //         NewLaserDataPainter* painter = dynamic_cast<NewLaserDataPainter*> (m_SensorDataDisplay->getSensorDataGLWidget()->getPainter("2D Laser Data"));
    //        // TODO convert laser data to world coordinates here or in painter
    //        if(painter)
    //        {
    //            painter->updateData(msg);
    //        }
    //        else
    //        {
    //            ROS_WARN_STREAM("Could not get Painter Object \"2D Laser Data\"");
    //        }
    //    }
}

void QtRosNode::callbackLearningStatus(const or_msgs::OrLearningStatusConstPtr &msg)
{
    emit learningStatus(msg->image_names, msg->object_type);
}

void QtRosNode::callbackObjectNames(const or_msgs::OrObjectNamesConstPtr &msg)
{
    emit objectNames(msg->object_names, msg->object_types);
}
