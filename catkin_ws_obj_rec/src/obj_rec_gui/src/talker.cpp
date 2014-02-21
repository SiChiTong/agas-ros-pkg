#include "talker.h"

#include <std_msgs/String.h>

Talker::Talker(ros::NodeHandle nodeHandle, const char* topic)
{
  m_Publisher = new ros::Publisher(nodeHandle.advertise<std_msgs::String>("robot_face/text_out", 1000));
}

void Talker::sendMessage(QString text)
{
  std_msgs::String msg;

  std::stringstream ss;
  ss <<  text.toStdString();
  msg.data = ss.str();

  ROS_INFO("%s", msg.data.c_str());

  m_Publisher->publish(msg);
}
