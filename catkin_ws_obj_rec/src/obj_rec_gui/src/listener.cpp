#include "listener.h"

#include <std_msgs/String.h>
#include <ros/package.h>
#include <QtCore>


Listener::Listener(ros::NodeHandle nodeHandle)
{
  m_CurTiltAngleSubscriber = nodeHandle.subscribe( "cur_tilt_angle", 1, &Listener::curTiltAngleCallback, this );
  m_CurTiltStatusSubscriber = nodeHandle.subscribe( "cur_tilt_status", 1, &Listener::curTiltStatusCallback, this );
}


void Listener::curTiltAngleCallback(const std_msgs::Float64 msg)
{
  m_TiltAngle = msg.data;
}

void Listener::curTiltStatusCallback(const std_msgs::UInt8 msg)
{
  m_TiltStatus = msg.data;
}

void Listener::imuCallback(const sensor_msgs::ImuConstPtr& msg)
{

  m_Imu.x = msg->orientation.x;
  m_Imu.y = msg->orientation.y;
  m_Imu.z = msg->orientation.z;

}

