#ifndef Listener_H
#define Listener_H

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64.h>
#include <std_msgs/UInt8.h>
#include <sensor_msgs/Imu.h>

#include "door_detection_messages/Door.h"
#include "door_detection_messages/DoorHandle.h"

#include <QObject>
#include <QString>

namespace Ui {
    class MainWindow;
}

namespace cv {
    class Mat;
}

class QImage;

class Listener : public QObject {

  Q_OBJECT

public:

  Listener(ros::NodeHandle nh);

  float getTiltAngle() const {return m_TiltAngle;}
  float getTiltStatus() const {return m_TiltStatus;}
  geometry_msgs::Vector3& getImu() const {return m_Imu;}

private:

  void curTiltAngleCallback(const std_msgs::Float64 msg);
  void curTiltStatusCallback(const std_msgs::UInt8 msg);
  void imuCallback(const sensor_msgs::ImuConstPtr& msg);

  ros::Subscriber m_CurTiltAngleSubscriber;
  ros::Subscriber m_CurTiltStatusSubscriber;

  float m_TiltAngle;
  int m_TiltStatus;

  geometry_msgs::Vector3 m_Imu;

};

#endif
