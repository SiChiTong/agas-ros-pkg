#ifndef TALKER_H
#define TALKER_H

#include "ros/ros.h"

#include <QObject>
#include <QString>

class Talker : public QObject {

  Q_OBJECT

public:

  Talker(ros::NodeHandle nh, const char* topic= "text_out");
  void sendMessage(QString text);

protected:

  ros::Publisher* m_Publisher;
};

#endif
