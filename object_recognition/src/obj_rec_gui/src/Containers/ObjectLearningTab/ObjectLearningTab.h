/*******************************************************************************
*  ObjectLearningTab.h
*
*  (C) 2006 AG Aktives Sehen <agas@uni-koblenz.de>
*           Universitaet Koblenz-Landau
*
*  Information on Code Review state:
*  §Author: RH§
*
*  Additional information:
*  $Id: ObjectLearningTab.h 24108 2008-04-10 11:49:43Z rhofmann $
*******************************************************************************/

#ifndef ObjectLearningTab_H
#define ObjectLearningTab_H

#include <set>
#include <QTreeWidget>

#include <ros/ros.h>
#include <sensor_msgs/Image.h>

#include "../../Widgets/ObjectRecognition/ObjectImagesControl.h"
#include "../../Widgets/ObjectRecognition/ObjectLearningControl.h"

class ImageStreamDisplay;

/**
 * @class  ObjectLearningTab
 * @author David Gossow (RX)
 * @brief  Interface for learning new objects from camera images or files
 */
class ObjectLearningTab : public QWidget {

    Q_OBJECT

  public:

    /**
    * Constructs all GUI Elements and connects the Signals and Slots.
    */
    ObjectLearningTab(ros::NodeHandle *nodeHandle, QWidget *parent = 0 );

    /** Does nothing. */
    ~ObjectLearningTab() {}

    /** @brief process incoming messages */
    void updateCameraImage(const unsigned char* image, unsigned width, unsigned height);

  public slots:
    void processLearningStatus(std::vector<std::string> filenames, std::string objType);

  private:


    ObjectImagesControl* m_ImagesControl;
};


#endif
