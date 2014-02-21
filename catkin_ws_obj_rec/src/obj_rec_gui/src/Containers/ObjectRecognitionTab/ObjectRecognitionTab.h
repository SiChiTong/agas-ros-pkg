/*******************************************************************************
*  ObjectRecognitionTab.h
*
*  (C) 2006 AG Aktives Sehen <agas@uni-koblenz.de>
*           Universitaet Koblenz-Landau
*
*  Information on Code Review state:
*  §Author: RH§
*
*  Additional information:
*  $Id: ObjectRecognitionTab.h 24108 2008-04-10 11:49:43Z rhofmann $
*******************************************************************************/

#ifndef ObjectRecognitionTab_H
#define ObjectRecognitionTab_H

#include <set>
#include <QTreeWidget>

#include <ros/ros.h>

#include "../../Widgets/ObjectRecognition/ObjectRecognitionDisplay.h"

/**
 * @class  ObjectRecognitionTab
 * @author David Gossow (RX)
 * @brief  Controls & displays for the object recognition subsystem
 */
class ObjectRecognitionTab : public QWidget {

    Q_OBJECT

  public:

    /**
    * Constructs all GUI Elements and connects the Signals and Slots.
    */
    ObjectRecognitionTab(ros::NodeHandle *nodeHandle, QWidget *parent = 0 );

    /** Does nothing. */
    ~ObjectRecognitionTab() {}

  public slots:

    void processObjectNames(std::vector<std::string> names, std::vector<std::string> types);

private:
    ObjectRecognitionDisplay* m_ObjectRecognitionDisplay;
};


#endif
