/*******************************************************************************
*  ObjectLearningControl.h
*
*  (C) 2008 AG Aktives Sehen <agas@uni-koblenz.de>
*           Universitaet Koblenz-Landau
*
*  §Author: SG;
*
*******************************************************************************/



#ifndef LEARNING_H
#define LEARNING_H

#include <ros/ros.h>

#include <QObject>
#include <QWidget>
#include <QRadioButton>

#include "Widgets/GLImageWidget/GLImageWidget.h"

//#include "../../Workers/Puma2/ImageWriter.h" // TODO
#include "Workers/Puma2/ColorImageRGB8.h"
#include "Workers/Puma2/GrayLevelImage8.h"
#include "Workers/Puma2/ColorToGrayOperator.h"
#include "Workers/Puma2/ColorImageYUV8.h"
#include "Workers/ImageSources/ImageSources.h"
#include "Workers/Math/Point2D.h"


class QPushButton;
class QListWidget;
class QLineEdit;
class QSlider;
class QLabel;

/**
 * @class ObjectLearningControl
 *
 * @author Simon Graeser
 *
 * @brief Implements a widget to control the creation of new templates for object recognition
 *
 */
class ObjectLearningControl : public QWidget {

  Q_OBJECT

  public:
  /**
   * Instantiates its tab widget.
   * @param [in] parent The QWidget the contains this QWidget.
   */

    ObjectLearningControl(ros::NodeHandle *nh, QWidget *parent = 0);
  /**
     *Does Nothing
   **/
    ~ObjectLearningControl();

  public slots:

    void setThreshold( int );
    void setOpenRadius( int );
    void setBorderSize(int value);
    void setIsolateLargestSegment( int state );

    /** @brief request image and wait */
    void grabBackgroundImage();
    /** @brief request image from camera */
    void grabForegroundImage();

    /** @brief request image and wait */
    void loadBackgroundImage();
    /** @brief request image from camera */
    void loadForegroundImage();

    void setCameraId( ImageSources::SourceId cameraId );

  signals:

    void imageSaved( QString fileName );

  private:

    ros::Publisher m_ORLearnCommandPublisher;

    QString m_LastImageFolder;

    QPushButton* grabBackgroundButton;
    QPushButton* grabForegroundButton;

    QPushButton* loadBackgroundButton;
    QPushButton* loadForegroundButton;

    ImageSources::SourceId m_CameraId;

    //path for open dialog
    QString m_LastOpenPath;

    // set to true after object is constructed, used to avoid publishing on ros topics in slots too early
    bool m_Ready;
};

#endif
