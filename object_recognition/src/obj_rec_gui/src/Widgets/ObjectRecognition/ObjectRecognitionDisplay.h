/*******************************************************************************
 *  ObjectRecognitionDisplay.h
 *
 *  (C) 2006 AG Aktives Sehen <agas@uni-koblenz.de>
 *           Universitaet Koblenz-Landau
 *
 *  Additional information:
 *  $Id: ObjectRecognitionDisplay.h 23656 2008-03-30 18:21:56Z dgossow $
 *******************************************************************************/

#ifndef ObjectRecognitionDisplay_H
#define ObjectRecognitionDisplay_H

#include <ros/ros.h>

#include <QGLWidget>

#include <list>
#include <vector>
#include <map>

// #include "Architecture/Profiler/Timer.h" // TODO

#include "Workers/Math/Box2D.h"
#include "Workers/ImageSources/ImageSources.h"
#include "KeyPointExtraction/KeyPoint.h"
#include "ObjectRecognition/MatchResult.h"
#include "Workers/Puma2/ColorImageRGB8.h"
#include "Workers/Puma2/ImageMask.h"


class QLabel;
class QGridLayout;
class ImageM;
class GLImageWidget;
class QCheckBox;
class QVBoxLayout;
class QBoxLayout;
class QTableWidget;
class QPushButton;
class QGroupBox;
class QComboBox;
class QWidget;

/**
 * @brief Widget displaying information about the object recognition process
 * @author David Gossow (RX)
 */

class ObjectRecognitionDisplay : public QGLWidget {

  Q_OBJECT

  public:

    ObjectRecognitionDisplay(ros::NodeHandle *nh, QBoxLayout* checkBoxLayout, QWidget* parent = 0 );
    ~ObjectRecognitionDisplay();

    void updateObjectTable(std::vector<std::string> names, std::vector<std::string> types);

  public slots:

    /** @brief process incoming messages */
   // void processMessage( Message* message );

    void optionsChanged();

    void cellClicked(int row,int column);

    void setCameraId( ImageSources::SourceId cameraId );

    /**
    *Used to delete objects
    **/
    void deleteObject();

    void grabImage();
    void loadImage();
    void startLoop();
    void stopLoop();

    /**
     * Used to load objects from disk
    **/
    void loadObjectDialog();
    void loadObject(std::string file);

  protected:

  private:

    enum Stage3MatcherT
    {
      HomographyMat=1,
      FundamentalMat=2
    };

    void updateDisplay( );
    void updateMatches( MatchResult& matchResult, std::list<KeyPointMatch> &matches );

    void addCheckBox( QBoxLayout* checkBoxLayout, std::map<std::string,QCheckBox*>& CBMap, std::string id, std::string label, bool checked=true );
    void initBinChooser();

    GLImageWidget* m_GlImageWidget;

    // TODO
    //Timer m_Timer;

    //Object Table
    QTableWidget* m_ObjectsList;
    //contains row of item mapped to name
    std::map< std::string, int > m_ObjectRows;
    int m_SelectedRow;

    QPushButton* m_DeleteObjectButton;
    QGroupBox* m_ObjectsGroupBox;

    //check boxes
    std::map< std::string, QCheckBox* > m_OptionCheckBoxes;
    QVBoxLayout* m_ObjCheckboxLayout;
    QComboBox* binChooserComboBox;

    //path for open dialog
    QString m_LastOpenPath;

    //cam id selector
    ImageSources::SourceId m_CameraId;

    //match data
    std::vector< KeyPoint > m_SceneKeyPoints;
    std::vector<MatchResult> m_MatchResults;
		std::vector< Box2D<int> > m_BoundingBoxes;

    Stage3MatcherT m_Stage3Matcher;

    ros::Publisher m_ORCommandPublisher;

    // set to true after object is constructed, used to avoid publishing on ros topics in slots too early
    bool m_Ready;
};

#endif
