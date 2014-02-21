/*******************************************************************************
*  ObjectImagesControl.h
*
*  (C) 2008 AG Aktives Sehen <agas@uni-koblenz.de>
*           Universitaet Koblenz-Landau
*
*  §Author: SG;
*
*******************************************************************************/



#ifndef ObjectImagesControl_H
#define ObjectImagesControl_H

#include <ros/ros.h>

#include <QObject>
#include <QWidget>
#include <QTableWidget>
#include <QRadioButton>
#include <QComboBox>
#include <vector>
#include <QKeyEvent>

// #include "Workers/ObjectRecognition/ImageProperties.h" // TODO delete this

class QTableWidget;
class QLineEdit;
class QPushButton;

/**
 * @class  ObjectImagesControl
 * @author Simon Graeser (RX), David Gossow (RX)
 * @brief  List of images used in object learning
 */
class ObjectImagesControl : public QWidget
{

    Q_OBJECT

  public:
    /**
     * Instantiates its tab widget.
     * @param [in] parent The QWidget the contains this QWidget.
     */

    ObjectImagesControl ( ros::NodeHandle *nh, QWidget *parent = 0 );
    /**
       *Does Nothing
     **/
    ~ObjectImagesControl();
     void updateImageTable(std::vector<std::string> &imageNames, std::string &objType);

  protected:
    void keyReleaseEvent(QKeyEvent*);

  private slots:
    void saveImage( );
    void removeImage();
    void saveObject();
    void rowSelected ( int, int );
    void objectNameFieldEdited ( QString );
    void setObjectType ( QString );
    void loadObject( );

  private:

    void updateTable();

    ros::Publisher m_ORLearnCommandPublisher;

    QLineEdit* m_ObjectNameField;
    QTableWidget* m_ImageTable;

    QLineEdit* m_ImageNameField;

    QString m_LastImageFolder;

    QPushButton* m_saveObjectButton;
    QPushButton* m_removeImageButton;
    QPushButton* m_resetImagesButton;

    QComboBox* m_TypeComboBox;

    int m_SelectedRow;

    // set to true after object is constructed, used to avoid publishing on ros topics in slots too early
    bool m_Ready;
};

#endif
