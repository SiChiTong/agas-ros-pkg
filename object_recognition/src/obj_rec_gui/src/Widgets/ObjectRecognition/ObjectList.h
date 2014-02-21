/*******************************************************************************
*  ObjectList.h
*
*  (C) 2008 AG Aktives Sehen <agas@uni-koblenz.de>
*           Universitaet Koblenz-Landau
*
*  §Author: SG;
*
*******************************************************************************/



#ifndef ObjectList_H
#define ObjectList_H

#include <QObject>
#include <QWidget>

class QPushButton;
class QTableWidget;
class QLabel;
class ORObjectPropertiesM;

//class GLImageWidget;

//DEBUG
class SpeechOutM;
/**
 * @class ObjectList
 *
 * @author Simon Graeser
 *
 * @brief Implements a widget to control the RoboCup@Home game "Lost and Found". TODO Why this comment??
 *
 */
class ObjectList : public QWidget {

    Q_OBJECT

  public:

   /**
    *
    */
    ObjectList(QWidget *parent = 0);

   /**
    *Does Nothing
    **/
    ~ObjectList();

  public slots:

    /**
    *Used to load objects from disk
    **/
    void loadObject();
    /**
    *Used to delete objects
    **/
    void deleteObject();

   // /** @brief Process incoming messages */
   // virtual void processMessage( Message* newMessage );

  signals:

  private:

    QTableWidget* m_objectsList;
    QPushButton* deleteObjectButton;
    QLabel* heading;

    QString m_LastOpenPath;

};

#undef THIS
#endif
