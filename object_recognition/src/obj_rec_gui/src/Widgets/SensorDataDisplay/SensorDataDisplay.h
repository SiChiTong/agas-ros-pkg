/*******************************************************************************
 *  SensorDataDisplay.h
 *
 *  (C) 2006 AG Aktives Sehen <agas@uni-koblenz.de>
 *           Universitaet Koblenz-Landau
 *
 *  Information on Code Review state:
 *  ï¿½Author: SW; DevelTest: 10.04.06; Reviewer: MS; Review: 09.05.06 ; State:NOKï¿½
 *
 *  Additional information:
 *  $Id: SensorDataDisplay.h 45119 2011-05-31 16:06:07Z robbie $
 *******************************************************************************/

#ifndef SENSORDATADISPLAY_H
#define SENSORDATADISPLAY_H

#include <QtGui>

class QLayout;
class QDialog;
class QComboBox;
class QWidget;

class Pose;
class SensorDataGLWidget;
class PainterPlugin;
class NewLaserDataPainter;

/**
 * @class  SensorDataDisplay
 * @author David Gossow (R12)
 * @brief  3D world display
 * @see    SensorDataGLWidget
 */
class SensorDataDisplay : public QWidget
{

    Q_OBJECT

  public:

    /** @brief sets up the layout. */
    SensorDataDisplay ( bool useDefaultPlugins, QWidget *parent = 0 );

    /** @brief The destructor. */
    ~SensorDataDisplay();
		
    SensorDataGLWidget* getSensorDataGLWidget() { return m_GlWidget; }

  public slots:

   // void processMessage ( Message* newMessage ); // TODO

   // void switchFS();

		/// @brief add a painter plugin
    void addPainter ( PainterPlugin* painter, bool visible );


  signals:

   // void fullScreenClicked();

  protected:

    /** @return Minimum size of the widget */
    virtual QSize minimumSizeHint() const;

    /** @return Preferred size of this widget. */
    virtual QSize sizeHint() const;

  private:

    QComboBox* m_NodeSelector;
    bool m_NodeSelectorInitialized;

    QDialog* m_OptionsDialog;
    QLayout* m_OptionsLayout;

    SensorDataGLWidget* m_GlWidget;
};


#endif

