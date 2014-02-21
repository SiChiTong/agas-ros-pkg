/*******************************************************************************
 *  ImageMessagesDisplay.h
 *
 *  (C) 2006 AG Aktives Sehen <agas@uni-koblenz.de>
 *           Universitaet Koblenz-Landau
 *
 *  Additional information:
 *  $Id: ImageMessagesDisplay.h 23656 2008-03-30 18:21:56Z dgossow $
 *******************************************************************************/

#ifndef ImageMessagesDisplay_H
#define ImageMessagesDisplay_H

#include <QGLWidget>
#include <map>

#include "../../Workers/ImageSources/ImageSources.h"

class QLabel;
class QGridLayout;
class ImageM;
class GLImageWidget;
class QSpinBox;
class QTimer;
class QCheckBox;


/**
 * @brief Widget able to display the content of image messages
 * @author David Gossow (RX)
 */

class ImageMessagesDisplay : public QGLWidget {

    Q_OBJECT

public:

  ImageMessagesDisplay( QWidget* parent = 0,
                        ImageSources::SourceId sourceId = ImageSources::None,
                        bool showGrabButton = false, bool showSelector = true );
  ~ImageMessagesDisplay() {}

  void updateImage(const unsigned char* image, unsigned width, unsigned height);

public slots:

  // void processMessage( Message* message ); // TODO

  void setSourceId( ImageSources::SourceId sourceId );

  /** @brief Request an image of the selected type */
  void grabImage();

  void saveImage();

  void togglePoll( int checkState );

  void changePollInterval( int interval );

public:

  GLImageWidget* m_GlImageWidget;

private:

  ImageSources::SourceId m_SourceId; // TODO evtl. loswerden oder mit ros topics umsetzen

  QSpinBox* m_PollSpinBox;
  QTimer* m_PollTimer;

  QCheckBox* m_YuvCheckBox;

};

#endif
