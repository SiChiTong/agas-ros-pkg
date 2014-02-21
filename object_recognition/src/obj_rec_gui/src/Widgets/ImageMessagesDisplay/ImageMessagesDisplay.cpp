/*******************************************************************************
 *  ImageMessagesDisplay.cpp
 *
 *  (C) 2006 AG Aktives Sehen <agas@uni-koblenz.de>
 *           Universitaet Koblenz-Landau
 *
 *  Additional information:
 *  $Id: ImageMessagesDisplay.cpp 23656 2008-03-30 18:21:56Z dgossow $
 *******************************************************************************/

#include <QGridLayout>
#include <QLabel>
#include <QMatrix>
#include <QPixmap>
#include <QSize>
#include <QComboBox>
#include <QPushButton>
#include <QFileDialog>
#include <QSpinBox>
#include <QCheckBox>
#include <QTimer>
#include <QWidget>

#include "ImageMessagesDisplay.h"
#include "ImageSourceSelector.h"

#include "../../Widgets/GLImageWidget/GLImageWidget.h"

// TODO add messages
//#include "Messages/ImageM.h"
//#include "Messages/ImageStreamM.h"
//#include "Messages/StereoImageM.h"
//#include "Messages/GetImageM.h"

#define THIS ImageMessagesDisplay

using namespace std;
using namespace puma2;


THIS::THIS ( QWidget* parent, ImageSources::SourceId sourceId, bool showGrabButton, bool showSelector )
        : QGLWidget ( parent )
{
	m_GlImageWidget = new GLImageWidget ( this );
	m_SourceId = sourceId;

	QGridLayout* grid = new QGridLayout;

    if(showSelector)
    {
      ImageSourceSelector* sourceSelector = new ImageSourceSelector ( sourceId, this );
      connect ( sourceSelector, SIGNAL ( sourceSelected ( ImageSources::SourceId ) ), this,
                SLOT ( setSourceId ( ImageSources::SourceId ) ) );

      grid->addWidget ( sourceSelector, 0, 0, 1, 5 );

      grid->addWidget ( m_GlImageWidget, 1, 0, 1, 5 );
    }
    else
    {
      grid->addWidget(m_GlImageWidget, 0,0,0,0);
    }

	m_YuvCheckBox = 0;

	if ( showGrabButton )
	{
		QPushButton* button = new QPushButton ( "Get Image" );
		grid->addWidget ( button, 2, 0 );
		connect ( button, SIGNAL ( pressed() ), this, SLOT ( grabImage() ) );

		button = new QPushButton ( "Save Image" );
		grid->addWidget ( button, 2, 1 );
		connect ( button, SIGNAL ( pressed() ), this, SLOT ( saveImage() ) );

		m_PollTimer = new QTimer ( this );
		connect ( m_PollTimer, SIGNAL ( timeout() ), this, SLOT ( grabImage() ) );

		QCheckBox* pollCheckBox = new QCheckBox ( "Poll [ms]" );
		pollCheckBox->setCheckState ( Qt::Unchecked );
		connect ( pollCheckBox, SIGNAL ( stateChanged ( int ) ), this,
		          SLOT ( togglePoll ( int ) ) );

		m_PollSpinBox = new QSpinBox();
		m_PollSpinBox->setRange ( 50, 20000 );
		m_PollSpinBox->setSingleStep ( 250 );
		m_PollSpinBox->setValue ( 1000 );
		m_PollSpinBox->setMaximumWidth ( 80 );
		m_PollSpinBox->setEnabled ( false );
		connect ( m_PollSpinBox, SIGNAL ( valueChanged ( int ) ), this,
		          SLOT ( changePollInterval ( int ) ) );

		m_YuvCheckBox  = new QCheckBox ( "YUV", this );
		m_YuvCheckBox->setChecked ( false );

		grid->addWidget ( pollCheckBox, 2, 2 );
		grid->addWidget ( m_PollSpinBox, 2, 3 );
		grid->addWidget ( m_YuvCheckBox, 2, 4 );
	}

	grid->setRowStretch ( 1, 1 );
	grid->setColumnStretch ( 0, 1 );
	grid->setColumnStretch ( 1, 1 );
	setLayout ( grid );

//	m_Timer = Timer ( ProfilerEntry::CODE_SEGMENT, "GUI Thread", "Image Message Display" ); // TODO
}


void THIS::togglePoll ( int checkState )
{
	if ( checkState == Qt::Checked )
	{
		m_PollTimer->start ( m_PollSpinBox->value() );
		m_PollSpinBox->setEnabled ( true );
	}
	else
	{
		m_PollTimer->stop();
		m_PollSpinBox->setEnabled ( false );
	}
}


void THIS::changePollInterval ( int interval )
{
	if ( m_PollTimer->isActive() )
	{
		m_PollTimer->setInterval ( interval );
	}
}

void THIS::grabImage()
{
	if ( m_YuvCheckBox && ( m_YuvCheckBox->checkState() == Qt::Checked ) )
	{
        // TODO
        // sendMessage ( new GetImageM ( m_SourceId, ImageGrabber::Y8UV8, ImageGrabber::FULL ) );
	}
	else
	{
        // TODO
        // sendMessage ( new GetImageM ( m_SourceId, ImageGrabber::RGB8, ImageGrabber::FULL ) );
	}
}

void THIS::saveImage()
{
	QString filename = QFileDialog::getSaveFileName ( this, tr ( "Save Image" ), "images",
	                   tr ( "*.png" ) );
	if ( filename.size() != 0 )
	{
		string filenameString = filename.toStdString();
		if ( filenameString.find ( ".png" ) == string::npos )
		{
			filenameString += ".png";
		}
		m_GlImageWidget->saveImage ( filenameString );
	}
}

void THIS::setSourceId ( ImageSources::SourceId sourceId )
{
	m_SourceId = sourceId;
	ostringstream stream;
	stream << "Setting source id to " << int ( m_SourceId )
	<< " (" << ImageSources::getSourceDesc ( m_SourceId )
	<< ") and grabbing image.";
    // TRACE_SYSTEMINFO ( stream.str() ) // TODO use ros
	grabImage();
}

void THIS::updateImage(const unsigned char* image, unsigned width, unsigned height)
{
    // IMAGE_STREAM_M:
    //  if ( m_SourceId == castMessage->getSourceId() )

    m_GlImageWidget->clearForms();
//            //store vertices
//            std::list< VectorObject2D >::iterator vectorObjectIt;
//            std::list< VectorObject2D > vectorObjects = castMessage->getVectorObjects();
//            for ( vectorObjectIt = vectorObjects.begin(); vectorObjectIt != vectorObjects.end(); vectorObjectIt++ )
//            {
//                m_GlImageWidget->addVectorObject ( *vectorObjectIt );
//            }
    m_GlImageWidget->setColorImage ( image, width, height );
}


// TODO add message handling

/*

void THIS::processMessage ( Message* message )
{
    ostringstream stream;
    ostringstream filename;

        case MessageTypes::IMAGE_M:
        {
            ImageM* castMessage = Message::castTo<ImageM> ( message );
            if ( m_SourceId == castMessage->getSourceId() )
            {
                stream << "m_SourceId " << m_SourceId << " castMessage->getSourceId() "
                << castMessage->getSourceId();
                TRACE_SYSTEMINFO ( stream.str() )
                m_GlImageWidget->clearForms();
                //store vertices
                std::list< VectorObject2D >::iterator vectorObjectIt;
                std::list< VectorObject2D > vectorObjects = castMessage->getVectorObjects();
                for ( vectorObjectIt = vectorObjects.begin(); vectorObjectIt != vectorObjects.end(); vectorObjectIt++ )
                {
                    m_GlImageWidget->addVectorObject ( *vectorObjectIt );
                }
                //store&display image
                //ImageSources::SourceId sid = castMessage->getSourceId();
                switch ( castMessage->getColorFormat() )
                {
                    case ImageGrabber::RGB8:
                                                TRACE_SYSTEMINFO ( "Received RGB8 image" )
                        m_GlImageWidget->setColorImage ( castMessage->getRgbImage() );
                        break;
                    case ImageGrabber::Y8UV8:
                                                TRACE_SYSTEMINFO ( "Received Y8UV8 image" )
                        m_GlImageWidget->setColorImage ( castMessage->getGrayLevelImage(),
                                                         castMessage->getUvImage() );
                        break;
                    default:
                                                TRACE_ERROR ("Received unknown image type")
                        break;
                }
            }
        }

}

  */

#undef THIS



