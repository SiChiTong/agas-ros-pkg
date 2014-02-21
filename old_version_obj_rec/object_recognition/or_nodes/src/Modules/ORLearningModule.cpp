/*******************************************************************************
*  ORLearningModule.cpp
*
*  (C) 2005, 2013 AG Aktives Sehen <agas@uni-koblenz.de>
*                 Universitaet Koblenz-Landau
*
******************************************************************************/

#include "ORLearningModule.h"

#include <sstream>
#include <fstream>

#include <ros/package.h>
#include <sensor_msgs/image_encodings.h>
#include <or_msgs/OrLearningStatus.h>
#include <or_msgs/VectorObject2D.h>
#include <or_msgs/Point2D.h>

#include <opencv2/highgui/highgui.hpp>

#include "Architecture/Config/Config.h"

#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/text_oarchive.hpp>

#include "Workers/ImageHelpers/ImageMaskCV.h"
#include "Workers/VectorGraphics/VectorObject2D.h"

#include "ObjectRecognition/ObjectProperties.h"
#include "ObjectRecognition/ImagePropertiesCV.h"

#include "ConnectedComponentAnalyzer/ConnectedComponentAnalyzer.h"


#define THIS ORLearningModule

THIS::THIS(ros::NodeHandle *nh, std::string inputTopic)
{
    m_PathForSaving = Config::getString ( "ObjectRecognition.sDataPath" );

    // histogram parameters
    m_HistogramBinSize = Config::getInt ( "ObjectRecognition.Histogram.iBinSize" );

    m_HistogramMinY = Config::getInt ( "ObjectRecognition.Histogram.iMinY" );
    m_HistogramMaxY = Config::getInt ( "ObjectRecognition.Histogram.iMaxY" );

    m_HistogramClearRange = Config::getFloat ( "ObjectRecognition.Histogram.fHistogramClearRange" );

    m_OpenRadius = 15.0;
    m_BorderSize = 5.0;

    m_BackgroundImageGray = 0;
    m_BackgroundImageColor = 0;
    m_ForegroundImageGray = 0;
    m_ForegroundImageColor = 0;
    m_DifferenceThreshold = 0;
    m_IsolateLargestSegment = false;

    ADD_MACHINE_STATE ( m_ModuleMachine, IDLE );
    ADD_MACHINE_STATE ( m_ModuleMachine, WAITING_FOR_FOREGROUND );
    ADD_MACHINE_STATE ( m_ModuleMachine, WAITING_FOR_BACKGROUND );
    m_ModuleMachine.setName ( "ObjectLearning State" );

    m_ObjectProperties = new ObjectProperties( );

    m_ImageRequested = false;

    // subscribe to topics
    m_ORLearnCommandSubscriber = nh->subscribe<or_msgs::OrLearnCommand>("/or/learn_commands", 10, &ORLearningModule::callbackOrLearnCommand, this);

    // subscribe to image input topic
    m_ImageSubscriber = nh->subscribe<sensor_msgs::Image>(inputTopic, 10, &ORLearningModule::callbackImage, this);

    // advertise topics
    m_OLPrimaryImagePublisher = nh->advertise<or_msgs::OrImage>("or/obj_learn_primary", 10);
    m_OLDebugImagePublisher = nh->advertise<or_msgs::OrImage>("or/debug_image", 10);
    m_ORLearningStatusPublisher = nh->advertise<or_msgs::OrLearningStatus>("or/learning_status", 10);
}


THIS::~THIS()
{
    delete m_BackgroundImageGray;
    delete m_BackgroundImageColor;
    delete m_ForegroundImageGray;
    delete m_ForegroundImageColor;
    delete m_ObjectProperties;
}

void THIS::callbackOrLearnCommand( const or_msgs::OrLearnCommand::ConstPtr& message )
{
    try
    {
        std::ostringstream s;
        switch ( message->command )
        {
        case ORLearningModule::SetDifferenceThreshold:
            m_DifferenceThreshold = message->int_value;
            ROS_INFO_STREAM ( "Setting difference threshold to " << m_DifferenceThreshold );
            if ( m_BackgroundImageGray && m_ForegroundImageGray )
            {
                previewIsolatedImage();
            }
            break;

        case ORLearningModule::SetOpenRadius:
            m_OpenRadius = message->int_value;
            ROS_INFO_STREAM ( "Setting opening radius to " << m_OpenRadius );
            if ( m_BackgroundImageGray && m_ForegroundImageGray )
            {
                previewIsolatedImage();
            }
            break;

        case ORLearningModule::SetBorderSize:
            m_BorderSize = message->int_value;
            ROS_INFO_STREAM ( "Setting border size to " << m_BorderSize );
            if ( m_BackgroundImageGray && m_ForegroundImageGray )
            {
                previewIsolatedImage();
            }
            break;

        case ORLearningModule::SetIsolateLargestSegment:
            if ( message->string_value == "true" )
            {
                ROS_INFO_STREAM ( "Using single largest segment" );
                m_IsolateLargestSegment = true;
            }
            else
            {
                ROS_INFO_STREAM ( "Using all segments" );
                m_IsolateLargestSegment = false;
            }
            if ( m_BackgroundImageGray && m_ForegroundImageGray )
            {
                previewIsolatedImage();
            }
            break;

        case ORLearningModule::GrabBackgroundImage:
        {
            m_ModuleMachine.setState ( WAITING_FOR_BACKGROUND );
            ROS_INFO_STREAM ( "Grabbing background image" );
            m_ImageRequested = true;
            break;
        }
        case ORLearningModule::GrabForegroundImage:
        {
            m_ModuleMachine.setState ( WAITING_FOR_FOREGROUND );
            ROS_INFO_STREAM ( "Grabbing foreground image" );
            m_ImageRequested = true;
            break;
        }
        case ORLearningModule::LoadBackgroundImage:
        {
            m_ModuleMachine.setState ( WAITING_FOR_BACKGROUND );
            std::string fileName = message->string_value;
            ROS_INFO_STREAM("Loading background image: " << fileName);
            //read and publish image
            loadImage(fileName);
            break;
        }

        case ORLearningModule::LoadForegroundImage:
        {
            m_ModuleMachine.setState ( WAITING_FOR_FOREGROUND );
            std::string fileName = message->string_value;
            ROS_INFO_STREAM("Loading foreground image: " << fileName);
            //read and publish image
            loadImage(fileName);
            break;
        }
        case ORLearningModule::DisplayImage:
            ROS_INFO_STREAM ( "Displaying Image #" << message->int_value );
            displayImage ( message->int_value );
            break;

        case ORLearningModule::SaveImage:
        {
            ROS_INFO_STREAM ( "Saving Image '" << message->string_value << "'" );
            saveImage( message->string_value );
            // publish learning status message
            or_msgs::OrLearningStatus learn_state_msg;
            learn_state_msg.image_names = m_ObjectProperties->getImageNames();
            learn_state_msg.object_type = m_ObjectType;
            m_ORLearningStatusPublisher.publish(learn_state_msg);
            break;
        }
        case ORLearningModule::DeleteImage:
        {
            ROS_INFO_STREAM ( "Deleting image #" << message->int_value << "'" );
            deleteImage ( message->int_value );
            // publish learning status message
            or_msgs::OrLearningStatus learn_state_msg;
            learn_state_msg.image_names = m_ObjectProperties->getImageNames();
            learn_state_msg.object_type = m_ObjectType;
            m_ORLearningStatusPublisher.publish(learn_state_msg);
            break;
        }
        case ORLearningModule::SetObjectType:
            ROS_INFO_STREAM ( "Setting object type to '" << message->string_value << "'" );
            m_ObjectType = message->string_value;
            break;

        case ORLearningModule::SaveObject:
        {
            ROS_INFO_STREAM ( "Saving object as '" << message->string_value << "'" );
            saveObject ( message->string_value );
            // publish learning status message
            or_msgs::OrLearningStatus learn_state_msg;
            learn_state_msg.image_names = m_ObjectProperties->getImageNames();
            learn_state_msg.object_type = m_ObjectType;
            m_ORLearningStatusPublisher.publish(learn_state_msg);
            break;
        }
        case ORLearningModule::LoadObject:
        {
            ROS_INFO_STREAM ( "Loading object '" << message->string_value << "'" );
            loadObject ( message->string_value );
            // publish learning status message
            or_msgs::OrLearningStatus learn_state_msg;
            learn_state_msg.image_names = m_ObjectProperties->getImageNames();
            learn_state_msg.object_type = m_ObjectType;
            m_ORLearningStatusPublisher.publish(learn_state_msg);
            break;
        }
        } // closes switch
        if ( s.str() != "" )
        {
            ROS_INFO_STREAM ( s.str() );
        }
    }
    catch ( const char* exception_msg )
    {
        std::ostringstream stream;
        stream << "Caught exception: " << std::endl << exception_msg;
        ROS_ERROR_STREAM ( stream.str() );
        throw exception_msg;
    }
}

void THIS::callbackImage( const sensor_msgs::Image::ConstPtr& message )
{
    if(m_ImageRequested)
    {
        m_ImageRequested = false;
        processImageMessage(message);
    }
}

void THIS::processImageMessage( const sensor_msgs::Image::ConstPtr& message )
{
    try
    {
        cv_bridge::CvImagePtr color_img_ptr;
        cv_bridge::CvImagePtr gray_img_ptr(new cv_bridge::CvImage);

        try
        {
            color_img_ptr = cv_bridge::toCvCopy(message);
            cvtColor( color_img_ptr->image, gray_img_ptr->image, CV_BGR2GRAY );
        }
        catch (cv_bridge::Exception error)
        {
            ROS_ERROR("Error converting Image message to OpenCV image.");
        }

        switch ( m_ModuleMachine.state() )
        {
        case WAITING_FOR_BACKGROUND:
            setBackground ( gray_img_ptr, color_img_ptr );
            m_ModuleMachine.setState ( IDLE );
            break;
        case WAITING_FOR_FOREGROUND:
            setForeground ( gray_img_ptr, color_img_ptr );
            previewIsolatedImage();
            m_ModuleMachine.setState ( IDLE );
            break;
        default: break;
        }

        or_msgs::OrImage or_image_msg;
        or_image_msg.image_gray = *(gray_img_ptr->toImageMsg());
        or_image_msg.image_color = *(color_img_ptr->toImageMsg());
        or_image_msg.filename = "";
        m_OLDebugImagePublisher.publish(or_image_msg);
    }
    catch ( const char* exception_msg )
    {
        std::ostringstream stream;
        stream << "Caught exception: " << std::endl << exception_msg;
        ROS_ERROR_STREAM ( stream.str() );
        throw exception_msg;
    }
}


void THIS::loadImage(std::string fileName)
{
    cv_bridge::CvImage gray_image, color_image;
    gray_image.image = cv::imread(fileName.c_str(), CV_LOAD_IMAGE_GRAYSCALE);
    gray_image.encoding = sensor_msgs::image_encodings::TYPE_8UC1;
    color_image.image = cv::imread(fileName.c_str(), CV_LOAD_IMAGE_COLOR);
    color_image.encoding = sensor_msgs::image_encodings::TYPE_8UC3;

    or_msgs::OrImage or_image_msg;
    or_image_msg.image_gray = *(gray_image.toImageMsg());
    or_image_msg.image_color = *(color_image.toImageMsg());
    or_image_msg.filename = fileName;
    m_OLDebugImagePublisher.publish(or_image_msg);

    // TODO testcode remove this
    //    cv_bridge::CvImagePtr test = cv_bridge::toCvCopy(or_image_msg.image_gray);
    //    imwrite("/home/vseib/Desktop/test_gray.png", test->image);
    //    cv_bridge::CvImagePtr test2 = cv_bridge::toCvCopy(or_image_msg.image_color);
    //    imwrite("/home/vseib/Desktop/test_color.png", test2->image);
}

void THIS::setBackground ( cv_bridge::CvImagePtr gray_image, cv_bridge::CvImagePtr color_image )
{
    //safe gray image
    if ( m_BackgroundImageGray ) { delete m_BackgroundImageGray; }
    m_BackgroundImageGray = new cv::Mat(gray_image->image);

    //safe color image
    if ( m_BackgroundImageColor ) { delete m_BackgroundImageColor; }
    m_BackgroundImageColor = new cv::Mat(color_image->image);

    // send color image to OLPrimary-GUI
    or_msgs::OrImage or_color_image_msg;
    or_color_image_msg.image_color = *(color_image->toImageMsg());
    m_OLPrimaryImagePublisher.publish(or_color_image_msg);
}

void THIS::setForeground ( cv_bridge::CvImagePtr gray_image, cv_bridge::CvImagePtr color_image )
{
    //safe gray image
    if ( m_ForegroundImageGray ) { delete m_ForegroundImageGray; }
    m_ForegroundImageGray = new cv::Mat(gray_image->image);

    //safe color image
    if ( m_ForegroundImageColor ) { delete m_ForegroundImageColor; }
    m_ForegroundImageColor = new cv::Mat(color_image->image);
}


void THIS::saveImage ( std::string name )
{
    ImagePropertiesCV* imageProperties = makeImageProperties( name, true );
    if ( imageProperties )
    {
        imageProperties->calculateProperties();
        m_ObjectProperties->addImageProperties( imageProperties );
    }
}


void THIS::displayImage ( int index )
{
    std::vector<ImagePropertiesCV*> pVec = m_ObjectProperties->getImageProperties( );

    if ( index > (int)pVec.size() )
    {
        ROS_ERROR_STREAM( "Image #" << index << " does not exist!" );
        return;
    }

    const ImagePropertiesCV* imageProperties = pVec[ index ];

    // send color image to OLPrimary-GUI
    cv_bridge::CvImagePtr cv_ptr_color;
    cv_ptr_color->image = *(imageProperties->getMaskedImageUV()); // TODO instead of UV an rgb image was saved: does this change something?
    cv_ptr_color->encoding = sensor_msgs::image_encodings::TYPE_8UC3;
    or_msgs::OrImage or_image_msg;
    or_image_msg.image_color = *(cv_ptr_color->toImageMsg());

    const std::vector< KeyPoint >* keyPoints = imageProperties->getKeyPoints();

    for ( unsigned kp = 0; kp < keyPoints->size(); kp++ )
    {
        // TODO make creation of ROS messages nicer

        VectorObject2D arrow ( ( *keyPoints ) [kp].getCircle(), 0, 0, 0, 5.0 );
        // convert VectorObject2D to ROS message
        or_msgs::VectorObject2D arrow_msg;
        arrow_msg.r = arrow.r();
        arrow_msg.g = arrow.g();
        arrow_msg.b = arrow.b();
        arrow_msg.line_width = arrow.lineWidth();
        std::vector<Point2D> tmp_vertices = arrow.vertices();
        for(unsigned i = 0; i < tmp_vertices.size(); i++)
        {
            or_msgs::Point2D point;
            point.x = tmp_vertices.at(i).x();
            point.y = tmp_vertices.at(i).y();
            arrow_msg.vertices.push_back(point);
        }
        or_image_msg.vector_objects.push_back( arrow_msg );


        VectorObject2D arrow2 ( ( *keyPoints ) [kp].getCircle(), 1, 1, 1, 3.0 );
        // convert VectorObject2D to ROS message
        or_msgs::VectorObject2D arrow_msg2;
        arrow_msg2.r = arrow2.r();
        arrow_msg2.g = arrow2.g();
        arrow_msg2.b = arrow2.b();
        arrow_msg2.line_width = arrow2.lineWidth();
        tmp_vertices = arrow2.vertices();
        for(unsigned i = 0; i < tmp_vertices.size(); i++)
        {
            or_msgs::Point2D point;
            point.x = tmp_vertices.at(i).x();
            point.y = tmp_vertices.at(i).y();
            arrow_msg.vertices.push_back(point);
        }
        or_image_msg.vector_objects.push_back( arrow_msg2 );
    }

    Point2D center = imageProperties->getCenter();

    int size = imageProperties->getImageMask()->getWidth() / 20;

    ROS_INFO_STREAM( center.x() << " " << center.y() );

    std::vector<Point2D> vLine;
    vLine.push_back( center-Point2D( 0, size ) );
    vLine.push_back( center+Point2D( 0, size ) );
    or_msgs::VectorObject2D v_line_msg;
    for(unsigned i = 0; i < vLine.size(); i++)
    {
        or_msgs::Point2D point;
        point.x = vLine.at(i).x();
        point.y = vLine.at(i).y();
        v_line_msg.vertices.push_back(point);
    }
    v_line_msg.r = 1;
    v_line_msg.g = 1;
    v_line_msg.b = 1;
    v_line_msg.line_width = 3.0;
    or_image_msg.vector_objects.push_back(v_line_msg);
    v_line_msg.r = 0;
    v_line_msg.g = 0;
    v_line_msg.b = 1;
    v_line_msg.line_width = 1.0;
    or_image_msg.vector_objects.push_back(v_line_msg);


    std::vector<Point2D> hLine;
    hLine.push_back( center-Point2D( size, 0 ) );
    hLine.push_back( center+Point2D( size, 0 ) );
    or_msgs::VectorObject2D h_line_msg;
    for(unsigned i = 0; i < hLine.size(); i++)
    {
        or_msgs::Point2D point;
        point.x = hLine.at(i).x();
        point.y = hLine.at(i).y();
        h_line_msg.vertices.push_back(point);
    }
    h_line_msg.r = 1;
    h_line_msg.g = 1;
    h_line_msg.b = 1;
    h_line_msg.line_width = 3.0;
    or_image_msg.vector_objects.push_back(h_line_msg);
    h_line_msg.r = 0;
    h_line_msg.g = 0;
    h_line_msg.b = 1;
    h_line_msg.line_width = 1.0;
    or_image_msg.vector_objects.push_back(h_line_msg);


    VectorObject2D border ( *(imageProperties->getOutline()), 1, 0, 0, 1.0 );
    // convert VectorObject2D to ROS message
    or_msgs::VectorObject2D vector_msg;
    vector_msg.r = border.r();
    vector_msg.g = border.g();
    vector_msg.b = border.b();
    vector_msg.line_width = border.lineWidth();

    std::vector<Point2D> tmp_vertices = border.vertices();
    for(unsigned i = 0; i < tmp_vertices.size(); i++)
    {
        or_msgs::Point2D point;
        point.x = tmp_vertices.at(i).x();
        point.y = tmp_vertices.at(i).y();
        vector_msg.vertices.push_back(point);
    }
    or_image_msg.vector_objects.push_back(vector_msg);

    m_OLPrimaryImagePublisher.publish(or_image_msg);
}

void THIS::deleteImage ( int index )
{
    m_ObjectProperties->deleteImageProperties( index );
}

ImagePropertiesCV* THIS::makeImageProperties( std::string name, bool crop )
{
    if ( !m_BackgroundImageGray || !m_BackgroundImageColor )
    {
        ROS_ERROR_STREAM ( "Background image missing!" );
        return 0;
    }
    if ( !m_ForegroundImageGray || !m_ForegroundImageColor )
    {
        ROS_ERROR_STREAM ( "Foreground image missing!" );
        return 0;
    }

    ImageMaskCV mask( *m_ForegroundImageGray, *m_ForegroundImageColor, *m_BackgroundImageGray, *m_BackgroundImageColor, m_DifferenceThreshold );

    mask.erode ( 1.0 );
    mask.dilate ( m_OpenRadius );
    mask.erode ( m_OpenRadius - 1.0 );

    //delete all mask segments but largest
    if ( m_IsolateLargestSegment )
    {
        ConnectedComponentAnalyzer::isolateLargestSegment ( mask.getData(), mask.getWidth(), mask.getHeight() );
    }

    mask.dilate ( m_BorderSize );

    if ( crop )
    {
        //crop image & mask

        Box2D<int> area = mask.getBoundingBox();

        float borderSize = Config::getFloat( "ObjectRecognition.fObjectImageBorder" );
        int border = ( area.width()+area.height() ) / 2 * borderSize;
        area.expand( border+2 );

        area.clip( Box2D<int>( 0, 0, m_ForegroundImageGray->cols, m_ForegroundImageColor->rows ) );

        int newWidth = area.width();
        int newHeight= area.height();
        int minX = area.minX();
        int minY = area.minY();

        ImageMaskCV *croppedMask = mask.subMask( area );

        cv::Mat* croppedGrayImage = new cv::Mat(newHeight, newWidth, CV_8UC1);
        for ( int y=0; y<newHeight; y++ )
        {
            for ( int x=0; x<newWidth; x++ )
            {
                croppedGrayImage->at<unsigned char>(y,x) = m_ForegroundImageGray->at<unsigned char>(y+minY, x+minX);
            }
        }

        cv::Mat* croppedColorImage = new cv::Mat(newHeight, newWidth, CV_8UC3);
        for ( int y=0; y<newHeight; y++ )
        {
            for ( int x=0; x<newWidth; x++ )
            {
                croppedColorImage->at<cv::Vec3b>(y,x) = m_ForegroundImageGray->at<cv::Vec3b>(y+minY, x+minX);
            }
        }
        ImagePropertiesCV* imageProperties = new ImagePropertiesCV( name, croppedGrayImage, croppedColorImage, croppedMask );
        return imageProperties;
    }
    else
    {
        ImagePropertiesCV* imageProperties = new ImagePropertiesCV( name, new cv::Mat( *m_ForegroundImageGray ),
                                                                    new cv::Mat( *m_ForegroundImageColor ), new ImageMaskCV( mask ) );
        return imageProperties;
    }

}

void THIS::previewIsolatedImage()
{
    ImagePropertiesCV* imageProperties = makeImageProperties( "", false );

    if ( imageProperties )
    {
        // send color image to OLPrimary-GUI
        cv_bridge::CvImagePtr cv_ptr_color(new cv_bridge::CvImage);
        cv_ptr_color->image = *(imageProperties->getMaskedImageUV());
        cv_ptr_color->encoding = sensor_msgs::image_encodings::TYPE_8UC3;
        or_msgs::OrImage or_image_msg;
        or_image_msg.image_color = *(cv_ptr_color->toImageMsg());

        imageProperties->traceOutline();
        VectorObject2D border ( *(imageProperties->getOutline()), 1, 0, 0, 1.0 );

        // convert VectorObject2D to ROS message
        or_msgs::VectorObject2D vector_msg;
        vector_msg.r = border.r();
        vector_msg.g = border.g();
        vector_msg.b = border.b();
        vector_msg.line_width = border.lineWidth();

        std::vector<Point2D> tmp_vertices = border.vertices();
        for(unsigned i = 0; i < tmp_vertices.size(); i++)
        {
            or_msgs::Point2D point;
            point.x = tmp_vertices.at(i).x();
            point.y = tmp_vertices.at(i).y();
            vector_msg.vertices.push_back(point);
        }

        or_image_msg.vector_objects.push_back(vector_msg);
        m_OLPrimaryImagePublisher.publish(or_image_msg);
    }

    delete imageProperties;
}

void THIS::saveObject ( std::string objectName )
{
    m_ObjectProperties->setName( objectName );
    m_ObjectProperties->setType( m_ObjectType );

    std::string path = ros::package::getPath("or_nodes");
    std::string filename = path + m_PathForSaving + objectName + ".objprop";

    // write objectProperties file
    std::ofstream out ( filename.c_str() );
    boost::archive::text_oarchive oa(out);
    oa << m_ObjectProperties;

    delete m_ObjectProperties;
    m_ObjectProperties = new ObjectProperties();

    ROS_INFO_STREAM ( "Object saved to " << filename );
}


void THIS::loadObject ( std::string filename )
{
    delete m_ObjectProperties;

    //std::string filename = m_PathForSaving + objectName + ".objprop";

    ROS_INFO_STREAM( "Loading " + filename );
    std::ifstream ifs( filename.c_str() );
    boost::archive::text_iarchive ia(ifs);
    m_ObjectProperties = new ObjectProperties();
    ia >> m_ObjectProperties;
    ifs.close();

    m_ObjectType = m_ObjectProperties->getType();
    ROS_INFO_STREAM( m_ObjectType );
}

#undef THIS
