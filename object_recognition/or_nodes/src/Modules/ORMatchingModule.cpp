/*******************************************************************************
 *  ORMatchingModule.cpp
 *
 *  (C) 2006 AG Aktives Sehen <agas@uni-koblenz.de>
 *           Universitaet Koblenz-Landau
 *
 *******************************************************************************/

#include "ORMatchingModule.h"
#include "ORControlModule.h"

#include <sstream>
#include <fstream>
#include <limits>
#include <algorithm>

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>

#include "Architecture/Config/Config.h"
#include "Architecture/Singleton/Clock.h"

#include "Workers/Math/Math.h"

#include "ObjectRecognition/CvHomography.h"
#include "ObjectRecognition/HoughClusterer.h"
#include "ObjectRecognition/MatchHelper.h"
#include "ObjectRecognition/NNRMatcher.h"
#include "ObjectRecognition/SimpleHoughClusterer.h"
#include "ObjectRecognition/FLANNMatcher.h"

#include "KeyPointExtraction/DefaultExtractor.h"

#include <ros/package.h>
#include <or_msgs/OrMatchResult.h>
#include <or_msgs/OrObjectNames.h>

#define THIS ORMatchingModule

using namespace std;

THIS::THIS(ros::NodeHandle *nh, std::string inputTopic)
{
    m_Stage1Matcher = ORMatchingModule::Stage1MatcherT( Config::getInstance()->getInt( "ObjectRecognition.iStage1Matcher" ) );
    m_Stage2Matcher = ORMatchingModule::Stage2MatcherT( Config::getInstance()->getInt( "ObjectRecognition.iStage2Matcher" ) );
    m_ImagesRequested = 0;
    m_FlannMatcher = 0;

    m_Extractor = DefaultExtractor::createInstance();
    ROS_INFO_STREAM( "Selected feature extractor: " << m_Extractor->getName() << std::endl << std::endl << m_Extractor->getDescription()  );

    // subscribe to messages
    m_OrCommandSubscriber = nh->subscribe<or_msgs::OrCommand>("/or/commands", 100, &ORMatchingModule::callbackOrCommand, this);
    m_ExtractKeyPointsSubscriber = nh->subscribe<or_msgs::ExtractKeyPoints>("/or/extract", 100, &ORMatchingModule::callbackExtractKeyPoints, this);

    // subscribe to image input topic
    m_ImageSubscriber = nh->subscribe<sensor_msgs::Image>(inputTopic, 10, &ORMatchingModule::callbackImage, this);

    // advertise messages
    m_ORMatchResultPublisher = nh->advertise<or_msgs::OrMatchResult>("or/match_result", 100);
    m_ORObjectNamesPublisher = nh->advertise<or_msgs::OrObjectNames>("or/obj_names", 100);
}

THIS::~THIS()
{
    ROS_INFO_STREAM( "Mean processing time: " << Math::mean( m_ProcessingTimes ) << "ms" );
    delete m_Extractor;
    if(m_FlannMatcher)
        delete m_FlannMatcher;
}

void THIS::callbackOrCommand( const or_msgs::OrCommand::ConstPtr& msg )
{
    switch( msg->command )
    {
    case ORControlModule::UnloadObject:
        removeObjectProperties( msg->string_value );
        break;

    default:
        break;
    }
}


void THIS::callbackExtractKeyPoints( const or_msgs::ExtractKeyPoints::ConstPtr& msg )
{
    //m_SourceId = (ImageSources::SourceId) msg->img_source; // TODO check for correct image source

    m_BoundingBoxes.clear();
    for( unsigned i = 0; i < msg->bounding_boxes.size(); i++)
    {
        Box2D<double> box(msg->bounding_boxes.at(i).minX, msg->bounding_boxes.at(i).minY,
                          msg->bounding_boxes.at(i).maxX, msg->bounding_boxes.at(i).maxY);
        m_BoundingBoxes.push_back(box);
    }
    m_ImagesRequested++;
}

void THIS::callbackImage( const sensor_msgs::Image::ConstPtr& message )
{
    if ( m_ImagesRequested > 0 )
    {
        m_ImagesRequested--;
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

            ROS_INFO_STREAM( "analyzing gray image" );
            m_ImageWidth = gray_img_ptr->image.cols;
            m_ImageHeight = gray_img_ptr->image.rows;

            //convert bboxes from image to pixel coords
            std::vector< Box2D<int> > boundingBoxes( m_BoundingBoxes.size() );
            for ( unsigned i=0; i<m_BoundingBoxes.size(); i++ )
            {
                boundingBoxes[i].setMinX( ( m_BoundingBoxes[i].minX() *  0.5 + 0.5 ) * m_ImageWidth );
                boundingBoxes[i].setMaxX( ( m_BoundingBoxes[i].maxX() *  0.5 + 0.5 ) * m_ImageWidth );
                boundingBoxes[i].setMinY( ( m_BoundingBoxes[i].maxY() * -0.5 + 0.5 ) * m_ImageHeight );
                boundingBoxes[i].setMaxY( ( m_BoundingBoxes[i].minY() * -0.5 + 0.5 ) * m_ImageHeight );
                ROS_INFO_STREAM( "Using bounding box: min = " << boundingBoxes[i].minX() << ", " << boundingBoxes[i].minY() <<  " max = " << boundingBoxes[i].maxX() << ", " << boundingBoxes[i].maxY() );
            }

//            // TODO only for testing: create a "test-mode" demo in a similar way
//            cv::namedWindow( "Display window", CV_WINDOW_AUTOSIZE );// Create a window for display.
//            cv::imshow( "Display window", gray_img_ptr->image );   // Show our image inside it.
//            cv::waitKey(0);

           processImages ( &gray_img_ptr->image, message->header.seq, boundingBoxes );
        }
        catch ( const char* exception_msg )
        {
            std::ostringstream stream;
            stream << "Caught exception: " << std::endl << exception_msg;
            ROS_ERROR_STREAM ( stream.str() );
            throw exception_msg;
        }
}


void THIS::processImages( cv::Mat *image, int seqNum, std::vector< Box2D<int> > boundingBoxes )
{
    std::vector< KeyPoint > *rawKeyPoints = new std::vector< KeyPoint >();

    int startTime = Clock::getInstance()->getTimestamp();

    m_Extractor->setImage( *image );
    m_Extractor->getKeyPoints( *rawKeyPoints );

    ROS_INFO_STREAM ( "Found " << rawKeyPoints->size() << " keypoints in " << ( Clock::getInstance()->getTimestamp() - startTime ) << "ms" );

    processKeyPoints( image, rawKeyPoints, boundingBoxes, seqNum );

    double deltaT = Clock::getInstance()->getTimestamp() - startTime;
    m_ProcessingTimes.push_back( deltaT );
    // ROS_INFO_STREAM( "Processing time: " + Tracer::toString(deltaT) + "ms" ); // TODO
}


void THIS::processKeyPoints(  cv::Mat *image, std::vector< KeyPoint > *rawKeyPoints, std::vector< Box2D<int> > boundingBoxes, unsigned seqNum )
{
    int startTime = Clock::getInstance()->getTimestamp();

    if( boundingBoxes.size() == 0 )
    {
        boundingBoxes.push_back( Box2D<int>(0,0,image->cols,image->rows) );
    }

    std::vector<MatchResult*> matchResults;

    if (rawKeyPoints->size() == 0) {
        ROS_ERROR_STREAM("No raw keypoints");
    } else {
        ROS_INFO_STREAM("Got " << rawKeyPoints->size() << " raw keypoints.");
    }
    //loop over bounding boxes
    for ( unsigned b=0; b<boundingBoxes.size(); b++ )
    {
        std::vector< KeyPoint > keyPoints;
        keyPoints.reserve( rawKeyPoints->size() );

        //maps indices of bounding box keypoints to (raw) scene keypoints
        std::vector< unsigned > keyPointIndexMap;
        keyPointIndexMap.reserve( rawKeyPoints->size() );

        //get keypoints for this bb
        for ( unsigned k=0; k<rawKeyPoints->size(); k++ )
        {
            if ( boundingBoxes[b].contains( (*rawKeyPoints)[k].x, (*rawKeyPoints)[k].y ) )
            {
                //keyPoint is in one bounding box -> save and go to next kp
                keyPoints.push_back( (*rawKeyPoints)[k] );
                keyPointIndexMap.push_back( k );
            }
        }

        if (keyPoints.size() == 0)
            ROS_ERROR_STREAM("No keypoints for bounding box " << b);

        //recreate lookup
        if(m_Stage1Matcher==Flann)
        {
            delete m_FlannMatcher;
            m_FlannMatcher = new FLANNMatcher();
            m_FlannMatcher->createIndex(&keyPoints);
        }

        //match all objects
        for ( unsigned i=0; i<m_ObjectList.size(); i++ )
        {
            MatchResult* matchResult=new MatchResult;
            if ( matchObject( &keyPoints, m_ObjectList[i], *matchResult ) )
            {
                ROS_INFO_STREAM( "Detected object '" << m_ObjectList[i].getName() << "' in bounding box # " << b );
                matchResult->boundingBoxIndex = b;
                matchResult->keyPointIndexMap = keyPointIndexMap;
                matchResults.push_back( matchResult );
            }
            else
            {
                delete matchResult;
            }
        }
    }

    ROS_INFO_STREAM ( "Matching took " << ( Clock::getInstance()->getTimestamp() - startTime ) << "ms" );

    // TODO - clean up creation of ROS message for ORMatchResult

    // prepare ROS messages for publishing
    // create image message
    cv_bridge::CvImage result_img;
    result_img.image = *image;

    sensor_msgs::Image image_msg;
    image_msg = *(result_img.toImageMsg());

    // create KeyPoints message
    std::vector<or_msgs::KeyPoint> key_points;
    or_msgs::KeyPoint kp;
    for( unsigned i = 0; i < rawKeyPoints->size(); i++)
    {
        kp.x = rawKeyPoints->at(i).x;
        kp.y = rawKeyPoints->at(i).y;
        kp.scale = rawKeyPoints->at(i).scale;
        kp.strength = rawKeyPoints->at(i).strength;
        kp.orientation = rawKeyPoints->at(i).orientation;
        kp.sign = rawKeyPoints->at(i).sign;
        kp.feature_vector = rawKeyPoints->at(i).featureVector;
        kp.vector_limits = rawKeyPoints->at(i).vectorLimits;
        key_points.push_back(kp);
    }

    // create MatchResult message
    std::vector<or_msgs::MatchResult> match_results;
    or_msgs::MatchResult mr;
    for( unsigned i = 0; i < matchResults.size(); i++)
    {
        mr.object_name = matchResults.at(i)->objectName;
        mr.object_type = matchResults.at(i)->objectType;

        cv_bridge::CvImage result;
        //result.image = adapt_matchresult_image.asIplImage();
        result.image = *image;

        mr.image = *(result.toImageMsg());
        mr.image_index = matchResults.at(i)->imageIndex;
        mr.image_name = matchResults.at(i)->imageName;

        or_msgs::Point2D point_outline;
        for( unsigned outline_index = 0; outline_index < matchResults.at(i)->outline.size(); outline_index++ )
        {
            point_outline.x = matchResults.at(i)->outline.at(outline_index).x();
            point_outline.x = matchResults.at(i)->outline.at(outline_index).y();
            mr.outline.push_back(point_outline);
        }

        or_msgs::Point2D point_bbox;
        for( unsigned bb_index = 0; bb_index < matchResults.at(i)->bBox.size(); bb_index++ )
        {
            point_bbox.x = matchResults.at(i)->bBox.at(bb_index).x();
            point_bbox.x = matchResults.at(i)->bBox.at(bb_index).y();
            mr.outline.push_back(point_bbox);
        }

        mr.center.x = matchResults.at(i)->center.x();
        mr.center.y = matchResults.at(i)->center.y();
        mr.bounding_box_index = matchResults.at(i)->boundingBoxIndex;

        for( unsigned kpim_index = 0; kpim_index < matchResults.at(i)->keyPointIndexMap.size(); kpim_index++ )
        {
            mr.key_point_index_map.push_back(  matchResults.at(i)->keyPointIndexMap.at(kpim_index) );
        }

        or_msgs::KeyPoint obj_kp;
        for( unsigned kp_index = 0; kp_index < matchResults.at(i)->objectKeyPoints.size(); kp_index++)
        {
            obj_kp.x = matchResults.at(i)->objectKeyPoints.at(kp_index).x;
            obj_kp.y = matchResults.at(i)->objectKeyPoints.at(kp_index).y;
            obj_kp.scale = matchResults.at(i)->objectKeyPoints.at(kp_index).scale;
            obj_kp.strength = matchResults.at(i)->objectKeyPoints.at(kp_index).strength;
            obj_kp.orientation = matchResults.at(i)->objectKeyPoints.at(kp_index).orientation;
            obj_kp.sign = matchResults.at(i)->objectKeyPoints.at(kp_index).sign;
            obj_kp.feature_vector = matchResults.at(i)->objectKeyPoints.at(kp_index).featureVector;
            obj_kp.vector_limits = matchResults.at(i)->objectKeyPoints.at(kp_index).vectorLimits;
            mr.object_key_points.push_back(obj_kp);
        }

        or_msgs::KeyPointMatch kp_match_1;
        std::list<KeyPointMatch>::iterator kp_match_iter_1;
        for( kp_match_iter_1 = matchResults.at(i)->stage1Matches.begin(); kp_match_iter_1 != matchResults.at(i)->stage1Matches.end(); kp_match_iter_1++)
        {
            kp_match_1.index1 = kp_match_iter_1->index1;
            kp_match_1.index2 = kp_match_iter_1->index2;
            kp_match_1.distance = kp_match_iter_1->distance;
            kp_match_1.turn_angle = kp_match_iter_1->turnAngle;
            kp_match_1.scale_quotient = kp_match_iter_1->scaleQuotient;
            mr.stage1_matches.push_back( kp_match_1 );
        }


        or_msgs::KeyPointMatchArray kp_match_2_arr;
        for( unsigned st2m_index = 0; st2m_index < matchResults.at(i)->stage2Matches.size(); st2m_index++)
        {
            or_msgs::KeyPointMatch kp_match_2;
            std::list<KeyPointMatch>::iterator kp_match_iter_2;
            for( kp_match_iter_2 = matchResults.at(i)->stage2Matches.at(st2m_index).begin(); kp_match_iter_2 != matchResults.at(i)->stage2Matches.at(st2m_index).end(); kp_match_iter_2++)
            {
                kp_match_2.index1 = kp_match_iter_2->index1;
                kp_match_2.index2 = kp_match_iter_2->index2;
                kp_match_2.distance = kp_match_iter_2->distance;
                kp_match_2.turn_angle = kp_match_iter_2->turnAngle;
                kp_match_2.scale_quotient = kp_match_iter_2->scaleQuotient;
                kp_match_2_arr.key_point_match_array.push_back(kp_match_2);
            }
            mr.stage2_matches.push_back(kp_match_2_arr);
        }

        or_msgs::KeyPointMatch kp_match_3;
        std::list<KeyPointMatch>::iterator kp_match_iter_3;
        for( kp_match_iter_3 = matchResults.at(i)->stage3Matches.begin(); kp_match_iter_3 != matchResults.at(i)->stage3Matches.end(); kp_match_iter_3++)
        {
            kp_match_3.index1 = kp_match_iter_3->index1;
            kp_match_3.index2 = kp_match_iter_3->index2;
            kp_match_3.distance = kp_match_iter_3->distance;
            kp_match_3.turn_angle = kp_match_iter_3->turnAngle;
            kp_match_3.scale_quotient = kp_match_iter_3->scaleQuotient;
            mr.stage3_matches.push_back( kp_match_3 );
        }

        or_msgs::KeyPoint scene_kp;
        for( unsigned scene_kp_index = 0; scene_kp_index < matchResults.at(i)->sceneKeyPointsWithinOutline.size(); scene_kp_index++)
        {
            scene_kp.x = matchResults.at(i)->sceneKeyPointsWithinOutline.at(scene_kp_index).x;
            scene_kp.y = matchResults.at(i)->sceneKeyPointsWithinOutline.at(scene_kp_index).y;
            scene_kp.scale = matchResults.at(i)->sceneKeyPointsWithinOutline.at(scene_kp_index).scale;
            scene_kp.strength = matchResults.at(i)->sceneKeyPointsWithinOutline.at(scene_kp_index).strength;
            scene_kp.orientation = matchResults.at(i)->sceneKeyPointsWithinOutline.at(scene_kp_index).orientation;
            scene_kp.sign = matchResults.at(i)->sceneKeyPointsWithinOutline.at(scene_kp_index).sign;
            scene_kp.feature_vector = matchResults.at(i)->sceneKeyPointsWithinOutline.at(scene_kp_index).featureVector;
            scene_kp.vector_limits = matchResults.at(i)->sceneKeyPointsWithinOutline.at(scene_kp_index).vectorLimits;
            mr.scene_key_points_within_outline.push_back(scene_kp);
        }

        for( unsigned hom_index = 0; hom_index < 9; hom_index++)
        {
            mr.homography.at(hom_index) = matchResults.at(i)->homography.m_HomMat[hom_index];
        }

        match_results.push_back(mr);
    }

    // create boundingbox message
    std::vector<or_msgs::BoundingBox2D> bboxes;
    or_msgs::BoundingBox2D bbox;
    for( unsigned bb_index = 0; bb_index < boundingBoxes.size(); bb_index++)
    {
        bbox.minX = boundingBoxes.at(bb_index).minX();
        bbox.minY = boundingBoxes.at(bb_index).minY();
        bbox.maxX = boundingBoxes.at(bb_index).maxX();
        bbox.maxY = boundingBoxes.at(bb_index).maxY();
        bboxes.push_back(bbox);
    }


    // create OrMatchResult message containing the results of the object recognition
    or_msgs::OrMatchResult or_match_result_msg;
    or_match_result_msg.image = image_msg;
    or_match_result_msg.key_points = key_points;
    or_match_result_msg.match_results = match_results;
    or_match_result_msg.bounding_boxes = bboxes;
    or_match_result_msg.seq_num = seqNum;
    m_ORMatchResultPublisher.publish(or_match_result_msg);
}


bool THIS::matchObject( std::vector< KeyPoint >* sceneKeyPoints, ObjectProperties& objProperties, MatchResult& matchResult )
{
    matchResult.objectName = objProperties.getName();
    matchResult.objectType = objProperties.getType();

    std::vector< ImagePropertiesCV* > objImageProperties = objProperties.getImageProperties();

    std::ostringstream stream;
    std::ostringstream stream2;

    int minMatches = Config::getInt( "ObjectRecognition.iMinMatchedKeyPoints" );
    if ( minMatches < 4 ) { minMatches = 4; }

    bool matchFound = false;
    int maxMatches = 0;

    stream2 << "Image:  #matches stage 1 / stage 2 / stage 3 / ratio / success" << std::endl;

    //loop over all object images
    for ( unsigned i=0; i < objImageProperties.size(); i++ )
    {
        std::list< KeyPointMatch > stage1Matches;

        if(m_Stage1Matcher==Flann)
        {
            stage1Matches = matchStage1Flan( objImageProperties[i] );
        }
        else
        {
            stage1Matches = matchStage1( sceneKeyPoints, objImageProperties[i] );
        }

        std::vector< std::list< KeyPointMatch> > stage2Matches = matchStage2( sceneKeyPoints, objImageProperties[i], stage1Matches );

        //Number of stage2 matches is 0 if vector is empty of length of first entry (sorted in descending order)
        int numMatches2 = stage2Matches.size() > 0 ? stage2Matches[0].size() : 0;

        Homography homography;
        int numMatches3 = 0;
        bool success = false;
        std::list< KeyPointMatch > stage3Matches;
        std::vector<KeyPoint> sceneKeyPointsWithinOutline;

        stage3Matches = matchStage3( sceneKeyPoints, objImageProperties[i], stage2Matches, homography );
        numMatches3 = stage3Matches.size();
        //Compute probability of recognition result
        sceneKeyPointsWithinOutline = getSceneKeyPointsWithinOutline(sceneKeyPoints, homography, stage3Matches);

        double ratio = 0;

        if(sceneKeyPointsWithinOutline.size()==0)
        {
            ratio = 0;
        }
        else
        {
            int numObjFeatures = objImageProperties[i]->getKeyPoints()->size();
            int numSceneFeatures = sceneKeyPointsWithinOutline.size();
            //edit(dg): use smaller number of features for comparison, in case the object and scene image differ very much in size
            int numFeaturesMin = numObjFeatures < numSceneFeatures ? numObjFeatures : numSceneFeatures;
            ratio = (double) numMatches3/numFeaturesMin;
        }

        if(ratio>1)ratio=1; //happens if there are more scene keypoints in bounding box than in real object


        //fMinMatchPercentage of all available features in bounding box of object should have been matched
        if ((numMatches3 >= minMatches) && ratio >= Config::getFloat("ObjectRecognition.fMinMatchPercentage"))
        {
            //      cout << Config::getFloat("ObjectRecognition.fMinMatchPercentage");
            success = true;
        }

        stream2 << i << ": " << stage1Matches.size() << " / " << numMatches2 << " / " << numMatches3;

        stream2 << " / " <<  double(int(ratio*100))/100.0;

        //     stream2 << "Ratio: " <<  ratio << " (numMatches3/sceneKeyPointsWithinOutline) = " << numMatches3 << "/" << sceneKeyPointsWithinOutline.size() << endl;

        if (success)
        {
            stream2 << " X";
            if ( numMatches3 > maxMatches )
            {
                matchResult.imageIndex = i;
                matchResult.imageName = objImageProperties[i]->getName();
                matchResult.stage1Matches = stage1Matches;
                matchResult.stage2Matches = stage2Matches;
                matchResult.stage3Matches = stage3Matches;
                matchResult.sceneKeyPointsWithinOutline = sceneKeyPointsWithinOutline;
                matchResult.homography = homography;
                maxMatches = numMatches3;
                matchFound = true;
            }
        }
        stream2 << endl;
    }

    if ( matchFound )
    {
        matchResult.outline = *( objImageProperties[ matchResult.imageIndex ]->getOutline() );
        matchResult.center = objImageProperties[ matchResult.imageIndex ]->getCenter();

        matchResult.image = objImageProperties[ matchResult.imageIndex ]->getMaskedImageUV();

        matchResult.bBox = objImageProperties[ matchResult.imageIndex ]->getBoundingBox();
        matchResult.objectKeyPoints = *( objImageProperties[ matchResult.imageIndex ]->getKeyPoints() );
        stream2 << "Detected " << objProperties.getType() << ": " << objProperties.getName() << " (image " << matchResult.imageIndex << " " << matchResult.imageName  << ")";
        stream2 << "\nHomography:\n" << matchResult.homography.toString();
    }

    //Write result to "log/orResult.txt"
    std::fstream filestr;
    filestr.open("log/orResult.txt", ios::in | ios::out | ios::ate);

    //FIXME
    string fileName;// = inbox<ImageM> ( MessageTypes::IMAGE_M )->getFileName();

    if ( matchFound )
    {
        double tmpRatio = 0;
        if(matchResult.sceneKeyPointsWithinOutline.size()==0)
        {
            tmpRatio = 0;
        }
        else
        {
            tmpRatio = (double) matchResult.stage3Matches.size()/matchResult.sceneKeyPointsWithinOutline.size();
        }

        if(tmpRatio>1)tmpRatio=1; //happens if there are more scene keypoints in bounding box than in real object
        filestr << fileName << " -> detected " <<  ": " << matchResult.objectName << " (image " << matchResult.imageIndex << " " << matchResult.imageName << " #" << matchResult.objectKeyPoints.size() << "# ) *" << matchResult.stage3Matches.size() << "* with ratio "<< tmpRatio << "\n";
    }
    else
    {
        filestr << fileName << " -> failed\n";
    }
    filestr.close();

    ROS_INFO_STREAM( stream2.str() );
    ROS_DEBUG_STREAM( stream.str() );

    return matchFound;
}



std::list< KeyPointMatch > THIS::matchStage1( vector< KeyPoint > *sceneKeyPoints, ImagePropertiesCV *objImageProperties )
{
    int startTime = Clock::getInstance()->getTimestamp();

    vector< KeyPoint >* objectImageKeyPoints=objImageProperties->getKeyPoints();

    float maxNNR = Config::getFloat( "ObjectRecognition.NNRMatching.fMaxNearestNeighbourRatio" );
    NNRMatcher nnrMatcher( sceneKeyPoints, objectImageKeyPoints );
    nnrMatcher.match( maxNNR );

    ROS_INFO_STREAM ( "Nearest-Neighbor Matching of " << sceneKeyPoints->size() << " vs " << objectImageKeyPoints->size() << " keypoints took " << ( Clock::getInstance()->getTimestamp() - startTime ) << "ms" );

    ROS_DEBUG_STREAM( nnrMatcher.getLog() );
    return nnrMatcher.getMatches();
}



std::list< KeyPointMatch > THIS::matchStage1Flan( ImagePropertiesCV *objImageProperties )
{
    int startTime = Clock::getInstance()->getTimestamp();

    vector< KeyPoint >* objectImageKeyPoints=objImageProperties->getKeyPoints();

    float maxNNR = Config::getFloat( "ObjectRecognition.NNRMatching.fMaxNearestNeighbourRatio" );

    m_FlannMatcher->match(objectImageKeyPoints,maxNNR);

    ROS_INFO_STREAM ( "Nearest-Neighbor Matching with FLAN found " << m_FlannMatcher->getNumMatches() << " of " << objectImageKeyPoints->size() << " keypoints took " << ( Clock::getInstance()->getTimestamp() - startTime ) << "ms" );

    ROS_INFO_STREAM( m_FlannMatcher->getLog() );
    return m_FlannMatcher->getMatches();
}



bool sizeComp( std::list< KeyPointMatch> list1, std::list< KeyPointMatch> list2 )
{
    return list1.size() > list2.size();
}



std::vector< std::list< KeyPointMatch> > THIS::matchStage2( vector< KeyPoint > *sceneKeyPoints, ImagePropertiesCV *objImageProperties, std::list< KeyPointMatch > &stage1Matches )
{
    int startTime = Clock::getInstance()->getTimestamp();

    vector< KeyPoint >* objectImageKeyPoints=objImageProperties->getKeyPoints();

    MatchHelper::calcScaleQuotients( sceneKeyPoints, objectImageKeyPoints, stage1Matches );
    MatchHelper::calcTurnAngles( sceneKeyPoints, objectImageKeyPoints, stage1Matches );

    switch ( m_Stage2Matcher )
    {
    case SimpleHoughClustering:
    {
        ROS_DEBUG_STREAM("matchStage2 -> SimpleHoughClustering");

        /* perform Hough Clustering on matches */
        SimpleHoughClusterer houghClusterer( sceneKeyPoints, objectImageKeyPoints, stage1Matches );
        houghClusterer.eliminateByOrientation();
        houghClusterer.eliminateByScale();
        houghClusterer.eliminateByPosition( Config::getFloat( "ObjectRecognition.SimpleHoughClustering.fMaxMatchDistance" ) );
        ROS_DEBUG_STREAM( houghClusterer.getLog() );
        vector< std::list< KeyPointMatch> > matches;
        matches.push_back(houghClusterer.getMatches());

        ROS_INFO_STREAM ( "SimpleHoughClustering took " << ( Clock::getInstance()->getTimestamp() - startTime ) << "ms" );
        return matches;
    }

    case HoughClustering:
    {
        ROS_DEBUG_STREAM("matchStage2 -> HoughClustering");

        /* perform Hough Clustering on matches */
        /*
      TRACE_SYSTEMINFO("colorFormat: "<<  inbox<ImageM> ( MessageTypes::IMAGE_M )->getColorFormat());

      int w = inbox<ImageM> ( MessageTypes::IMAGE_M )->getRgbImage()->getWidth();
      int h = inbox<ImageM> ( MessageTypes::IMAGE_M )->getRgbImage()->getHeight();

      TRACE_SYSTEMINFO("w h ok");*/

        HoughClusterer houghClusterer( sceneKeyPoints, objImageProperties->getKeyPoints(), objImageProperties->getCenter(), m_ImageWidth, m_ImageHeight );

        houghClusterer.setNNMatches(stage1Matches);

        vector< std::list< KeyPointMatch> > matches = houghClusterer.clusterAccumulator();

        if(Config::getInstance()->getBool( "ObjectRecognition.HoughClustering.bPlot" ))
        {
            std::string path = ros::package::getPath("or_nodes");

            cv::Mat* guiImageNN = new cv::Mat();
            houghClusterer.getImage( *guiImageNN );
            cv::imwrite(path + "/images/ORHoughAccumulatorNN.ppm", *guiImageNN);

            cv::Mat* guiImageClustered = new cv::Mat();
            houghClusterer.getImage( *guiImageClustered );
            cv::imwrite(path + "/images/ORHoughAccumulatorClustered.ppm", *guiImageClustered);

            cv::Mat* guiImageDiff = new cv::Mat();
            guiImageDiff->resize(guiImageNN->rows, guiImageNN->cols);

            for ( int y=0; y<guiImageNN->rows; y++ )
            {
                for ( int x=0; x<guiImageNN->cols; x++ )
                {
                    cv::Vec3b diffVec = guiImageNN->at<cv::Vec3b>(y,x) - guiImageClustered->at<cv::Vec3b>(y,x);
                    guiImageDiff->at<cv::Vec3b>(y,x) = diffVec;
                }
            }
            cv::imwrite(path + "/images/ORHoughAccumulatorDiff.ppm", *guiImageDiff);

            delete guiImageNN;
            delete guiImageClustered;
            delete guiImageDiff;
        }

        ROS_INFO_STREAM ( "HoughClustering took " << ( Clock::getInstance()->getTimestamp() - startTime ) << "ms, " << matches.size() << " hypotheses found." );
        ROS_DEBUG_STREAM( houghClusterer.getLog() );

        std::sort( matches.begin(), matches.end(), sizeComp );

        return matches;
    }

    default:
    {
        std::vector< std::list< KeyPointMatch> > matches;
        matches.push_back( stage1Matches );
        return matches;
    }
    }

    return std::vector< std::list< KeyPointMatch> > ();
}


std::list< KeyPointMatch > THIS::matchStage3( std::vector< KeyPoint > *sceneKeyPoints,
                                              ImagePropertiesCV *objImageProperties,
                                              std::vector< std::list< KeyPointMatch> > &stage2Matches,
                                              Homography &homography )
{
    ROS_DEBUG_STREAM("-------- Homography -----------\n\n");

    if(stage2Matches.size()==0)
    {
        ROS_DEBUG_STREAM("No KeyPointMatch available for homography");
        return std::list< KeyPointMatch>();
    }

    int startTime = Clock::getInstance()->getTimestamp();

    //iterate stage2 matches; remember max homography; check only stage2 matches with more matches than best homography has

    std::list< KeyPointMatch> maxMatches;
    Homography maxHomography;

    int maxHomographies=5;
    int numHomographies = stage2Matches.size();

    if ( numHomographies > maxHomographies )
    {
        numHomographies = maxHomographies;
    }

    ROS_INFO_STREAM( "Calculating homographies for " << numHomographies << " of " << stage2Matches.size() << " hypotheses." );

    //Iterate over all bins
    for(int i=0;i<numHomographies;++i)
    {

        std::list< KeyPointMatch> stage2MatchList = stage2Matches[i];

        if(stage2MatchList.size()<=maxMatches.size())
        {
            ROS_DEBUG_STREAM("Stop checking bins -> bin "<< i << " has less or equal entries with " << stage2MatchList.size() << " matches compared to max matches " << maxMatches.size() << ".");
            break;
        }

        ROS_DEBUG_STREAM("Checking bin "<< i << " with " << stage2MatchList.size() << " matches.");

        /*
    if(i>= Config::getInt( "ObjectRecognition.Homography.iMaxBins"))
    {
      TRACE_INFO("Stop checking bins -> bin "<< i << ", because iMaxBins was reached.");
      break;
    }*/

        //compute homography for current bin
        CvHomography cvHomography( sceneKeyPoints, objImageProperties->getKeyPoints(), stage2MatchList );
        if ( cvHomography.computeHomography() )
        {
            //check if the object's bounding box can be transformed with the homography
            vector< Point2D > bBox = objImageProperties->getBoundingBox();
            homography = cvHomography.getHomography();
            if ( homography.checkValidity( bBox ) )
            {
                cvHomography.eliminateBadMatches();
                std::list< KeyPointMatch> homMatches = cvHomography.getMatches();

                //if best bin is worse than current homography take this bin
                if(maxMatches.size()<homMatches.size())
                {
                    ROS_DEBUG_STREAM("-> Homography is better with " << homMatches.size() << " matches instead of " << maxMatches.size() << " matches.");
                    maxMatches = homMatches;
                    maxHomography = homography;
                }
                else
                {
                    ROS_DEBUG_STREAM("-> Homography is worse or equal with " << homMatches.size() << " matches instead of " << maxMatches.size() << " matches.");
                }
            }
            else
            {
                ROS_DEBUG_STREAM("-> Homography is invalid");
            }
        }
        else
        {
            ROS_DEBUG_STREAM("-> Bin has no homography");
        }
    }

    homography = maxHomography;

    ROS_INFO_STREAM ( "Homography took " << ( Clock::getInstance()->getTimestamp() - startTime ) << "ms" );
    return maxMatches;
}

vector<KeyPoint> THIS::getSceneKeyPointsWithinOutline(vector< KeyPoint >* sceneKeyPoints, Homography& homography, std::list< KeyPointMatch >& stage3Matches)
{
    int startTime = Clock::getInstance()->getTimestamp();

    vector<KeyPoint> sceneKeyPointsWithinOutline;

    if(stage3Matches.size()==0)
    {
        ROS_DEBUG_STREAM( "Getting scene points within outline without stage3 matches took " << ( Clock::getInstance()->getTimestamp() - startTime ) << "ms" );
        return sceneKeyPointsWithinOutline;
    }

    // Project sceneKeyPoints with homography to get number of features within object outline (KeyPoint for display, Point2D to compute if valid)
    std::vector< std::pair< KeyPoint, Point2D> > projSceneKeyPoints;

    //project sceneKeyPoints with homography
    for ( vector< KeyPoint>::iterator match = sceneKeyPoints->begin(); match != sceneKeyPoints->end(); match++ )
    {
        Point2D point = homography.transform(match->position());
        if(point.isValid()) projSceneKeyPoints.push_back(std::pair< KeyPoint, Point2D>(*match,point));
    }

    if(projSceneKeyPoints.size()>0)
    {
        //Project stage3Matches with homography to get bounding box of outline
        std::vector<Point2D> projObjKeyPoints;
        for ( std::list<KeyPointMatch>::iterator match = stage3Matches.begin(); match != stage3Matches.end(); match++ )
        {
            Point2D point = homography.transform(sceneKeyPoints->at(match->index1).position());
            projObjKeyPoints.push_back(point);
        }

        //Get bounding box of projected object outline to check if scene keypoints are within this area

        int xmin = 0;
        int ymin = 0;
        int xmax = 0;
        int ymax = 0;

        if (projObjKeyPoints.size() > 0) {
            xmin = projObjKeyPoints.at(0).x();
            xmax = projObjKeyPoints.at(0).x();
            ymin = projObjKeyPoints.at(0).y();
            ymax = projObjKeyPoints.at(0).y();
        }

        for (unsigned int j=1;j<projObjKeyPoints.size();j++)
        {
            if ( projObjKeyPoints.at(j).x() < xmin ) xmin = projObjKeyPoints.at(j).x();
            if ( projObjKeyPoints.at(j).x() > xmax ) xmax = projObjKeyPoints.at(j).x();
            if ( projObjKeyPoints.at(j).y() < ymin ) ymin = projObjKeyPoints.at(j).y();
            if ( projObjKeyPoints.at(j).y() > ymax ) ymax = projObjKeyPoints.at(j).y();
        }

        for(unsigned int j=0;j<projSceneKeyPoints.size();++j)
        {
            int posX = projSceneKeyPoints.at(j).second.x();
            int posY = projSceneKeyPoints.at(j).second.y();
            if(posX>=xmin && posX<=xmax && posY>=ymin && posY<=ymax)
            {
                sceneKeyPointsWithinOutline.push_back(projSceneKeyPoints.at(j).first);
            }
        }
    }

    ROS_DEBUG_STREAM( "Getting scene points within outline took " << ( Clock::getInstance()->getTimestamp() - startTime ) << "ms" );

    return sceneKeyPointsWithinOutline;
}

void THIS::addObjectProperties(ObjectProperties* newProperties)
{
    ostringstream stream;
    stream << "Adding object: " << endl;
    newProperties->printOn( stream );
    ROS_INFO_STREAM( stream.str() );

    for ( unsigned i=0; i<m_ObjectList.size(); i++ )
    {
        if ( m_ObjectList[i].getName() == newProperties->getName() )
        {
            ROS_WARN_STREAM( "Object " << m_ObjectList[i].getName() << " already loaded. Replacing." );
            removeObjectProperties( m_ObjectList[i].getName() );
        }
    }

    m_ObjectList.push_back( *newProperties );

    sendObjectNames();
}


void THIS::removeObjectProperties(std::string name)
{
    std::deque<ObjectProperties>::iterator iterProperties;

    for ( iterProperties = m_ObjectList.begin(); iterProperties != m_ObjectList.end(); ++iterProperties )
    {
        if( name == iterProperties->getName() )
        {
            ROS_WARN_STREAM( "Unloading object " << name );
            m_ObjectList.erase(iterProperties);
            return;
        }
    }
    sendObjectNames();
}


void THIS::sendObjectNames()
{
    // Return changed objectList to GUI
    or_msgs::OrObjectNames obj_names_msg;
    std::deque<ObjectProperties>::iterator iter;
    for (iter = m_ObjectList.begin(); iter != m_ObjectList.end(); ++iter)
    {
        obj_names_msg.object_names.push_back(iter->getName());
        obj_names_msg.object_types.push_back(iter->getType());
    }

    m_ORObjectNamesPublisher.publish( obj_names_msg );
}

#undef THIS
