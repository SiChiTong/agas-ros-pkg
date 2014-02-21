/*******************************************************************************
 *  HoughClusterer.cpp
 *
 *  (C) 2007 AG Aktives Sehen <agas@uni-koblenz.de>
 *           Universitaet Koblenz-Landau
 *
 *  Additional information:
 *  $Id: $
 *******************************************************************************/

#include "HoughClusterer.h"

#include "Architecture/Config/Config.h"
#include "Architecture/Singleton/Clock.h"
#include "HoughIndexCalculator.h"
#include "Workers/Math/vec2.h"

#include <ros/ros.h>
#include <algorithm>    // for max_element
#include <assert.h>
#include <map>
#include <list>
#include <math.h>
#include <cstring>

#include <fstream>

using namespace std;

#define THIS HoughClusterer

THIS::THIS( vector< KeyPoint >* sceneKeyPoints, vector< KeyPoint >* objectImageKeyPoints, Point2D center, int imageWidth, int imageHeight)
{
  m_Log << endl << "-------- Hough Transform Clustering -----------\n\n";

  m_SceneKeyPoints = sceneKeyPoints;

  m_ObjectImageKeyPoints = objectImageKeyPoints;
  m_Center = center;

  m_ImageWidth = imageWidth;
  m_ImageHeight = imageHeight;

  /* Build 4-dim accumulator-array of: scale, orientation, x-location, y-location */
  m_ScaleBins = Config::getInt( "ObjectRecognition.HoughClustering.iScaleBins" );
  m_OrientationBins = Config::getInt( "ObjectRecognition.HoughClustering.iOrientationBins" );
  m_XLocationBins = Config::getInt( "ObjectRecognition.HoughClustering.iXLocationBins" );
  m_YLocationBins = Config::getInt( "ObjectRecognition.HoughClustering.iYLocationBins" );

  m_Log << "HoughClusterer [" << m_ScaleBins<< "*"<< m_OrientationBins << "*"<< m_XLocationBins <<"*"<<m_YLocationBins<< "]"<< " created -> ";

  int startTime = Clock::getInstance()->getTimestamp();
  m_HoughAccumulator = new HoughAccumulator();
  m_Log <<  "building accumulator took " << Clock::getInstance()->getTimestamp() - startTime << " ms \n\n";
}


void THIS::setNNMatches(std::list< KeyPointMatch > nnrMatches)
{
  m_Log << "Filling accumulator with " << nnrMatches.size() << " matches \n";

  int startTime = Clock::getInstance()->getTimestamp();

  std::list< KeyPointMatch >::iterator it;
  it = nnrMatches.begin();
  while(it!=nnrMatches.end())
  {
    incrAccumulatorValue(m_SceneKeyPoints->at(it->index1), m_ObjectImageKeyPoints->at(it->index2),*it);
    ++it;
  }

  m_Log <<  "-> filling accumulator took " << Clock::getInstance()->getTimestamp() - startTime << " ms \n\n";
}

vector< std::list< KeyPointMatch> > THIS::clusterAccumulator()
{
  m_Log << "Clustering Accumulator = " << "( + -> keep, - -> delete, x -> out of bounds -> index/value )\n\n";

  int startTime = Clock::getInstance()->getTimestamp();

  vector< std::list< KeyPointMatch> > newMatches;

  if(Config::getInt( "ObjectRecognition.HoughClustering.iAccumulatorSearchStrategy")==0)
  {
    newMatches = m_HoughAccumulator->getClusteredMatches();
  }
  else
  {
    ROS_INFO_STREAM("Find maximum in accumulator");
    newMatches = m_HoughAccumulator->getMaximumMatches();
  }

  m_Log << m_HoughAccumulator->getLog();
  m_Log << " -> clustering and re-filling took " << ( Clock::getInstance()->getTimestamp() - startTime ) << " ms \n\n";
  m_Log << "Number of clusters after hough clustering: " << newMatches.size() << endl;

  startTime = Clock::getInstance()->getTimestamp();
  m_Log << "Sorting matches in descending order took " << ( Clock::getInstance()->getTimestamp() - startTime ) << " ms \n\n";

  return newMatches;
}

void THIS::incrAccumulatorValue(KeyPoint scenePoint, KeyPoint objectPoint, KeyPointMatch match)
{
  /* Scale index*/

  int scaleIndexFloor;
  int scaleIndexCeil;

  double scaleQuotient = scenePoint.scale/objectPoint.scale;
  HoughIndexCalculator::calculateScaleIndex(scaleQuotient, scaleIndexFloor, scaleIndexCeil);

  /* Orientation index */

  int orientationFloor;
  int orientationCeil;

  double orientationQuotient = Math::minTurnAngle(scenePoint.orientation, objectPoint.orientation);
  HoughIndexCalculator::calculateOrientationIndex(orientationQuotient, orientationFloor, orientationCeil);

  /* Position index */

  int xDistanceFloor;
  int xDistanceCeil;

  int yDistanceFloor;
  int yDistanceCeil;

  HoughIndexCalculator::calculatePositionIndex(scenePoint, objectPoint, m_Center, m_ImageWidth, m_ImageHeight, xDistanceFloor, xDistanceCeil, yDistanceFloor, yDistanceCeil);

  /* each keypoint match votes for the 2 closest bins in each dimension of the accumulator
       -> 16 entries in case that the position is inside accumulator borders
    */

  int successCounter = 0;

  if(m_HoughAccumulator->incrAccumulatorValue(scaleIndexFloor,orientationFloor,xDistanceFloor,yDistanceFloor, match))
    successCounter++;
  if(m_HoughAccumulator->incrAccumulatorValue(scaleIndexFloor,orientationFloor,xDistanceFloor,yDistanceCeil, match))
    successCounter++;
  if(m_HoughAccumulator->incrAccumulatorValue(scaleIndexFloor,orientationFloor,xDistanceCeil,yDistanceFloor, match))
    successCounter++;
  if(m_HoughAccumulator->incrAccumulatorValue(scaleIndexFloor,orientationFloor,xDistanceCeil,yDistanceCeil, match))
    successCounter++;
  if(m_HoughAccumulator->incrAccumulatorValue(scaleIndexFloor,orientationCeil,xDistanceFloor,yDistanceFloor, match))
    successCounter++;
  if(m_HoughAccumulator->incrAccumulatorValue(scaleIndexFloor,orientationCeil,xDistanceFloor,yDistanceCeil, match))
    successCounter++;
  if(m_HoughAccumulator->incrAccumulatorValue(scaleIndexFloor,orientationCeil,xDistanceCeil,yDistanceFloor, match))
    successCounter++;
  if(m_HoughAccumulator->incrAccumulatorValue(scaleIndexFloor,orientationCeil,xDistanceCeil,yDistanceCeil, match))
    successCounter++;
  if(m_HoughAccumulator->incrAccumulatorValue(scaleIndexCeil,orientationFloor,xDistanceFloor,yDistanceFloor, match))
    successCounter++;
  if(m_HoughAccumulator->incrAccumulatorValue(scaleIndexCeil,orientationFloor,xDistanceFloor,yDistanceCeil, match))
    successCounter++;
  if(m_HoughAccumulator->incrAccumulatorValue(scaleIndexCeil,orientationFloor,xDistanceCeil,yDistanceFloor, match))
    successCounter++;
  if(m_HoughAccumulator->incrAccumulatorValue(scaleIndexCeil,orientationFloor,xDistanceCeil,yDistanceCeil, match))
    successCounter++;
  if(m_HoughAccumulator->incrAccumulatorValue(scaleIndexCeil,orientationCeil,xDistanceFloor,yDistanceFloor, match))
    successCounter++;
  if(m_HoughAccumulator->incrAccumulatorValue(scaleIndexCeil,orientationCeil,xDistanceFloor,yDistanceCeil, match))
    successCounter++;
  if(m_HoughAccumulator->incrAccumulatorValue(scaleIndexCeil,orientationCeil,xDistanceCeil,yDistanceFloor, match))
    successCounter++;
  if(m_HoughAccumulator->incrAccumulatorValue(scaleIndexCeil,orientationCeil,xDistanceCeil,yDistanceCeil, match))
    successCounter++;

  if(successCounter==0)
  {
    m_Log << "Incrementing match at scale "<< scaleQuotient << " orientation " << orientationQuotient << " posX "<< scenePoint.x << " posY "<< scenePoint.y << " failed." << endl;
  }
}

bool THIS::getAccumulatorValue(KeyPoint scenePoint, KeyPoint objectPoint, unsigned int& value)
{
  /* Scale index*/

  int scaleIndexFloor;
  int scaleIndexCeil;

  double scaleQuotient = scenePoint.scale/objectPoint.scale;
  HoughIndexCalculator::calculateScaleIndex(scaleQuotient, scaleIndexFloor, scaleIndexCeil);

  /* Orientation index */

  int orientationFloor;
  int orientationCeil;

  double orientationQuotient = Math::minTurnAngle(scenePoint.orientation, objectPoint.orientation);
  HoughIndexCalculator::calculateOrientationIndex(orientationQuotient, orientationFloor, orientationCeil);

  /* Position index */

  int xDistanceFloor;
  int xDistanceCeil;

  int yDistanceFloor;
  int yDistanceCeil;

  HoughIndexCalculator::calculatePositionIndex(scenePoint, objectPoint, m_Center, m_ImageWidth, m_ImageHeight, xDistanceFloor, xDistanceCeil, yDistanceFloor, yDistanceCeil);

  std::vector<int> accumulatorValues;

  unsigned int val;

  if(m_HoughAccumulator->getAccumulatorValue(scaleIndexFloor,orientationFloor,xDistanceFloor,yDistanceFloor,val))
    accumulatorValues.push_back(val);

  if(m_HoughAccumulator->getAccumulatorValue(scaleIndexFloor,orientationFloor,xDistanceFloor,yDistanceFloor,val))
    accumulatorValues.push_back(val);

  if(m_HoughAccumulator->getAccumulatorValue(scaleIndexFloor,orientationFloor,xDistanceFloor,yDistanceCeil,val))
    accumulatorValues.push_back(val);

  if(m_HoughAccumulator->getAccumulatorValue(scaleIndexFloor,orientationFloor,xDistanceCeil,yDistanceFloor,val))
    accumulatorValues.push_back(val);

  if(m_HoughAccumulator->getAccumulatorValue(scaleIndexFloor,orientationFloor,xDistanceCeil,yDistanceCeil,val))
    accumulatorValues.push_back(val);

  if(m_HoughAccumulator->getAccumulatorValue(scaleIndexFloor,orientationCeil,xDistanceFloor,yDistanceFloor,val))
    accumulatorValues.push_back(val);

  if(m_HoughAccumulator->getAccumulatorValue(scaleIndexFloor,orientationCeil,xDistanceFloor,yDistanceCeil,val))
    accumulatorValues.push_back(val);

  if(m_HoughAccumulator->getAccumulatorValue(scaleIndexFloor,orientationCeil,xDistanceCeil,yDistanceFloor,val))
    accumulatorValues.push_back(val);

  if(m_HoughAccumulator->getAccumulatorValue(scaleIndexFloor,orientationCeil,xDistanceCeil,yDistanceCeil,val))
    accumulatorValues.push_back(val);

  if(m_HoughAccumulator->getAccumulatorValue(scaleIndexCeil,orientationFloor,xDistanceFloor,yDistanceFloor,val))
    accumulatorValues.push_back(val);

  if(m_HoughAccumulator->getAccumulatorValue(scaleIndexCeil,orientationFloor,xDistanceFloor,yDistanceCeil,val))
    accumulatorValues.push_back(val);

  if(m_HoughAccumulator->getAccumulatorValue(scaleIndexCeil,orientationFloor,xDistanceCeil,yDistanceFloor,val))
    accumulatorValues.push_back(val);

  if(m_HoughAccumulator->getAccumulatorValue(scaleIndexCeil,orientationFloor,xDistanceCeil,yDistanceCeil,val))
    accumulatorValues.push_back(val);

  if(m_HoughAccumulator->getAccumulatorValue(scaleIndexCeil,orientationCeil,xDistanceFloor,yDistanceFloor,val))
    accumulatorValues.push_back(val);

  if(m_HoughAccumulator->getAccumulatorValue(scaleIndexCeil,orientationCeil,xDistanceFloor,yDistanceCeil,val))
    accumulatorValues.push_back(val);

  if(m_HoughAccumulator->getAccumulatorValue(scaleIndexCeil,orientationCeil,xDistanceCeil,yDistanceFloor,val))
    accumulatorValues.push_back(val);

  if(m_HoughAccumulator->getAccumulatorValue(scaleIndexCeil,orientationCeil,xDistanceCeil,yDistanceCeil,val))
    accumulatorValues.push_back(val);

  if(accumulatorValues.size()==0)
  {
    value =  0;
    m_Log << "Reading match "<< scaleQuotient << " " << orientationQuotient << " "<< scenePoint.x << " "<< scenePoint.y << " failed. \n";
    return false;
  }
  else
  {
    value = *std::max_element(accumulatorValues.begin(), accumulatorValues.end());
    return true;
  }
}

float THIS::getVariance()
{
  return m_HoughAccumulator->getVariance();
}

void THIS::getImage( cv::Mat& target )
{
  m_HoughAccumulator->getImage(target);
}


string THIS::getLog()
{
  return m_Log.str();
}


THIS::~THIS()
{
  delete m_HoughAccumulator;
}

#undef THIS
