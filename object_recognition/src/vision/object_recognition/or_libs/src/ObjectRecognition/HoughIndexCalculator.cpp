/*******************************************************************************
 *  HoughIndexCalculator.cpp
 *
 *  (C) 2007 AG Aktives Sehen <agas@uni-koblenz.de>
 *           Universitaet Koblenz-Landau
 *
 *  Additional information:
 *  $Id: $
 *******************************************************************************/

#include "HoughIndexCalculator.h"

#include "Architecture/Config/Config.h"
#include "Architecture/Singleton/Clock.h"

#include "Workers/Math/Math.h"
#include "Workers/Math/vec2.h"

#include <algorithm>    // for max_element
#include <assert.h>
#include <map>
#include <list>
#include <math.h>
#include <cstring>
#include <fstream>
#include <ros/ros.h>

using namespace std;

#define THIS HoughIndexCalculator

void THIS::calculateScaleIndex(double scaleQuotient, int& scaleIndexFloor, int& scaleIndexCeil)
{
  int scaleBins = Config::getInt( "ObjectRecognition.HoughClustering.iScaleBins" );

  //( log2( scaleQuotient ) -> anzahl der verdoppelungen(octaven) und halbierungen(octaven)  log2( ]0..inf] ) = [-inf..inf]
  //( log2( scaleQuotient ) / (maxOctaves-1) -> [-inf..inf] (-1 damit scalierung 1 und nicht nur octaven dazukommen)
  //( log2( scaleQuotient ) / (maxOctaves-1) / 2 + 0.5 ) -> wertebereich verschieben (bei scaleQuotient < 1 ist log2 negativ)
  //( log2( scaleQuotient ) / (maxOctaves-1) / 2 + 0.5 ) * scaleBins; -> auf anzahl der bins hochskalieren

  //Bsp: scaleQuotient 0.5 -> bin 3
  //Bsp: scaleQuotient 1 -> bin 5, also noch verdoppelungen und 4 halbierungen möglich
  //Bsp: scaleQuotient 2 -> bin 6

  /* Scale Index: a factor of 2 for scale (log2), 1/4 size to 4 times */
  int maxOctaves=5;
  float scaleIndex = ( log2( scaleQuotient ) / (maxOctaves-1) / 2 + 0.5 ) * scaleBins;

  //put all scales that are too large in last bin for ceil
  if ( scaleIndex >= scaleBins ) {
    scaleIndex = scaleBins-1;
  }

  if ( scaleIndex < 0 ) {
    scaleIndex = 0;
  }

  scaleIndexFloor = scaleIndex;
  scaleIndexCeil = scaleIndex+1;

  if(scaleIndexCeil >= scaleBins)
  {
    scaleIndexCeil = scaleBins-1;
  }
}

void THIS::calculateOrientationIndex(double turnAngle, int& orientationFloor, int& orientationCeil)
{
  int orientationBins = Config::getInt( "ObjectRecognition.HoughClustering.iOrientationBins" );

  //turnAngle in radians (-PI to PI)
  
  if(turnAngle<-M_PI || turnAngle>M_PI)
  {
    ROS_ERROR_STREAM("Orientation "<< turnAngle);
  }

  //( turnAngle+M_PI ) -> [0..2*PI]
  //( turnAngle+M_PI ) /M_PI/2.0 -> [0..1]
  //( turnAngle+M_PI ) /M_PI/2.0 * orientationBins -> [0..orientationBins] float

  float orientationIndex = ( turnAngle+M_PI ) /M_PI/2.0 * orientationBins;

  assert( orientationIndex >= 0.0 );

  //int(orientationIndex) % orientationBins; -> [0..orientationBins] int
  orientationFloor = int(orientationIndex) % orientationBins;
  orientationCeil = ( orientationFloor+1 ) % orientationBins;
}

void THIS::calculatePositionIndex(KeyPoint sceneKeyPoint, KeyPoint objectKeyPoint, Point2D center, int w, int h, int& xDistanceFloor, int& xDistanceCeil, int& yDistanceFloor, int& yDistanceCeil)
{
  int xLocationBins = Config::getInt( "ObjectRecognition.HoughClustering.iXLocationBins" );
  int yLocationBins = Config::getInt( "ObjectRecognition.HoughClustering.iYLocationBins" );

  //calculate offset between object and scene keypoints
  Point2D objectPoint = objectKeyPoint.position();
  Point2D scenePoint = sceneKeyPoint.position();

  //points from objectPoint to objectCenter
  Point2D v = center - objectPoint;
  v *= sceneKeyPoint.scale/ objectKeyPoint.scale;
  v.rotate ( Math::minTurnAngle(sceneKeyPoint.orientation, objectKeyPoint.orientation) );

  //position of object center in scene
  Point2D centerScene = v + scenePoint;

  //compute index depending on image dimension
  float xPositionIndex = (centerScene.x()/w)*xLocationBins;
  float yPositionIndex = (centerScene.y()/h)*yLocationBins;

  //values could be equal at border (don't just use ceil() and floor() )
  xDistanceFloor = xPositionIndex;
  xDistanceCeil = xDistanceFloor+1;

  yDistanceFloor = yPositionIndex;
  yDistanceCeil = yDistanceFloor+1;
}

#undef THIS
