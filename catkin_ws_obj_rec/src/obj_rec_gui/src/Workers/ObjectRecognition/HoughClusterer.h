/*******************************************************************************
 *  HoughClusterer.h
 *
 *  (C) 2007 AG Aktives Sehen <agas@uni-koblenz.de>
 *           Universitaet Koblenz-Landau
 *
 *  Additional information:
 *  $Id: $
 *******************************************************************************/

#ifndef HoughClusterer_H
#define HoughClusterer_H

#include "Workers/KeyPointExtraction/KeyPoint.h"
#include <vector>
#include <deque>
#include <sstream>
#include <list>

#include "Workers/ObjectRecognition/HoughAccumulator.h"
#include "Workers/KeyPointExtraction/KeyPointMatch.h"
#include "Workers/Puma2/ColorImageRGB8.h"
#include "Workers/Math/Math.h"
#include "Workers/Math/Box2D.h"

class ImageProperties;

/**
 * @class  HoughClusterer
 * @brief  Clusters feature matches using hough transform clustering
 * @author Susanne Thierfelder (R12)
 */
class HoughClusterer
{
public:

  HoughClusterer(){}

 /** @brief The constructor builds 4-dimensional accumulator-array of scale, orientation and 2-D-location
  *  @param sceneKeyPoints List of keypoints of the scene
  *  @param objectImageKeyPoints List of keypoints from learned object
  *  @param nnrMatches MatchList with indices
  */
  HoughClusterer( std::vector< KeyPoint >* sceneKeyPoints, std::vector< KeyPoint >* objectImageKeyPoints, Point2D center, int imageWidth, int imageHeight);

  /** @brief The destructor */
  ~HoughClusterer();

  std::string getLog();

  void getImage( puma2::ColorImageRGB8& target );

  float getVariance();

  void setNNMatches(std::list< KeyPointMatch > nnrMatches);
  std::vector< std::list< KeyPointMatch> > clusterAccumulator();

private:

  void incrAccumulatorValue(KeyPoint scenePoint, KeyPoint objectPoint, KeyPointMatch match);
  //Get highest accumulator value for keypoint values for rotation, scale and position; if nothing found return -1
  bool getAccumulatorValue(KeyPoint scenePoint, KeyPoint objectPoint, unsigned int& value);

  std::vector< KeyPoint >* m_SceneKeyPoints;
  std::vector< KeyPoint >* m_ObjectImageKeyPoints;

  Point2D m_Center;

  std::ostringstream m_Log;

  HoughAccumulator* m_HoughAccumulator;

  int m_ScaleBins;
  int m_OrientationBins;
  int m_XLocationBins;
  int m_YLocationBins;

  int m_ImageWidth;
  int m_ImageHeight;
};

#endif
