/*******************************************************************************
 *  HoughIndexCalculator.h
 *
 *  (C) 2007 AG Aktives Sehen <agas@uni-koblenz.de>
 *           Universitaet Koblenz-Landau
 *
 *  Additional information:
 *  $Id: $
 *******************************************************************************/

#ifndef HoughIndexCalculator_H
#define HoughIndexCalculator_H

#include "../KeyPointExtraction/KeyPoint.h"

/**
 * @class  HoughIndexCalculator
 * @brief  Calculates indices for HoughClustering
 * @author Susanne Thierfelder (R12)
 */
class HoughIndexCalculator
{
  public:
    static void calculateScaleIndex(double scaleQuotient, int& scaleIndexFloor, int& scaleIndexCeil);
    static void calculateOrientationIndex(double turnAngle, int& orientationFloor, int& orientationCeil);
    static void calculatePositionIndex(KeyPoint sceneKeyPoint, KeyPoint objectKeyPoint, Point2D center, int w, int h, int& xDistanceFloor, int& xDistanceCeil, int& yDistanceFloor, int& yDistanceCeil);

};

#endif
