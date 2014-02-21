/*******************************************************************************
 *  HoughAccumulator.h
 *
 *  (C) 2007 AG Aktives Sehen <agas@uni-koblenz.de>
 *           Universitaet Koblenz-Landau
 *******************************************************************************/

#ifndef HoughAccumulator_H
#define HoughAccumulator_H

#include "Workers/KeyPointExtraction/KeyPointMatch.h"
#include "Workers/Puma2/ColorImageRGB8.h"
#include "Workers/Math/Point2D.h"

#include <vector>
#include <list>
#include <sstream>

/**
 * @class  HoughAccumulator
 * @brief  This class implements the accumulator used for Hough Clustering by the Feature Classifier
 * @author Susanne Thierfelder (R12)
 */
class HoughAccumulator
{
  public:

    /** @brief The constructor */
    HoughAccumulator();

    /** @brief The destructor */
    ~HoughAccumulator();

    /** @brief Increment accumulator with given indices */
    bool incrAccumulatorValue(int scaleIndex, int orientationIndex, int xIndex, int yIndex, KeyPointMatch match);

    /** @brief Get accumulator value with given indices */
    bool getAccumulatorValue(int scaleIndex, int orientationIndex, int xIndex, int yIndex, unsigned int& value);

    /** @brief Reset accumulator entries */
    void resetAccumulator();

    /** @brief Cluster accumulator */
    std::vector< std::list< KeyPointMatch > > getClusteredMatches();

    /** @brief Cluster accumulator by searching a maximum */
    std::vector< std::list< KeyPointMatch > > getMaximumMatches();

    /** @brief Get histogram image */
    void getImage( puma2::ColorImageRGB8& target );

    /** @brief Get variance of accumulator */
    float getVariance();

    std::string getLog(){return m_Log.str();}

  private:

    //Sort KeyPointMatch-List in descending order
    struct compareMatchList
    {
      bool operator()(const std::list< KeyPointMatch>& a, const std::list< KeyPointMatch>& b )
      {
        return a.size() > b.size();
      }
    };

    unsigned int getIndex(int scaleIndex, int orientationIndex, int xIndex, int yIndex);
    bool verifyAccumulatorIndex(int scale, int orientation, int xLocation, int yLocation);
    unsigned int getMaxAccumulatorValue();

    int m_ScaleBins;
    int m_OrientationBins;
    int m_XLocationBins;
    int m_YLocationBins;

    std::list< KeyPointMatch >* m_AccumulatorArray;

    unsigned int m_AccumulatorSize;

    std::ostringstream m_Log;
};

#endif
