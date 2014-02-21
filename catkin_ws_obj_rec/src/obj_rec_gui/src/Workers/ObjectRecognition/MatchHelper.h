/*******************************************************************************
 *  MatchHelper.h
 *
 *  (C) 2007 AG Aktives Sehen <agas@uni-koblenz.de>
 *           Universitaet Koblenz-Landau
 *******************************************************************************/

#ifndef MatchHelper_H
#define MatchHelper_H

#include <vector>
#include <list>

#include "../../Workers/KeyPointExtraction/KeyPoint.h"
#include "../../Workers/KeyPointExtraction/KeyPointMatch.h"

/**
 * @class  MatchHelper
 * @brief  Add description here
 * @author Add name here
 */
class MatchHelper
{
  public:

    /** @brief The constructor */
    MatchHelper();

    /** @brief The destructor */
    ~MatchHelper();

    static void calcScaleQuotients( std::vector< KeyPoint >* keyPoints1, std::vector< KeyPoint >* keyPoints2, std::list< KeyPointMatch >& matches );

    static void calcTurnAngles( std::vector< KeyPoint >* keyPoints1, std::vector< KeyPoint >* keyPoints2, std::list< KeyPointMatch >& matches );

  private:

};

#endif
