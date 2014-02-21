/*******************************************************************************
 *  MatchHelper.cpp
 *
 *  (C) 2007 AG Aktives Sehen <agas@uni-koblenz.de>
 *           Universitaet Koblenz-Landau
 *******************************************************************************/

#include "MatchHelper.h"

#include "Workers/Math/Math.h"

#define THIS MatchHelper

THIS::THIS()
{
}

THIS::~THIS()
{
}

void THIS::calcScaleQuotients( std::vector< KeyPoint >* keyPoints1, std::vector< KeyPoint >* keyPoints2, std::list< KeyPointMatch >& matches )
{
  std::list< KeyPointMatch >::iterator currentMatch = matches.begin();
  while ( currentMatch != matches.end() )
  {
    unsigned index1 = currentMatch->index1;
    unsigned index2 = currentMatch->index2;
    currentMatch->scaleQuotient = (*keyPoints1)[ index1 ].scale/ (*keyPoints2)[ index2 ].scale;
    currentMatch++;
  }
}


void THIS::calcTurnAngles( std::vector< KeyPoint >* keyPoints1, std::vector< KeyPoint >* keyPoints2, std::list< KeyPointMatch >& matches )
{
  std::list< KeyPointMatch >::iterator currentMatch = matches.begin();
  while ( currentMatch != matches.end() )
  {
    unsigned index1 = currentMatch->index1;
    unsigned index2 = currentMatch->index2;
    currentMatch->turnAngle = Math::minTurnAngle ( (*keyPoints1)[ index1 ].orientation, (*keyPoints2)[ index2 ].orientation );
    currentMatch++;
  }
}

#undef THIS
