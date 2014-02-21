/*******************************************************************************
 *  NNRMatcher.cpp
 *
 *  (C) 2007 AG Aktives Sehen <agas@uni-koblenz.de>
 *           Universitaet Koblenz-Landau
 *
 *  Additional information:
 *  $Id: $
 *******************************************************************************/

#include "NNRMatcher.h"

#include "Workers/Math/vec2.h"

#include <assert.h>
#include <map>
#include <list>
#include <math.h>

#define THIS NNRMatcher

THIS::THIS ( std::vector< KeyPoint >* keyPointsA, std::vector< KeyPoint >* keyPointsB )
{
    m_KeyPointsA = keyPointsA;
    m_KeyPointsB = keyPointsB;
    m_Log << "NNRMatcher created\n";
    m_Log << "Number of keypoints (scenePoints/objectImagePoints): " << m_KeyPointsA->size() << " / " << m_KeyPointsB->size() << std::endl;
}

THIS::~THIS()
{
}

void THIS::match ( float maxDistRatio )
{
    if ( ( m_KeyPointsA->size() ==0 ) || ( m_KeyPointsB->size() ==0 ) || ( m_Matches.size() !=0 ) ) { return; }

    //int startTime = Clock::getInstance()->getTimestamp(); // TODO

    int size1=m_KeyPointsA->size();
    int size2=m_KeyPointsB->size();

    float maxDistRatioSquared=maxDistRatio*maxDistRatio;

    std::vector< unsigned > indPos;
    std::vector< unsigned > indNeg;
    indPos.reserve( m_KeyPointsB->size() );
    indNeg.reserve( m_KeyPointsB->size() );

    //sort keypoints2 by their sign
    for ( int index2=0; index2 < size2; index2++ )
    {
      if ( (*m_KeyPointsB)[index2].sign > 0 )
      {
        indPos.push_back( index2 );
      }
      else
      {
        indNeg.push_back( index2 );
      }
    }

    for ( int index1=0; index1 < size1; index1++ )
    {
        //index in second keypoint list corresponding to minimal distance
        int minIndexB=-1;
        //minimal distance found
        double minDist=1e10;
        //second-minimal distance
        double secondMinDist=1e10;

        //select list of indices of keypoints with positive or negative sign, according to currently matche keypoint
        std::vector< unsigned >& indices2 = (*m_KeyPointsA)[index1].sign > 0 ? indPos : indNeg;

        for ( unsigned k=0; k < indices2.size(); k++ )
        {
          int index2 = indices2[ k ];
          double distance = (*m_KeyPointsA)[index1].squaredDistance ( (*m_KeyPointsB)[index2] );//, secondMinDist );
            //new minimum found:
            if ( distance < minDist )
            {
                secondMinDist=minDist;
                minDist=distance;
                minIndexB=index2;
            }
            else
            if ( distance < secondMinDist )
            {
                secondMinDist=distance;
            }
        }
        if ( ( minIndexB != -1 ) && ( minDist/secondMinDist < maxDistRatioSquared ) )
        {
            KeyPointMatch match={ index1, minIndexB, minDist, 0, 0 };
            m_Matches.push_back ( match );
            m_Log << index1 << "->" << minIndexB << " (d" << minDist << "/r" << minDist/secondMinDist << ")  ";
        }

    }
    // m_Log << "\n--- " << m_Matches.size() << " keypoints matched in first phase in " << ( Clock::getInstance()->getTimestamp() - startTime ) << "ms\n"; // TODO

    eliminateMultipleMatches();

}


void THIS::eliminateMultipleMatches()
{
    //It is possible that more than one ipoint in First has been matched to
    //the same ipoint in Second, in this case eliminate all but the closest one

  // int startTime = Clock::getInstance()->getTimestamp(); // TODO

    //maps keypoints in Second to their closest match result
    //first: index in m_KeyPointsB
    //second: iterator in m_Matches
    std::map< unsigned, MatchElem > bestMatch;

    m_Log << "deleting ";

    MatchElem currentMatch=m_Matches.begin();
    while ( currentMatch != m_Matches.end() )
    {
        unsigned index2=currentMatch->index2;

        //check if a match with this keypoint in Second was found before
        std::map< unsigned, MatchElem >::iterator previous=bestMatch.find ( index2 );

        //this is the first match found which maps to current second index
        if ( previous == bestMatch.end() )
        {
            bestMatch[ index2 ] = currentMatch;
            currentMatch++;
            continue;
        }

        MatchElem previousMatch = previous->second;
        //a match mapping to this index in second has been found previously, and had a higher distance
        //so delete the previously found match
        if ( currentMatch->distance < previousMatch->distance )
        {
            m_Log << previousMatch->index1 << "->" << previousMatch->index2;
            m_Log << " (better:" << currentMatch->index1 << "->" << currentMatch->index2 << ")  ";
            m_Matches.erase ( previousMatch );
            bestMatch[ index2 ] = currentMatch;
            currentMatch++;
            continue;
        }
        //otherwise, the previously found best match is better than current,
        //so delete current
        m_Log << currentMatch->index1 << "->" << currentMatch->index2;
        m_Log << " (better:" << previousMatch->index1 << "->" << previousMatch->index2 << ")  ";
        currentMatch=m_Matches.erase ( currentMatch );
    }
    // m_Log << "\n--- " << m_Matches.size() << " remaining after multiple match elimination in " << ( Clock::getInstance()->getTimestamp() - startTime ) << "ms\n"; // TODO

}

std::string THIS::getLog()
{
    return m_Log.str();
}

#undef THIS
