/*******************************************************************************
 *  SimpleHoughClusterer.cpp
 *
 *  (C) 2007 AG Aktives Sehen <agas@uni-koblenz.de>
 *           Universitaet Koblenz-Landau
 *
 *  Additional information:
 *  $Id: $
 *******************************************************************************/

#include "SimpleHoughClusterer.h"

// #include "Architecture/Tracer/Tracer.h" // TODO

#include "Workers/Math/vec2.h"

#include <assert.h>
#include <map>
#include <list>
#include <math.h>


#define THIS SimpleHoughClusterer

THIS::THIS ( std::vector< KeyPoint >* keyPoints1,  std::vector< KeyPoint >* keyPoints2, std::list< KeyPointMatch >& matches )
{
    m_KeyPoints1 = keyPoints1;
    m_KeyPoints2 = keyPoints2;
    m_Matches = matches;
    m_Log << "SimpleHoughClusterer created\n";
    m_Log << "Number of keypoints (scenePoints/objectImagePoints): " << m_KeyPoints1->size() << " / " << m_KeyPoints2->size() << std::endl;
}


THIS::~THIS()
{
}


void THIS::eliminateByOrientation ( )
{
    m_Log << std::endl << "-------- Orientation based elimination -----------" << std::endl;

    int numBins=80;
    int windowSize=numBins/16;

    //Turn Angle histogram
     std::vector<double> hist;
    hist.assign ( numBins, 0 );

    std::list<KeyPointMatch>::iterator currentMatch=m_Matches.begin();
    while ( currentMatch != m_Matches.end() )
    {
        hist[ int ( ( currentMatch->turnAngle+M_PI ) /M_PI/2.0 * numBins ) % numBins ]++;
        currentMatch++;
    }

     std::vector<bool> deleteMap=computeDeleteMap( hist, numBins, windowSize);

    //Delete
    currentMatch=m_Matches.begin();
    while ( currentMatch != m_Matches.end() )
    {
        int index=int ( ( currentMatch->turnAngle+M_PI ) /M_PI/2.0 * numBins ) % numBins;
        //non-max
        if ( deleteMap[ index ] )
        {
            m_Log << "deleting: " << currentMatch->turnAngle/M_PI*180.0 << "deg " << hist[ index ] << "hist";
            currentMatch = m_Matches.erase ( currentMatch );
        }
        else
        {
            currentMatch++;
        }
    }

    m_Log << "\nTurn angles: ";
    currentMatch=m_Matches.begin();
    while ( currentMatch != m_Matches.end() )
    {
        m_Log << currentMatch->turnAngle << " ";
        currentMatch++;
    }
    m_Log << "\n--- " << m_Matches.size() << " remaining after orientation based elimination.\n";
    m_Log << std::endl;
}


void THIS::eliminateByScale ( )
{
    m_Log << std::endl << "-------- Scale based elimination -----------" <<  std::endl;

    int numBins=150;
    int windowSize=numBins/15;
    int maxOctaves=5;

    //scale quotient histogram
    std::vector<double> hist;
    hist.assign ( numBins, 0 );

    std::list<KeyPointMatch>::iterator currentMatch=m_Matches.begin();
    while ( currentMatch != m_Matches.end() )
    {
        int index = ( log2( currentMatch->scaleQuotient ) / (maxOctaves-1) / 2 + 0.5 ) * numBins;
        if ( index >= numBins ) { index=numBins-1; }
        if ( index < 0 ) { index=0; }

        hist[ index ]++;
        currentMatch++;
    }

     std::vector<bool> deleteMap=computeDeleteMap( hist, numBins, windowSize);

    //Delete
    currentMatch=m_Matches.begin();
    while ( currentMatch != m_Matches.end() )
    {
        int index = ( log2( currentMatch->scaleQuotient ) / maxOctaves / 2 + 0.5 ) * numBins;
        if ( index >= numBins ) { index=numBins-1; }
        if ( index < 0 ) { index=0; }

        //non-max
        if ( deleteMap[ index ] )
        {
            m_Log << "deleting: scale quotient=" << currentMatch->scaleQuotient << " log=" << log2(currentMatch->scaleQuotient) << "  histogram entry=" << hist[ index ];
            currentMatch = m_Matches.erase ( currentMatch );
        }
        else
        {
            currentMatch++;
        }
    }

    m_Log << "\nScale quotients: ";
    currentMatch=m_Matches.begin();
    while ( currentMatch != m_Matches.end() )
    {
        m_Log << currentMatch->scaleQuotient << " ";
        currentMatch++;
    }
    m_Log << "\n--- " << m_Matches.size() << " remaining after scale based elimination:\n";
}

void THIS::eliminateByPosition ( float maxDistance )
{
    float turnAngle = getMeanTurnAngle();
    float scaleChange = getMeanScaleQuotient();
    Point2D sourceCenter;
    Point2D targetCenter;

    bool outlierFound = true;

    while ( outlierFound )
    {
      getCenters ( targetCenter, sourceCenter );

      outlierFound = false;

      std::list<KeyPointMatch>::iterator currentMatch=m_Matches.begin();
      while ( currentMatch != m_Matches.end() )
      {
          Point2D position1 = (*m_KeyPoints1)[ currentMatch->index1 ].position();
          Point2D position2 = (*m_KeyPoints2)[ currentMatch->index2 ].position();

          position2 -= sourceCenter.toVector();
          position2.rotate ( turnAngle );
          position2 *= scaleChange;
          position2 += targetCenter.toVector();

          float distance = position1.distance ( position2 );
          float meanScale = ( (*m_KeyPoints1)[ currentMatch->index1 ].scale+ (*m_KeyPoints2)[ currentMatch->index2 ].scale) / 2.0;
          distance /= meanScale;

          if ( distance > maxDistance )
          {
              m_Log << " deleting " << currentMatch->index1 << "->" << currentMatch->index2 << "(" << distance << ")  ";
              currentMatch = m_Matches.erase ( currentMatch );
              outlierFound = true;
          }
          else
          {
              currentMatch++;
          }
      }
    }
    m_Log << "\n--- " << m_Matches.size() << " remaining after position based elimination:\n";
}


std::vector<bool> THIS::computeDeleteMap( std::vector<double> hist, int numBins, int windowSize )
{
  m_Log << "original histogram: ";
  for ( int i=0; i<numBins; i++ )
  {
    m_Log << hist[i] << " ";
  }
  m_Log <<  std::endl;

    //calc sliding window mean
  float windowSum=0;
  std::vector<double> hist2=hist;
  for ( int i=0; i<windowSize; i++ )
  {
        //cout << i << " ";
    windowSum+=hist2[i];
  }
  for ( int i=0; i<numBins; i++ )
  {
    hist[ ( i+ ( windowSize/2 ) ) % numBins ]=windowSum;
    windowSum-=hist2[i];
    windowSum+=hist2[ ( i+windowSize ) % numBins ];
  }

  m_Log << "    mean histogram: ";
  for ( int i=0; i<numBins; i++ )
  {
    m_Log << hist[i] << " ";
  }
  m_Log <<  std::endl;

    //find max
  float max=0;
  for ( int i=0; i<numBins; i++ )
  {
    if ( hist[i] > max ) max=hist[i];
  }
  m_Log << "           maximum: " << max << std::endl;

    //determine what to keep (elements which are near a maximum)
  std::vector<bool> deleteMap;
  deleteMap.assign ( numBins, true );
  for ( int i=0; i<numBins; i++ )
  {
    for ( int j=-windowSize/2; j<=windowSize/2; j++ )
    {
      if ( hist[ ( numBins+i+j ) %numBins] >= max )
      {
        deleteMap[i]=false;
        break;
      }
    }
  }

  m_Log << "            delete: ";
  for ( int i=0; i<numBins; i++ )
  {
    m_Log << deleteMap[i] << " ";
  }
  m_Log <<  std::endl;
  return deleteMap;
}


float THIS::getMeanTurnAngle() const
{
    std::vector<float> turnAngles;
    turnAngles.reserve ( m_Matches.size() );

    std::list< KeyPointMatch >::const_iterator currentMatch=m_Matches.begin();
    while ( currentMatch != m_Matches.end() )
    {
        turnAngles.push_back ( currentMatch->turnAngle );
        currentMatch++;
    }
    float mean=Math::meanAngle ( turnAngles );
    return mean;
}


float THIS::getMeanScaleQuotient() const
{
    std::vector<float> scaleQuotients;
    scaleQuotients.reserve ( m_Matches.size() );

    std::list< KeyPointMatch >::const_iterator currentMatch=m_Matches.begin();
    while ( currentMatch != m_Matches.end() )
    {
        scaleQuotients.push_back ( currentMatch->scaleQuotient );
        currentMatch++;
    }
    float mean=Math::mean ( scaleQuotients );
    return mean;
}


void THIS::getCenters ( Point2D& center1, Point2D& center2 )
{
  double numPoints = double ( m_Matches.size() );

  std::list< KeyPointMatch >::iterator currentMatch=m_Matches.begin();
  while ( currentMatch != m_Matches.end() )
  {
    unsigned index1 = currentMatch->index1;
    unsigned index2 = currentMatch->index2;

    center1 += (*m_KeyPoints1)[index1].position().toVector();
    center2 += (*m_KeyPoints2)[index2].position().toVector();

    currentMatch++;
  }

  center1 = Point2D ( center1.x() / numPoints, center1.y() / numPoints );
  center2 = Point2D ( center2.x() / numPoints, center2.y() / numPoints );
}


std::string THIS::getLog()
{
  return m_Log.str();
}


#undef THIS
