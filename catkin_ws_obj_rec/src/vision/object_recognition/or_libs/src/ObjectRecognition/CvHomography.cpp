/*******************************************************************************
 *  CvHomography.cpp
 *
 *  (C) 2007 AG Aktives Sehen <agas@uni-koblenz.de>
 *           Universitaet Koblenz-Landau
 *******************************************************************************/

#include "CvHomography.h"

#include "Architecture/Config/Config.h"

#include <math.h>
#include <cv.h>

#define THIS CvHomography

using namespace std;

THIS::THIS ( vector< KeyPoint >* keyPoints1, vector< KeyPoint >* keyPoints2, std::list< KeyPointMatch >& matches )
{
  m_KeyPoints1 = keyPoints1;
  m_KeyPoints2 = keyPoints2;
  m_Matches = matches;
  m_Success = false;
  m_MaxReprojectionError = Config::getFloat( "ObjectRecognition.Ransac.fMaxReprojectionError" );
}

THIS::~THIS()
{
}

bool THIS::computeHomography()
{
  double homMat[9];
  CvMat homMatCv;

  memset ( homMat, 0, 9*sizeof ( double ) );
  homMatCv = cvMat ( 3, 3, CV_64F, homMat );

  std::vector<CvPoint2D32f> points1Cv, points2Cv;
  CvMat points1CvMat, points2CvMat;

  int numMatches = m_Matches.size();

  if ( numMatches < 4 )
  {
    return false;
  }

  // Set vectors to correct size
  points1Cv.resize ( numMatches );
  points2Cv.resize ( numMatches );

  // Copy Ipoints from match vector into cvPoint vectors
  std::list<KeyPointMatch>::iterator currentMatch = m_Matches.begin();
  int i = 0;
  while ( currentMatch != m_Matches.end() )
  {
    points1Cv[i] = cvPoint2D32f ( ( *m_KeyPoints1 ) [ currentMatch->index1 ].x,
                                  ( *m_KeyPoints1 ) [ currentMatch->index1 ].y );
    points2Cv[i] = cvPoint2D32f ( ( *m_KeyPoints2 ) [ currentMatch->index2 ].x,
                                  ( *m_KeyPoints2 ) [ currentMatch->index2 ].y );
    currentMatch++;
    i++;
  }

  points1CvMat = cvMat ( 1, numMatches, CV_32FC2, &points1Cv[0] );
  points2CvMat = cvMat ( 1, numMatches, CV_32FC2, &points2Cv[0] );

  // Find the homography (transformation) between the two sets of points

  //0 - regular method using all the point pairs
  //CV_RANSAC - RANSAC-based robust method
  //CV_LMEDS - Least-Median robust method

  int method = 0;
  switch (Config::getInstance()->getInt( "ObjectRecognition.Homography.iMethod" ))
  {
    case 0 :
      method = 0;
      break;
    case 1 :
      method = CV_RANSAC;
      break;
    case 2 :
      method = CV_LMEDS;
      break;
    default:
      ROS_ERROR_STREAM("Undefined methode to find homography");
      break;
  }

  m_Success = cvFindHomography( &points2CvMat, &points1CvMat, &homMatCv, method, m_MaxReprojectionError );

//   float n=sqrt(homMat[0]*homMat[0]+homMat[3]*homMat[3]) * sqrt(homMat[1]*homMat[1]+homMat[4]*homMat[4]);
//
//   float det = homMat[0]*homMat[4] - homMat[1]*homMat[3];
//   det /= n;
//   float l = fabs( det );
//
//   if ( l < 0.8 )
//   {
//     m_Success = false;
//   }
//
//   TRACE_INFO( "det: " << det );
//
//   /*
//   float scalar= homMat[0]*homMat[1] + homMat[3]*homMat[4];
//   scalar /= n;
//   */

  m_Homography = Homography( homMat );

  return m_Success;
}

void THIS::eliminateBadMatches()
{
  vector<Point2D> points2;
  vector<Point2D> projPoints;

  points2.reserve( m_Matches.size() );

  std::list<KeyPointMatch>::iterator currentMatch = m_Matches.begin();
  while ( currentMatch != m_Matches.end() )
  {
    Point2D pos2 = ( *m_KeyPoints2 ) [ currentMatch->index2 ].position();
    points2.push_back( pos2 );
    currentMatch++;
  }

  m_Homography.transform( points2, projPoints );

  currentMatch = m_Matches.begin();
  int i = 0;
  while ( currentMatch != m_Matches.end() )
  {
    Point2D pos1 = ( *m_KeyPoints1 ) [ currentMatch->index1 ].position();
    float scale = ( *m_KeyPoints1 ) [ currentMatch->index1 ].scale;
    if ( pos1.distance( projPoints[i] ) > m_MaxReprojectionError*scale )
    {
      currentMatch = m_Matches.erase( currentMatch );
    }
    else
    {
      currentMatch++;
    }
    i++;
  }
}

#undef THIS
