/*******************************************************************************
 *  CvFundamentalMat.cpp
 *
 *  (C) 2007 AG Aktives Sehen <agas@uni-koblenz.de>
 *           Universitaet Koblenz-Landau
 *******************************************************************************/

#include "CvFundamentalMat.h"

#include "Architecture/Config/Config.h"

#include <math.h>

#define THIS CvFundamentalMat

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

bool THIS::computeFundamentalMat()
{
  double fundamentalMat[9];

  memset ( fundamentalMat, 0, 9*sizeof ( double ) );
  m_FundMatCv = cvMat ( 3, 3, CV_64F, fundamentalMat );

  std::vector<CvPoint2D32f> points1Cv, points2Cv;


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

  m_Points1CvMat = cvMat ( 1, numMatches, CV_32FC2, &points1Cv[0] );
  m_Points2CvMat = cvMat ( 1, numMatches, CV_32FC2, &points2Cv[0] );

  int method = 0;
  switch (Config::getInstance()->getInt( "ObjectRecognition.FundamentalMat.iMethod" ))
  {
    case 0 :
      method = CV_FM_7POINT;
      break;
    case 1 :
      method = CV_FM_8POINT;
      break;
    case 2 :
      method = CV_FM_RANSAC;
      break;
    case 3:
      method = CV_FM_LMEDS;
      break;
    default:
      ROS_ERROR_STREAM("Undefined methode to find fundamental matrix");
      break;
  }
  CvMat* status;
  status = cvCreateMat(1,numMatches,CV_8UC1);


//int cvFindFundamentalMat(const CvMat* points1, const CvMat* points2, CvMat* fundamental_matrix, int method=CV_FM_RANSAC, double param1=1., double param2=0.99, CvMat* status=NULL)
  //returns the number of fundamental matrices found (1 or 3) and 0, if no matrix is found.
  m_Success = cvFindFundamentalMat( &m_Points2CvMat, &m_Points1CvMat, &m_FundMatCv, method, m_MaxReprojectionError,0.99,status);

  ROS_INFO_STREAM("Status = " << m_Success );

  return m_Success > 0 ? true : false;
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

//  void cvComputeCorrespondEpilines(const CvMat* points, int which_image, const CvMat* fundamental_matrix, CvMat* correspondent_lines)
//which image?
  //Tranform object points to scene by fundamental mat

  double correspondent_lines[3];

  memset ( correspondent_lines, 0, 3*sizeof ( double ) );

  CvMat correspondent_linesCV;
  correspondent_linesCV= cvMat(3,m_Matches.size(),CV_32F,correspondent_lines);

  cvComputeCorrespondEpilines(&m_Points2CvMat, 1, &m_FundMatCv, &correspondent_linesCV);

  //Compare line distance to point distance

  currentMatch = m_Matches.begin();
  int i = 0;
  while ( currentMatch != m_Matches.end() )
  {
//    Point2D pos1 = ( *m_KeyPoints1 ) [ currentMatch->index1 ].position();
//    float scale = ( *m_KeyPoints1 ) [ currentMatch->index1 ].scale;
//    if ( pos1.distance( projPoints[i] ) > m_MaxReprojectionError*scale )
//     int x = correspondent_lines[i].x;
//     int y = correspondent_lines[i].y;
//    if()
//    {
//      currentMatch = m_Matches.erase( currentMatch );
//    }
//    else
//    {
//      currentMatch++;
//    }
    i++;
  }
}


#undef THIS
