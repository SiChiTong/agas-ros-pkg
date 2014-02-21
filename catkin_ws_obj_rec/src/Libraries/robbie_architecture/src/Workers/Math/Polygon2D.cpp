/*******************************************************************************
 *  Polygon2D.cpp
 *
 *  (C) 2008 AG Aktives Sehen <agas@uni-koblenz.de>
 *           Universitaet Koblenz-Landau
 *
 *  Additional information:
 *  $Id: Polygon2D.cpp 44313 2011-04-06 22:46:28Z agas $
 *******************************************************************************/

#include <stdlib.h>
#include "Polygon2D.h"
#include "Point2D.h"
#include "Line2D.h"


#define THIS Polygon2D

using namespace std;

vector<Line2D> THIS::getLines() const
{
  vector<Line2D> lines;
  vector<Point2D>::const_iterator pointIt = m_Points.begin() +1;
  while ( pointIt != m_Points.end() )
  {
    lines.push_back ( Line2D ( * ( pointIt-1 ), *pointIt ) );
    pointIt++;
  }
  lines.push_back ( Line2D ( * ( m_Points.end()-1 ), *m_Points.begin() ) );

  return lines;
}

void THIS::clipLines ( std::vector<Line2D>& linesToClip ) const
{
  // for each line to clip
  vector<Line2D>::iterator linesIt = linesToClip.begin();
  while ( linesIt != linesToClip.end() )
  {
    if ( !clipLine ( *linesIt ) )
    {
      // line lies completely outside of the polygon
      linesToClip.erase ( linesIt );
      continue;
    }
    linesIt++;
  }
}

bool THIS::clipLine ( Line2D& lineToClip ) const
{
  vector<Line2D> polygonLines = getLines();
  CVec2 v = lineToClip.vec();

  float tIn = 0.0;
  float tOut = 1.0;

  // for each line of the polygon
  vector<Line2D>::const_iterator polygonIt = polygonLines.begin();
  while ( polygonIt != polygonLines.end() )
  {
    CVec2 n = ( *polygonIt ).getNormal();
//     cout << "polLine "<< ( *polygonIt ).toString() << endl;
//     cout << "n "<< n.toString() << endl;

    CVec2 w0 = lineToClip.start()- ( *polygonIt ).start();
    CVec2 w1 = lineToClip.end()- ( *polygonIt ).start();

//     cout << "w0 " <<w0.toString() << endl;
//     cout << "w1 " <<w1.toString() << endl;

    float c0 = w0.dot ( n );
    float c1 = w1.dot ( n );

    // test if line lies completely inside
    if ( c0 < 0 && c1 < 0 )
    {
      // do nothing;
      polygonIt++;
      continue;
    }
    // test if line lies completely outside
    else if ( c0 >= 0 && c1 >= 0 )
    {
/*      cout << "return 1" << endl;*/
      return false;
    }
    // clip lines
    else
    {
      float denominator = v.dot ( n );

      if ( denominator == 0 )
      {
        // impossible to happen
        polygonIt++;
        continue;
      }

      float t = -c0/denominator;

      // test for "in" or "out" point
      if ( denominator < 0 && t > tIn )
      {
        // intersection point is an "in point"
        tIn = t;
      }
      else if ( denominator > 0 && t < tOut )
      {
        // intersection point is an "out point"
        tOut = t;
      }
    }
    polygonIt++;
  }

  if ( tIn > tOut ) {
/*    cout << "return 2" << endl;*/
    return false;
  }

  lineToClip.setEnd ( lineToClip.start() + tOut * v );
  lineToClip.setStart ( lineToClip.start() + tIn * v );

//   cout << "return true" << endl;

  return true;
}

#undef THIS
