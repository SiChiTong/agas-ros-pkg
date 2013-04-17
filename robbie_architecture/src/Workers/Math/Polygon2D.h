/*******************************************************************************
 *  Polygon2D.h
 *
 *  (C) 2008 AG Aktives Sehen <agas@uni-koblenz.de>
 *           Universitaet Koblenz-Landau
 *
 * $Id: Polygon2D.h 44313 2011-04-06 22:46:28Z agas $
 *******************************************************************************/

#ifndef POLYGON2D_H
#define POLYGON2D_H

#include <vector>

class Polygon2D;
class Point2D;
class Line2D;

/**
 * @class Polygon2D
 *
 * @author Susanne Maur
 *
 */
class Polygon2D
{
  public:
    
    inline Polygon2D() {}

    /**
     * Creates a new polygon
     * The points must be given counterclockwise.
     */
    inline Polygon2D ( std::vector<Point2D>& points )
    {
      m_Points = points;
    }
    
    inline ~Polygon2D() {}

    inline std::vector<Point2D> getPoints() const{
      return m_Points;
    }

    /**
     * Returns the line representation of the polygon.
     * @return line representation of the polygon.
     */
    std::vector<Line2D> getLines() const;

    /**
     * Clips lines to this polygon. Not tested!!!
     * Implementation of Cyrus-Beck.
     * The polygon must be difined counterclockwise.
     * @param linesToClip The lines to clip.
     */
    void clipLines ( std::vector<Line2D>& linesToClip ) const;

    /**
     * Clips a line to this polygon.
     * Implementation of Cyrus-Beck.
     * The polygon must be difined counterclockwise.
     * @param lineToClip The lines to clip.
     * @return True if it was possible to clip the line, false if the line lies completely outside.
     */
    bool clipLine ( Line2D& lineToClip ) const;

  private:
    
    std::vector<Point2D> m_Points;

};

#endif

