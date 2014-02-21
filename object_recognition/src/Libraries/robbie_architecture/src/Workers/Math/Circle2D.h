/*******************************************************************************
 *  Circle2D.h
 *
 *  (C) 2008 AG Aktives Sehen <agas@uni-koblenz.de>
 *           Universitaet Koblenz-Landau
 *
 * $Id :$
 *******************************************************************************/

#ifndef Circle2D_H
#define Circle2D_H

#include "Point2D.h"
#include <vector>

/**
 * @class Circle2D
 * @author David Gossow
 */
class Circle2D {

  public:

    Circle2D();

    /** Creates a new 2D Circle given by center and radius */
    Circle2D( double x, double y, double radius );
    Circle2D( Point2D center, double radius );

    /**
     * Destructor, does nothing.
     */
    ~Circle2D();

    double x() const;
    double y() const;
    double radius() const;
    Point2D center() const;

    void setX(double x);
    void setY(double y);
    void setCenter( Point2D center );
    void setRadius( double radius );

    std::vector<Point2D> vertices( int steps=40 );

  protected:

    Point2D m_Center;
    double m_Radius;
};

#endif
