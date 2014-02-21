/*******************************************************************************
 *  VectorObject2D.h
 *
 *  (C) 2007 AG Aktives Sehen <agas@uni-koblenz.de>
 *           Universitaet Koblenz-Landau
 *
 *  Additional information:
 *  $Id: $
 *******************************************************************************/

#ifndef VectorObject2D_H
#define VectorObject2D_H

#include "Workers/Math/Point2D.h"
#include <vector>

//#include "Architecture/Serializer/ExtendedOutStream.h" TODO ged rid of these?
//#include "Architecture/Serializer/ExtendedInStream.h"

/**
 * @class  VectorObject2D
 * @brief  Represents a 2D vector graphic object
 * @author David Gossow (RX)
 */
class VectorObject2D
{

  public:

    enum StyleT {
      Lines,
      Dots
    };

    static const unsigned short ClassVersion;

    VectorObject2D( const std::vector<Point2D> vertices, float r, float g, float b, float lineWidth=1.0, StyleT style=Lines );

    ~VectorObject2D();

    void paintGl();

    inline std::vector<Point2D>& vertices() { return m_Vertices; }
    inline float r() { return m_R; }
    inline float g() { return m_G; }
    inline float b() { return m_B; }
    inline float lineWidth() { return m_LineWidth; }
    inline StyleT style() { return m_Style; }

    inline void setVertices( std::vector<Point2D>& vertices ) { m_Vertices=vertices; }
    inline void setColor( float r, float g, float b ) { m_R=r; m_G=g; m_B=b; }
    inline void setLineWidth( float lineWidth ) { m_LineWidth=lineWidth; }
    inline void setStyle( StyleT style ) { m_Style=style; }

//    /** @brief Serialize to stream */
//    void storer( ExtendedOutStream& extStrm ) const;

//    /** @brief Deserialize from stream */
//    VectorObject2D( ExtendedInStream& extStrm, unsigned short sourceVersion );

  private:

    std::vector<Point2D> m_Vertices;
    float m_R;
    float m_G;
    float m_B;
    float m_LineWidth;
    StyleT m_Style;
};

#endif
