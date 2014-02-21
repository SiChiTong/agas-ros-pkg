/*******************************************************************************
 *  ImageProperties.cpp
 *
 *  (C) 2007 AG Aktives Sehen <agas@uni-koblenz.de>
 *           Universitaet Koblenz-Landau
 *
 *  Additional information:
 *  $Id: $
 *******************************************************************************/

#include "ImageProperties.h"
#include "../../Workers/KeyPointExtraction/DefaultExtractor.h"
#include "../../Workers/KeyPointExtraction/KeyPointHelper.h"

#include "../../Workers/Puma2/ColorImageRGB8.h"
#include "../../Workers/Puma2/Y8UV8ToRGB8Operator.h"

//#include "Architecture/Tracer/Tracer.h" // TODO
//#include "Architecture/Config/Config.h" // TODO

#define THIS ImageProperties

using namespace puma2;

THIS::THIS()
{
  clear();
}

THIS::THIS ( std::string name, puma2::GrayLevelImage8* imageY, puma2::ColorImageUV8* imageUV, ImageMask* imageMask )
{
  clear();

  if ( !imageY || !imageUV )
  {
    //TRACE_ERROR( "Received 0-pointer as source image." ); // TODO use ros
    return;
  }

  m_Name = name;
  m_ImageY = imageY;
  m_ImageUV = imageUV;
  m_ImageMask = imageMask;

  applyMask();
}

void THIS::clear()
{
  m_ImageY = 0;
  m_ImageUV = 0;
  m_MaskedImageY = 0;
  m_MaskedImageUV = 0;
  m_Histogram = 0;
  m_KeyPoints = 0;
  m_Outline = 0;
  m_ImageMask = 0;
  m_ImageMaskWithBorder = 0;
}


THIS::~THIS()
{
  deleteAll();
}

void THIS::deleteAll()
{
  delete m_ImageY;
  delete m_ImageUV;
  delete m_ImageMask;
  delete m_ImageMaskWithBorder;
  delete m_MaskedImageY;
  delete m_MaskedImageUV;
  delete m_Histogram;
  delete m_KeyPoints;
  delete m_Outline;}

THIS& THIS::operator= ( const THIS& other )
{
  deleteAll();
  clear();

  m_Name = other.m_Name;
  m_Center = other.m_Center;
  m_Border = other.m_Border;
  m_Center = other.m_Center;

  m_ImageY = new GrayLevelImage8( * ( other.m_ImageY ) );
  m_ImageUV = new ColorImageUV8( * ( other.m_ImageUV ) );

  if ( other.m_ImageMask )
  {
    m_ImageMask = new ImageMask( *(other.m_ImageMask) );
  }

  if ( other.m_ImageMaskWithBorder )
  {
    m_ImageMaskWithBorder = new ImageMask( *(other.m_ImageMaskWithBorder) );
  }

  if ( other.m_MaskedImageY )
  {
    m_MaskedImageY = new GrayLevelImage8( * ( other.m_MaskedImageY ) );
  }

  if ( other.m_MaskedImageUV )
  {
    m_MaskedImageUV = new ColorImageUV8( * ( other.m_MaskedImageUV ) );
  }

  if ( other.m_Histogram )
  {
    m_Histogram = new HistogramUV ( * ( other.m_Histogram ) );
  }

  if ( other.m_KeyPoints )
  {
    m_KeyPoints=new std::vector< KeyPoint > ( * ( other.m_KeyPoints ) );
  }

  if ( other.m_Outline )
  {
    m_Outline=new std::vector<Point2D>( * ( other.m_Outline ) );
  }

  return *this;
}


THIS::THIS ( const THIS& other )
{
  clear();
  operator= ( other );
}


void THIS::calculateProperties()
{
  applyMask();
  traceOutline();
  extractKeyPoints();
  calcHistogram();
}


void THIS::applyMask()
{
  if ( m_MaskedImageY )
  {
    return;
  }

  if ( m_ImageMask )
  {
//    float borderSize = Config::getFloat( "ObjectRecognition.fObjectImageBorder" );
    m_Border = 0;//= ( m_ImageY->getWidth() + m_ImageY->getHeight() ) / 2 * borderSize;

    int width = m_ImageY->getWidth();
    int height = m_ImageY->getHeight();
    int newWidth = width + 2*m_Border;
    int newHeight = height + 2*m_Border;

    if(m_MaskedImageY)
        delete m_MaskedImageY;

    m_MaskedImageY = new GrayLevelImage8( newWidth, newHeight );

    if(m_MaskedImageUV)
        delete m_MaskedImageUV;

    m_MaskedImageUV = new ColorImageUV8( newWidth, newHeight );

//     for ( int y=0; y<newHeight; y++ )
//     {
//       for ( int x=0; x<newWidth; x++ )
//       {
//         (*m_MaskedImageY)[y][x] = 0;
//         (*m_MaskedImageUV)[y][x][0] = 127;
//         (*m_MaskedImageUV)[y][x][1] = 127;
//       }
//     }

    for ( int y=0; y<height; y++ )
    {
      for ( int x=0; x<width; x++ )
      {
        (*m_MaskedImageY)[m_Border+y][m_Border+x] = (*m_ImageY)[y][x];
      }
    }

    for ( int y=0; y<height; y++ )
    {
      for ( int x=0; x<width; x++ )
      {
        (*m_MaskedImageUV)[m_Border+y][m_Border+x] = (*m_ImageUV)[y][x];
      }
    }

    m_ImageMaskWithBorder = new ImageMask( newWidth, newHeight );
    m_ImageMaskWithBorder->fill( ImageMask::MASKED );
    unsigned char* maskData = m_ImageMask->getData();
    unsigned char* maskDataNew = m_ImageMaskWithBorder->getData();

    int i=0;
    for ( int y=m_Border; y<m_Border+height; y++ )
    {
      for ( int x=m_Border; x<m_Border+width; x++ )
      {
        maskDataNew[x+newWidth*y] = maskData[i];
        i++;
      }
    }

//     m_ImageMaskWithBorder->apply( *m_MaskedImageY, 0 );
//     m_ImageMaskWithBorder->apply( *m_MaskedImageUV, 127, 127 );
    m_Center = m_ImageMaskWithBorder->getGravCenter();
  }
  else
  {
    m_MaskedImageY = new GrayLevelImage8( *m_ImageY );
    m_MaskedImageUV = new ColorImageUV8( *m_ImageUV );
    m_Center = Point2D( m_ImageY->getWidth()/2, m_ImageY->getHeight()/2 );
  }
}


void THIS::calcHistogram()
{
  if ( m_Histogram )
  {
    return;
  }

  applyMask();

  unsigned binSize = 10; // TODO Config::getInt( "ObjectRecognition.Histogram.iBinSize" );

  m_Histogram=new HistogramUV ( binSize );
  m_Histogram->addImage ( *m_ImageUV, *m_ImageY );
}


void THIS::extractKeyPoints()
{
  if ( m_KeyPoints )
  {
    return;
  }

  applyMask();

  KeyPointExtractor* extractor = DefaultExtractor::createInstance();

  m_KeyPoints = new std::vector<KeyPoint>();
  extractor->setImage( *m_MaskedImageY );
  extractor->getKeyPoints( *m_KeyPoints );

  if ( m_ImageMask )
  {
    KeyPointHelper::maskFilter( *m_KeyPoints, *m_KeyPoints, *m_ImageMaskWithBorder );
  }
/*
  for ( int i=0; (i<10 && i<m_KeyPoints->size()); i++ )
  {
    for ( int j=0; j<(*m_KeyPoints)[i].featureVector.size(); j++ )
    {
      cout << (*m_KeyPoints)[i].featureVector[j] << " ";
    }
    cout << endl;
  }*/

  delete extractor;
}

void THIS::traceOutline()
{
  if ( m_Outline )
  {
    return;
  }
  m_Outline = new std::vector<Point2D>();
  //trace object m_Borders (find longest m_Border)
  if ( m_ImageMask )
  {
    ImageMask maskOutline = *m_ImageMask;
    maskOutline.dilate(1);
    maskOutline.findBorders();
    unsigned char *m_BorderData=maskOutline.getData();
    unsigned rowStart=0;
    unsigned w=maskOutline.getWidth();
    unsigned h=maskOutline.getHeight();

    float minDist = ( m_ImageY->getWidth() + m_ImageY->getHeight() ) * 0.002;

    m_Outline->reserve ( 2000 );

    for ( unsigned row=0; row<h; row++ )
    {
      rowStart=row*w;
      for ( unsigned i=rowStart; i<rowStart+w; i++ )
      {
        if ( m_BorderData[i] == ImageMask::MASKED )
        {
          //cout << "found m_Borderpixel at " << i-rowStart << "," << row << endl;
          //found one m_Border pixel, trace this m_Border
          unsigned x=i-rowStart;
          unsigned y=row;

          m_BorderData[i] = ImageMask::VISIBLE;
          m_Outline->push_back ( Point2D ( x+m_Border,y+m_Border ) );

          bool pixelFound=true;
          while ( pixelFound )
          {
            //cout << x << "," << y << "  ";
            if ( Point2D ( x+m_Border,y+m_Border ).distance( m_Outline->back() ) > minDist ) {
              m_Outline->push_back ( Point2D ( x+m_Border,y+m_Border ) );
            }

            //search neighbours
            //search order (inverted because stack pushing inverts the result):
            //   538
            //   1 2
            //   746
            int xpos[8]={-1,+1, 0, 0,-1,+1,-1,+1};
            int ypos[8]={ 0, 0,-1,+1,-1,+1,+1,-1};

            pixelFound=false;
            for ( int j=7; j>=0; j-- )
            {
              int tmp_x=x+xpos[j];
              int tmp_y=y+ypos[j];
              if ( ( tmp_x < 0 ) || ( tmp_y < 0 ) || ( tmp_x >= (int)m_ImageMask->getWidth() ) ||  ( tmp_y >= (int)m_ImageMask->getHeight() ) ) {
                continue;
              }
              if ( m_BorderData[ ( tmp_y ) *w + tmp_x ] == ImageMask::MASKED )
              {
                x=tmp_x;
                y=tmp_y;
                //cout << tmp_x << "," << tmp_y << "  ";
                m_BorderData[ y*w + x ] = ImageMask::VISIBLE;
                pixelFound=true;
                break;
              }
            }
            //cout << " - ";
          }
          m_Outline->push_back ( Point2D ( x+m_Border,y+m_Border ) );
          m_Outline->push_back( Point2D::invalidPoint() );
        }
      }
    }
  }
}

std::vector<Point2D> THIS::getBoundingBox() const
{
  std::vector<Point2D> bBox;
  bBox.reserve( 5 );
  bBox.push_back( Point2D( 0, 0 ) );
  bBox.push_back( Point2D( m_ImageMask->getWidth()-1+2*m_Border, 0 ) );
  bBox.push_back( Point2D( m_ImageMask->getWidth()-1+2*m_Border, m_ImageMask->getHeight()-1+2*m_Border ) );
  bBox.push_back( Point2D( 0, m_ImageMask->getHeight()-1+2*m_Border ) );
  bBox.push_back( Point2D( 0, 0 ) );
  return bBox;
}


#undef THIS

