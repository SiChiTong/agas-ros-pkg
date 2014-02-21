/*******************************************************************************
 *  ImagePropertiesCV.cpp
 *
 *  (C) 2007 AG Aktives Sehen <agas@uni-koblenz.de>
 *           Universitaet Koblenz-Landau
 *
 *  Additional information:
 *  $Id: $
 *******************************************************************************/

#include "ImagePropertiesCV.h"
#include "../KeyPointExtraction/DefaultExtractor.h"
#include "../KeyPointExtraction/KeyPointHelper.h"

#include "Architecture/Config/Config.h"

#define THIS ImagePropertiesCV

THIS::THIS()
{
  clear();
}

THIS::THIS ( std::string name, cv::Mat* imageY, cv::Mat* imageUV, ImageMaskCV* imageMask )
{
  clear();

  if ( !imageY || !imageUV )
  {
    ROS_ERROR_STREAM( "Received 0-pointer as source image." );
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

  m_ImageY = new cv::Mat( * ( other.m_ImageY ) );
  m_ImageUV = new cv::Mat( * ( other.m_ImageUV ) );

  if ( other.m_ImageMask )
  {
    m_ImageMask = new ImageMaskCV( *(other.m_ImageMask) );
  }

  if ( other.m_ImageMaskWithBorder )
  {
    m_ImageMaskWithBorder = new ImageMaskCV( *(other.m_ImageMaskWithBorder) );
  }

  if ( other.m_MaskedImageY )
  {
    m_MaskedImageY = new cv::Mat( * ( other.m_MaskedImageY ) );
  }

  if ( other.m_MaskedImageUV )
  {
    m_MaskedImageUV = new cv::Mat( * ( other.m_MaskedImageUV ) );
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
}


void THIS::applyMask()
{
  if ( m_MaskedImageY )
  {
    return;
  }

  if ( m_ImageMask )
  {
    m_Border = 0;

    int width = m_ImageY->cols;
    int height = m_ImageY->rows;
    int newWidth = width + 2*m_Border;
    int newHeight = height + 2*m_Border;

    if(m_MaskedImageY)
        delete m_MaskedImageY;

    m_MaskedImageY = new cv::Mat( newHeight, newWidth, CV_8UC1);

    if(m_MaskedImageUV)
        delete m_MaskedImageUV;

    m_MaskedImageUV = new cv::Mat( newHeight, newWidth, CV_8UC3);

    for ( int y=0; y<height; y++ )
    {
      for ( int x=0; x<width; x++ )
      {
          m_MaskedImageY->at<unsigned char>(m_Border+y, m_Border+x) = m_ImageY->at<unsigned char>(y,x);
      }
    }

    for ( int y=0; y<height; y++ )
    {
      for ( int x=0; x<width; x++ )
      {
          m_MaskedImageUV->at<cv::Vec3b>(m_Border+y, m_Border+x) = m_ImageUV->at<cv::Vec3b>(y,x);
      }
    }

    m_ImageMaskWithBorder = new ImageMaskCV( newWidth, newHeight );
    m_ImageMaskWithBorder->fill( ImageMaskCV::MASKED );
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

    m_Center = m_ImageMaskWithBorder->getGravCenter();
  }
  else
  {
    m_MaskedImageY = new cv::Mat( *m_ImageY );
    m_MaskedImageUV = new cv::Mat( *m_ImageUV );

    m_Center = Point2D( m_ImageY->cols/2, m_ImageY->rows/2 );
  }
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
    ImageMaskCV maskOutline = *m_ImageMask;
    maskOutline.dilate(1);
    maskOutline.findBorders();
    unsigned char *m_BorderData=maskOutline.getData();
    unsigned rowStart=0;
    unsigned w=maskOutline.getWidth();
    unsigned h=maskOutline.getHeight();

    float minDist = ( m_ImageY->cols + m_ImageY->rows ) * 0.002;

    m_Outline->reserve ( 2000 );

    for ( unsigned row=0; row<h; row++ )
    {
      rowStart=row*w;
      for ( unsigned i=rowStart; i<rowStart+w; i++ )
      {
        if ( m_BorderData[i] == ImageMaskCV::MASKED )
        {
          //cout << "found m_Borderpixel at " << i-rowStart << "," << row << endl;
          //found one m_Border pixel, trace this m_Border
          unsigned x=i-rowStart;
          unsigned y=row;

          m_BorderData[i] = ImageMaskCV::VISIBLE;
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
              if ( m_BorderData[ ( tmp_y ) *w + tmp_x ] == ImageMaskCV::MASKED )
              {
                x=tmp_x;
                y=tmp_y;
                //cout << tmp_x << "," << tmp_y << "  ";
                m_BorderData[ y*w + x ] = ImageMaskCV::VISIBLE;
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

