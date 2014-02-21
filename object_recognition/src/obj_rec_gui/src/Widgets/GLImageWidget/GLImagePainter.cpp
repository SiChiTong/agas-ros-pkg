/*******************************************************************************
 *  GLImagePainter.cpp
 *
 *  (C) 2006 AG Aktives Sehen <agas@uni-koblenz.de>
 *           Universitaet Koblenz-Landau
 ******************************************************************************/


// TODO
// #include "Architecture/Config/Config.h"
#include "../../Workers/Puma2/ThermalToColorOperator.h"
#include "../../Workers/Puma2/Y8UV8ToRGB8Operator.h"
//#include "../../Workers/Puma2/ImageWriter.h"  // TODO

#include "GLImagePainter.h"

#include <sstream>
#include <QMouseEvent>

using namespace puma2;

#define THIS GLImagePainter

THIS::THIS()
{
  m_ImageWidth = 1;
  m_ImageHeight = 1;

  setPixelAspectRatio ( 1.0 );

  m_TextureId = -1;
  m_TextureData = 0;
  m_TextureResolution = 0;
  m_TextureFormat = ImageGrabber::GRAY8;
  m_TextureByteSize = 0;
}

THIS::~THIS()
{
  if ( m_TextureData )
  {
    // TRACE_SYSTEMINFO ( "Deleting texture memory" ) // TODO
    delete m_TextureData;
  }
  if ( int ( m_TextureId ) != -1 )
  {
    // TRACE_SYSTEMINFO ( "Deleting OpenGL texture" ) // TODO
    glDeleteTextures ( 1, &m_TextureId );
  }
}

void THIS::paintImage( float alpha )
{

  //TRACE_INFO( "paintImage()" );


  glEnable ( GL_BLEND );
  glBlendFunc ( GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA );

  glColor4f ( 1.0, 1.0, 1.0, alpha );
  //glColor3f ( 1.0, 1.0, 1.0 );

  if ( m_TextureData )
  {

    if ( int ( m_TextureId ) == -1 )
    {
      glGenTextures ( 1, &m_TextureId );
      std::ostringstream stream;
      stream << "Using OpenGL texture # " << m_TextureId;
      //TRACE_INFO ( stream.str() ); // TODO
    }
    else
    {
      glDeleteTextures ( 1, &m_TextureId );
      glGenTextures ( 1, &m_TextureId );
    }

    std::ostringstream stream;
    stream << "Loading texture data to texture ID " << m_TextureId;
    // TRACE_SYSTEMINFO ( stream.str() ) // TODO

    glPolygonMode ( GL_FRONT_AND_BACK, GL_FILL );
    glEnable ( GL_TEXTURE_2D );
    glBindTexture ( GL_TEXTURE_2D, m_TextureId );

    if ( m_TextureFormat == ImageGrabber::RGB8 )
    {
      glTexImage2D ( GL_TEXTURE_2D, 0, GL_RGB, m_TextureResolution, m_TextureResolution, 0, GL_RGB, GL_UNSIGNED_BYTE, m_TextureData );
    }
    else
    {
      glTexImage2D ( GL_TEXTURE_2D, 0, GL_LUMINANCE, m_TextureResolution, m_TextureResolution, 0, GL_LUMINANCE, GL_UNSIGNED_BYTE, m_TextureData );
    }

    glTexParameterf ( GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST );
    glTexParameterf ( GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST );
    glTexEnvf ( GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE );

    float usedTextureY = float ( m_ImageHeight ) / float ( m_TextureResolution );
    float usedTextureX = float ( m_ImageWidth ) / float ( m_TextureResolution );

    //draw
    glBegin ( GL_POLYGON );
    glTexCoord2f ( 0.0,  0.0 );
    glVertex3f ( -0.5, 0.5, 0.0 );
    glTexCoord2f ( 0.0,  usedTextureY );
    glVertex3f ( -0.5, -0.5, 0.0 );
    glTexCoord2f ( usedTextureX, usedTextureY );
    glVertex3f ( 0.5, -0.5, 0.0 );
    glTexCoord2f ( usedTextureX,  0.0 );
    glVertex3f ( 0.5, 0.5, 0.0 );
    glEnd();

    glDisable ( GL_TEXTURE_2D );
  }


}


void THIS::paintVectorObjects()
{
  glPolygonMode ( GL_FRONT_AND_BACK, GL_LINE );

  //paint white border
  glColor3f ( 1.0, 1.0, 1.0 );
  glLineWidth( 1.0 );
  glBegin ( GL_POLYGON );
  glVertex3f ( -0.5, -0.5, 0.0 );
  glVertex3f ( -0.5, 0.5, 0.0 );
  glVertex3f ( 0.5, 0.5, 0.0 );
  glVertex3f ( 0.5, -0.5, 0.0 );
  glEnd();

  glColor3f ( 1.0, 1.0, 1.0 );
  glPushMatrix();
  //scale to fit [-0.5..0.5]
  glScalef ( 1.0 , -1.0, 1.0 );
  glTranslatef ( -0.5, -0.5, 0.0 );
  //scale to fit [0..1]
  glScalef ( 1.0 / m_ImageWidth, 1.0 / m_ImageHeight, 1.0 );
  //translate vertex coordinates to pixel centers (instead of top-left pixel corner)
  glTranslatef ( 0.5, 0.5, 0.0 );

  std::list< VectorObject2D >::iterator vectorObjectIt;
  vectorObjectIt = m_VectorObjects.begin();
  while ( vectorObjectIt != m_VectorObjects.end() )
  {
    vectorObjectIt->paintGl();
    vectorObjectIt++;
  }

  glPopMatrix();
}


void THIS::setColorImage ( const ColorImageRGB8* image, float aspect )
{
  if ( !image )
  {
    //TRACE_ERROR ( "Received 0-pointer!" ); // TODO
    return;
  }
  setPixelAspectRatio ( aspect );
  updateTexture ( ( unsigned char* ) ( const_cast<ColorImageRGB8*> ( image )->unsafeRowPointerArray() [0][0] ), image->getWidth(), image->getHeight(), ImageGrabber::RGB8 );
}

void THIS::setColorImage ( const GrayLevelImage8* yImage, const ColorImageUV8* uvImage, float aspect )
{
  if ( ( !yImage ) || ( !uvImage ) )
  {
    //TRACE_ERROR ( "Received 0-pointer!" ); // TODO
    return;
  }

  ColorImageRGB8 rgbImage;
  Y8UV8ToRGB8Operator o ( *yImage, *uvImage, rgbImage );
  setPixelAspectRatio ( aspect );
  updateTexture ( ( unsigned char* ) ( rgbImage.unsafeRowPointerArray() [0][0] ), rgbImage.getWidth(), rgbImage.getHeight(), ImageGrabber::RGB8 );
}

void THIS::setGrayLevelImage ( const puma2::GrayLevelImage8* image, float aspect )
{
  if ( !image )
  {
    //TRACE_ERROR ( "Received 0-pointer!" ); // TODO
    return;
  }

  setPixelAspectRatio ( aspect );
  updateTexture ( ( unsigned char* ) ( const_cast<GrayLevelImage8*> ( image )->unsafeRowPointerArray() [0] ), image->getWidth(), image->getHeight(), ImageGrabber::GRAY8 );
}

void THIS::setThermalImage ( const puma2::GrayLevelImage8* image, float roomTemp )
{
  if ( !image )
  {
    //TRACE_ERROR ( "Received 0-pointer!" ); // TODO
    return;
  }
  //TRACE_INFO ( "Received a new ThermalImage" ); // TODO

  ColorImageRGB8 coloredThermalImage;
  ThermalToColorOperator ( *image, coloredThermalImage, roomTemp - 20, roomTemp + 100 );

  //by default, stretch to display as 4:3 image
  float width = image->getWidth();
  float height = image->getHeight();
  float correctAspect = 4.0 / 3.0;
  float imageAspect = float ( width ) / float ( height );
  setColorImage ( &coloredThermalImage, correctAspect / imageAspect );
}

void THIS::setAnaglyphImage ( const puma2::GrayLevelImage8* leftYImage, const puma2::GrayLevelImage8* rightYImage, float aspect )
{
  if ( !leftYImage || !rightYImage )
  {
    // TRACE_ERROR ( "Received 0-pointer!" ); // TODO
    return;
  }

  int w = leftYImage->getWidth();
  int h = leftYImage->getHeight();

  setPixelAspectRatio ( aspect );

  unsigned char* tempImage = new unsigned char[w*h*3];
  unsigned char* left = ( unsigned char* ) ( const_cast<GrayLevelImage8*> ( leftYImage )->unsafeRowPointerArray() [0] );
  unsigned char* right = ( unsigned char* ) ( const_cast<GrayLevelImage8*> ( rightYImage )->unsafeRowPointerArray() [0] );


  int anaglyphOffset = 1; // TODO Config::getInt ( "Gui.iAnaglyphOffset" );

  //Move left image <anaglyphOffset> pixels to the left and put into R channel
  //Move right image <anaglyphOffset> pixels to the right and put into G+B channel

  for ( int y = 0; y < h; y++ )
  {
    int i = y * w;
    int j = i * 3;
    for ( int x = 0; x < w - anaglyphOffset; x++ )
    {
      tempImage[j] = left[i+anaglyphOffset];
      i++;
      j += 3;
    }
    for ( int x = w - anaglyphOffset; x < w; x++ )
    {
      tempImage[j] = 0;
      j += 3;
    }
  }
  for ( int y = 0; y < h; y++ )
  {
    int i = y * w;
    int j = i * 3;
    for ( int x = 0; x < anaglyphOffset; x++ )
    {
      tempImage[j+1] = 0;
      tempImage[j+2] = 0;
      j += 3;
      i++;
    }
    for ( int x = anaglyphOffset; x < w; x++ )
    {
      tempImage[j+1] = right[i-anaglyphOffset];
      tempImage[j+2] = right[i-anaglyphOffset];
      j += 3;
      i++;
    }
  }

  updateTexture ( tempImage, w, h, ImageGrabber::RGB8 );

  delete tempImage;
}


void THIS::addVectorObject ( VectorObject2D vectorObject )
{
  m_VectorObjects.push_back ( vectorObject );
}


void THIS::clearForms()
{
  m_VectorObjects.clear();
}


// INTERNAL FUNCTIONS/////////////////////////////////////////////////////////////////////////////////////

void THIS::updateTexture ( unsigned char* imageData, unsigned width, unsigned height, ImageGrabber::ColorFormat format )
{
  unsigned char* texturePos;
  unsigned char* imagePos;

  // recreate texture if necessary
  unsigned newTextureResolution = GLImagePainter::calcTextureSize ( width, height );
  if ( ( m_TextureData == 0 ) || ( m_TextureResolution != newTextureResolution ) )
  {
    std::ostringstream stream;
    int newTextureRes = GLImagePainter::calcTextureSize ( width, height );
    stream << "Resizing Texture to " << newTextureRes << "x" << newTextureRes;
    //TRACE_SYSTEMINFO ( stream.str() ); // TODO
    initTexture ( newTextureRes, width, height, format );
  }
  //clear texture if image size has changed but texture size doesn't have to change
  else if ( ( m_ImageWidth != width ) || ( m_ImageHeight != height ) )
  {
    clearTexture();
  }

  std::ostringstream stream;
  stream << "Updating  texture";
  // TRACE_SYSTEMINFO ( stream.str() ) // TODO

  m_ImageWidth = width;
  m_ImageHeight = height;
  m_TextureFormat = format;

  unsigned bytesPerPixel = 1;
  if ( format == ImageGrabber::RGB8 ) { bytesPerPixel = 3; }

  //draw in top-left corner of the texture
  texturePos = m_TextureData;
  imagePos = imageData;

  //draw new image in the center
  for ( unsigned y = 0; y < height ; y++ )
  {
    memcpy ( texturePos, imagePos, width*bytesPerPixel );
    //jump to next row in texture
    texturePos += m_TextureResolution * bytesPerPixel;
    imagePos += width * bytesPerPixel;
  }
}

void THIS::clearTexture()
{
  memset ( m_TextureData, 128, m_TextureByteSize );
}


void THIS::initTexture ( unsigned resolution, unsigned imageWidth, unsigned imageHeight, ImageGrabber::ColorFormat textureFormat )
{
  m_ImageWidth = imageWidth;
  m_ImageHeight = imageHeight;

  unsigned bytesPerPixel = 1;
  if ( textureFormat == ImageGrabber::RGB8 ) { bytesPerPixel = 3; }

  m_TextureResolution = resolution;

  if ( m_TextureData ) { delete m_TextureData; }
  m_TextureByteSize = m_TextureResolution * m_TextureResolution * bytesPerPixel;
  m_TextureData = new unsigned char[ m_TextureByteSize ];
  clearTexture();
}


void THIS::saveImage ( std::string filename )
{
  if ( !m_TextureData )
  {
    //TRACE_ERROR ( "No image set!" ); // TODO
    return;
  }
  // TRACE_INFO ( "Saving texture image to " + filename ); // TODO
  switch ( m_TextureFormat )
  {
    case ImageGrabber::RGB8:
    {
      ColorImageRGB8 image ( m_ImageWidth, m_ImageHeight );
      //copy from top-left corner of the texture
      for ( unsigned y = 0; y < m_ImageHeight ; y++ )
      {
        for ( unsigned x = 0; x < m_ImageWidth; x++ )
        {
          image[y][x][0] = m_TextureData[x*3+y*m_TextureResolution*3];
          image[y][x][1] = m_TextureData[x*3+y*m_TextureResolution*3+1];
          image[y][x][2] = m_TextureData[x*3+y*m_TextureResolution*3+2];
        }
      }
      //ImageWriter::writeImage ( image, filename ); // TODO
      break;
    }
    case ImageGrabber::GRAY8:
    {
      GrayLevelImage8 image ( m_ImageWidth, m_ImageHeight );
      //copy from top-left corner of the texture
      for ( unsigned y = 0; y < m_ImageHeight ; y++ )
      {
        for ( unsigned x = 0; x < m_ImageWidth; x++ )
        {
          image[y][x] = m_TextureData[x+y*m_TextureResolution];
        }
      }
     // ImageWriter::writeImage ( image, filename ); // TODO
      break;
    }
    default:
      // TRACE_ERROR ( "Cannot save image: Invalid format!" ); // TODO
      break;
  }
}


// TODO this method is from RobbieGLWidget --> leave here or find better location for it
int THIS::calcTextureSize( int width, int height )
{
    int sizeSqared = 2;
    //find the next potence of two into which the image fits
    while ( ( sizeSqared < width || sizeSqared < height ) && ( sizeSqared <= 256*256 ) )
    {
      sizeSqared = sizeSqared * 2;
    }
    return sizeSqared;
}



#undef THIS
