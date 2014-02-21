/*******************************************************************************
 *  MapGLWidget.cpp
 *
 *  (C) 2006 AG Aktives Sehen <agas@uni-koblenz.de>
 *           Universitaet Koblenz-Landau
 ******************************************************************************/


#include "GLImageWidget.h"

//#include "Architecture/Config/Config.h" // TODO
#include "../../Workers/Puma2/ThermalToColorOperator.h"
#include "../../Workers/Puma2/Y8UV8ToRGB8Operator.h"
// #include "../../Workers/Puma2/ImageWriter.h"

#include <sstream>
#include <QMouseEvent>

using namespace puma2;

#define THIS GLImageWidget

const float ZOOM_STEP = sqrt(sqrt(2));

THIS::THIS(QGLWidget *parent) : QGLWidget(parent) {

  setCursor(Qt::SizeAllCursor);

  m_Zoom=1.0;
  m_PosX=0.0;
  m_PosY=0.0;

  m_ImageWidth=1;
  m_ImageHeight=1;
  m_ViewportWidth=1;
  m_ViewportHeight=1;

  setPixelAspectRatio(1.0);

  m_TextureId=-1;
  m_TextureData=0;
  m_TextureResolution=0;
  m_TextureFormat=ImageGrabber::GRAY8;
  m_TextureByteSize=0;

  setMinimumSize(40,40);

}

THIS::~THIS() {
  if( m_TextureData ) {
    //TRACE_SYSTEMINFO( "Deleting texture memory" ) // TODO use ros
    delete[] m_TextureData;
  }
  if ( int(m_TextureId) != -1 ) {
    // TRACE_SYSTEMINFO( "Deleting OpenGL texture" ) // TODO use ros
    glDeleteTextures(1,&m_TextureId);
  }
}

void THIS::initializeGL() {

  glClearColor(1.0, 1.0, 1.0, 0.0);

  QSize s = size();
  int w = s.width();
  int h = s.height();
  m_ViewportWidth=w;
  m_ViewportHeight=h;
  glViewport(0, 0, m_ViewportWidth, m_ViewportHeight);
  glDisable( GL_DEPTH_TEST );
}

void THIS::resizeGL(int w, int h)
{
  m_ViewportWidth=w;
  m_ViewportHeight=h;
  glViewport(0, 0, m_ViewportWidth, m_ViewportHeight);
  updateGL();
}

void THIS::paintGL() {

  //only update if visible on screen
  if (isVisible())
  {
    glClear( GL_COLOR_BUFFER_BIT );
    glPolygonMode( GL_FRONT_AND_BACK, GL_FILL );

    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    //the Y axis points downwards, center of view in (0,0,0):
  //  glOrtho(-0.5, 0.5, 0.5, -0.5, -1.0, 1.0);
    glOrtho( -0.5, 0.5, 0.5, -0.5, -1.0, 1.0);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();

    //correct aspect
    float viewPortAspect=float(m_ViewportWidth) / float(m_ViewportHeight);
    float imageAspect=float(m_ImageWidth) / float(m_ImageHeight) * m_PixelAspectRatio;
    float scaleX,scaleY;

    if (viewPortAspect > imageAspect) {
      scaleX=imageAspect/viewPortAspect;
      scaleY=1.0;
    } else {
      scaleX=1.0;
      scaleY=viewPortAspect/imageAspect;
    }

    //e.g. 1.0 means the image is as large as the viewport, 2.0 that it is two times larger
    float relativeImageWidth=scaleX*m_Zoom;
    float relativeImageHeight=scaleY*m_Zoom;
    float maxX;
    float maxY;

    if ( relativeImageWidth < 1.0 ) {
      maxX=(0.5/relativeImageWidth)-0.5;
    } else {
      maxX=(0.5*relativeImageWidth)-0.5;
    }
    if ( relativeImageHeight < 1.0 ) {
      maxY=(0.5/relativeImageHeight)-0.5;
    } else {
      maxY=(0.5*relativeImageHeight)-0.5;
    }

    if ( m_PosY >  maxY ) { m_PosY= maxY; }
    if ( m_PosY < -maxY ) { m_PosY=-maxY; }
    if ( m_PosX >  maxX ) { m_PosX= maxX; }
    if ( m_PosX < -maxX ) { m_PosX=-maxX; }

    glTranslatef(m_PosX,m_PosY,0.0);
    glScalef(m_Zoom,m_Zoom,0.0);
    glScalef( scaleX, scaleY, 1.0 );

    if (m_TextureData)
    {
      paintImage();
      paintVectorObjects();
    }
  }
}

void THIS::paintImage(){

  glPolygonMode( GL_FRONT_AND_BACK, GL_FILL );
  glColor3f(1.0, 1.0, 1.0);

  if (m_TextureData) {

    if ( int ( m_TextureId ) == -1 )
    {
      glGenTextures ( 1, &m_TextureId );
      std::ostringstream stream;
      stream << "Using OpenGL texture # " << m_TextureId;
      // TRACE_INFO ( stream.str() ); // TODO use ros
    }
    else
    {
//      glDeleteTextures ( 1, &m_TextureId );
//      glGenTextures ( 1, &m_TextureId );
    }

    std::ostringstream stream;
    stream << "Loading texture data to texture ID " << m_TextureId;
    // TRACE_SYSTEMINFO( stream.str() ) // TODO use ros

    glEnable(GL_TEXTURE_2D);
    glBindTexture(GL_TEXTURE_2D, m_TextureId);

    if (m_TextureFormat==ImageGrabber::RGB8) {
      glTexImage2D( GL_TEXTURE_2D, 0, GL_RGB, m_TextureResolution, m_TextureResolution, 0, GL_RGB, GL_UNSIGNED_BYTE, m_TextureData );
    } else {
      glTexImage2D( GL_TEXTURE_2D, 0, GL_LUMINANCE, m_TextureResolution, m_TextureResolution, 0, GL_LUMINANCE, GL_UNSIGNED_BYTE, m_TextureData );
    }
  }

  glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
  glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST );
  glTexEnvf( GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE );

  float usedTextureY=float(m_ImageHeight) / float(m_TextureResolution);
  float usedTextureX=float(m_ImageWidth) / float(m_TextureResolution);

  //draw
  glColor3f(0.0, 1.0, 0.0);
  glBegin(GL_POLYGON);
    glTexCoord2f(  0.0,  0.0 );
    glVertex3f( -0.5, -0.5, 0.0 );
    glTexCoord2f(  0.0,  usedTextureY );
    glVertex3f( -0.5, 0.5, 0.0 );
    glTexCoord2f(  usedTextureX, usedTextureY  );
    glVertex3f( 0.5, 0.5, 0.0 );
    glTexCoord2f(  usedTextureX,  0.0 );
    glVertex3f( 0.5, -0.5, 0.0 );
  glEnd();

  glDisable(GL_TEXTURE_2D);
}


void THIS::paintVectorObjects()
{
  glPolygonMode( GL_FRONT_AND_BACK, GL_LINE );
  glColor3f(1.0, 1.0, 1.0);
  glPushMatrix();
  //scale to fit [-0.5..0.5]
  glTranslatef( -0.5, -0.5, 0.0 );
  //scale to fit [0..1]
  glScalef( 1.0/m_ImageWidth, 1.0/m_ImageHeight, 1.0 );
  //translate vertex coordinates to pixel centers (instead of top-left pixel corner)
  glTranslatef( 0.5, 0.5, 0.0 );

  std::list< VectorObject2D >::iterator vectorObjectIt;
  vectorObjectIt = m_VectorObjects.begin();
  while( vectorObjectIt != m_VectorObjects.end() )
  {
    vectorObjectIt->paintGl();
    vectorObjectIt++;
  }

  glPopMatrix();
}

void THIS::setColorImage(const unsigned char *image, unsigned width, unsigned height, float aspect)
{
    if (!image) {
      //ROS_ERROR_STREAM("Received 0-pointer!");
      return;
    }
    setPixelAspectRatio( aspect );

//    unsigned char convertedImage[image->size()];
//    for(unsigned i = 0; i < image->size(); i++)
//    {
//        convertedImage[i] = image->at(i);
//    }
    updateTexture( (unsigned char*)(const_cast<unsigned char*>(image)), width, height, ImageGrabber::RGB8 );
}

void THIS::setColorImage(const ColorImageRGB8* image, float aspect )
{
  if (!image) {
    // TRACE_ERROR("Received 0-pointer!"); // TODO use ros
    return;
  }
  setPixelAspectRatio( aspect );
  updateTexture( (unsigned char*)(const_cast<ColorImageRGB8*>(image)->unsafeRowPointerArray()[0][0]), image->getWidth(), image->getHeight(), ImageGrabber::RGB8 );
}

void THIS::setColorImage(const GrayLevelImage8* imageY, const ColorImageUV8* imageUV, float aspect )
{
  if ((!imageY) || (!imageUV)) {
    //TRACE_ERROR("Received 0-pointer!"); // TODO use ros
    return;
  }

  ColorImageRGB8 imageRGB;
  Y8UV8ToRGB8Operator o( *imageY, *imageUV, imageRGB );
  setPixelAspectRatio( aspect );
  updateTexture( (unsigned char*)( imageRGB.unsafeRowPointerArray()[0][0]), imageRGB.getWidth(), imageRGB.getHeight(), ImageGrabber::RGB8 );
}

void THIS::setGrayLevelImage(const puma2::GrayLevelImage8* image, float aspect )
{
  if (!image) {
    // TRACE_ERROR("Received 0-pointer!"); // TODO use ros
    return;
  }

  setPixelAspectRatio( aspect );
  updateTexture( (unsigned char*)(const_cast<GrayLevelImage8*>(image)->unsafeRowPointerArray()[0]), image->getWidth(), image->getHeight(), ImageGrabber::GRAY8 );
}

void THIS::setThermalImage(const puma2::GrayLevelImage8* image, float roomTemp )
{
  if (!image) {
    //TRACE_ERROR("Received 0-pointer!"); // TODO use ros
    return;
  }
  // TRACE_INFO("Received a new ThermalImage");// TODO use ros

  ColorImageRGB8 coloredThermalImage;
  ThermalToColorOperator( *image, coloredThermalImage, roomTemp-20, roomTemp+100 );

  //by default, stretch to display as 4:3 image
  float width=image->getWidth();
  float height=image->getHeight();
  float correctAspect= 4.0 / 3.0;
  float imageAspect=float(width) / float(height);
  setColorImage( &coloredThermalImage, correctAspect / imageAspect );
}

void THIS::setAnaglyphImage( const puma2::GrayLevelImage8* leftYImage, const puma2::GrayLevelImage8* rightYImage, float aspect )
{
  if ( !leftYImage || !rightYImage ) {
    // TRACE_ERROR("Received 0-pointer!"); // TODO use ros
    return;
  }

  int w=leftYImage->getWidth();
  int h=leftYImage->getHeight();

  setPixelAspectRatio( aspect );

  unsigned char* tempImage=new unsigned char[w*h*3];
  unsigned char* left=(unsigned char*)(const_cast<GrayLevelImage8*>(leftYImage)->unsafeRowPointerArray()[0]);
  unsigned char* right=(unsigned char*)(const_cast<GrayLevelImage8*>(rightYImage)->unsafeRowPointerArray()[0]);


  int anaglyphOffset=1; // TODO add config Config::getInt( "Gui.iAnaglyphOffset" );

  //Move left image <anaglyphOffset> pixels to the left and put into R channel
  //Move right image <anaglyphOffset> pixels to the right and put into G+B channel

  for ( int y=0; y<h; y++ )
  {
    int i=y*w;
    int j=i*3;
    for ( int x=0; x<w-anaglyphOffset; x++ )
    {
      tempImage[j]=left[i+anaglyphOffset];
      i++;
      j+=3;
    }
    for ( int x=w-anaglyphOffset; x<w; x++ )
    {
      tempImage[j]=0;
      j+=3;
    }
  }
  for ( int y=0; y<h; y++ )
  {
    int i=y*w;
    int j=i*3;
    for ( int x=0; x<anaglyphOffset; x++ )
    {
      tempImage[j+1]=0;
      tempImage[j+2]=0;
      j+=3;
      i++;
    }
    for ( int x=anaglyphOffset; x<w; x++ )
    {
      tempImage[j+1]=right[i-anaglyphOffset];
      tempImage[j+2]=right[i-anaglyphOffset];
      j+=3;
      i++;
    }
  }

  updateTexture( tempImage, w, h, ImageGrabber::RGB8 );

  delete tempImage;
}


void THIS::addVectorObject( VectorObject2D vectorObject )
{
  m_VectorObjects.push_back( vectorObject );
}


void THIS::clearForms()
{
  m_VectorObjects.clear();
}


// INTERNAL FUNCTIONS/////////////////////////////////////////////////////////////////////////////////////

void THIS::updateTexture( unsigned char* imageData, unsigned width, unsigned height, ImageGrabber::ColorFormat format )
{
  unsigned char* texturePos;
  unsigned char* imagePos;

  // recreate texture if necessary
  unsigned newTextureResolution=calcTextureSize(width,height); // TODO
  if ( (m_TextureData==0) || (m_TextureResolution!=newTextureResolution) )
  {
    std::ostringstream stream;
    int newTextureRes= calcTextureSize(width,height); // TODO
    stream << "Resizing Texture to " << newTextureRes << "x" << newTextureRes;
    // TRACE_SYSTEMINFO(stream.str()); // TODO use ros
    initTexture( newTextureRes, width, height, format );
  }
  //clear texture if image size has changed but texture size doesn't have to change
  else if ( (m_ImageWidth != width) || (m_ImageHeight != height) ) {
    clearTexture();
  }

  std::ostringstream stream;
  stream << "Updating  texture";
  // TRACE_SYSTEMINFO( stream.str() ) // TODO use ros

  m_ImageWidth=width;
  m_ImageHeight=height;
  m_TextureFormat=format;

  unsigned bytesPerPixel=1;
  if (format==ImageGrabber::RGB8) { bytesPerPixel=3; }

  //draw in top-left corner of the texture
  texturePos=m_TextureData;
  imagePos=imageData;

  //draw new image in the center
  for( unsigned y = 0; y < height ; y++ ) {
    memcpy ( texturePos, imagePos, width*bytesPerPixel );
    //jump to next row in texture
    texturePos+=m_TextureResolution*bytesPerPixel;
    imagePos+=width*bytesPerPixel;
  }

  updateGL();
}

void THIS::clearTexture()
{
  memset(m_TextureData, 128, m_TextureByteSize );
}


void THIS::initTexture( unsigned resolution, unsigned imageWidth, unsigned imageHeight, ImageGrabber::ColorFormat textureFormat )
{
  m_ImageWidth=imageWidth;
  m_ImageHeight=imageHeight;

  unsigned bytesPerPixel=1;
  if (textureFormat==ImageGrabber::RGB8) { bytesPerPixel=3; }

  m_TextureResolution = resolution;

  if( m_TextureData ) { delete m_TextureData; }
  m_TextureByteSize=m_TextureResolution * m_TextureResolution * bytesPerPixel;
  m_TextureData = new unsigned char[ m_TextureByteSize ];
  clearTexture();
}


void THIS::saveImage( std::string filename )
{
  if ( !m_TextureData )
  {
    //TRACE_ERROR("No image set!"); // TODO use ros
    return;
  }
  // TRACE_INFO("Saving texture image to "+filename); // TODO use ros
  switch( m_TextureFormat )
  {
    case ImageGrabber::RGB8:
    {
      ColorImageRGB8 image( m_ImageWidth, m_ImageHeight );
      //copy from top-left corner of the texture
      for( unsigned y = 0; y < m_ImageHeight ; y++ )
      {
        for( unsigned x = 0; x < m_ImageWidth; x++ )
        {
          image[y][x][0]=m_TextureData[x*3+y*m_TextureResolution*3];
          image[y][x][1]=m_TextureData[x*3+y*m_TextureResolution*3+1];
          image[y][x][2]=m_TextureData[x*3+y*m_TextureResolution*3+2];
        }
      }
      // ImageWriter::writeImage( image, filename ); // TODO
      break;
    }
    case ImageGrabber::GRAY8:
    {
      GrayLevelImage8 image( m_ImageWidth, m_ImageHeight );
      //copy from top-left corner of the texture
      for( unsigned y = 0; y < m_ImageHeight ; y++ )
      {
        for( unsigned x = 0; x < m_ImageWidth; x++ )
        {
          image[y][x]=m_TextureData[x+y*m_TextureResolution];
        }
      }
      // ImageWriter::writeImage( image, filename ); // TODO
      break;
    }
    default:
      break;
     //  TRACE_ERROR("Cannot save image: Invalid format!"); // TODO use ros
  }
}


// USER INTERACTION //////////////////////////////////////////////////////////////////////////////////////////

void THIS::mouseDoubleClickEvent( QMouseEvent* event = 0 )
{
  //correct aspect
  float viewPortAspect=float(m_ViewportWidth) / float(m_ViewportHeight);
  float imageAspect=float(m_ImageWidth) / float(m_ImageHeight) * m_PixelAspectRatio;
//  float scaleX,scaleY;

//   if (viewPortAspect > imageAspect) {
//     scaleX=imageAspect/viewPortAspect;
//     scaleY=1.0;
//   } else {
//     scaleX=1.0;
//     scaleY=viewPortAspect/imageAspect;
//   }
//

  if (viewPortAspect > imageAspect) {
    m_Zoom = float(m_ImageHeight) / float(m_ViewportHeight);
  } else {
    m_Zoom = float(m_ImageWidth) / float(m_ViewportWidth);
  }

  m_PosX=0.0;
  m_PosY=0.0;
  updateGL();
}

void THIS::zoom(float z)
{
  m_Zoom = z;
  m_PosX=0.0;
  m_PosY=0.0;
  updateGL();
}

void THIS::mousePressEvent(QMouseEvent* event)
{
  m_MousePosOld = event->pos();
}

void THIS::mouseMoveEvent(QMouseEvent* event)
{
  float deltaX=float(-m_MousePosOld.x() + event->x());
  float deltaY=float(-m_MousePosOld.y() + event->y());
  if (event->buttons() & Qt::LeftButton) {
    m_PosX += deltaX / m_ViewportWidth;
    m_PosY += deltaY / m_ViewportHeight;
  } else if (event->buttons() & Qt::RightButton) {
    m_Zoom *= std::pow( float(1.1), float(deltaY/10.0) );
    if (m_Zoom < 1.0) { m_Zoom=1.0; }
    if (m_Zoom > 32.0) { m_Zoom=32.0; }
  }
  m_MousePosOld = event->pos();
  updateGL();
}


void THIS::wheelEvent(QWheelEvent *event) {
  if (event->delta() < 0 ) {
    m_Zoom*=ZOOM_STEP;
    //NOTE wrong calculation, to be replaced
    m_PosX+=( 0.5 - float(event->x())/float(m_ViewportWidth ) ) / m_Zoom;
    m_PosY+=( 0.5 - float(event->y())/float(m_ViewportHeight) ) / m_Zoom;
  }
  if (event->delta() > 0 ) {
    m_Zoom/=ZOOM_STEP;
  }
  //if (m_Zoom < 1.0) { m_Zoom=1.0; }
  if (m_Zoom > 32.0) { m_Zoom=32.0; }
  updateGL();
}


// TODO originally in RobbieWidget
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
