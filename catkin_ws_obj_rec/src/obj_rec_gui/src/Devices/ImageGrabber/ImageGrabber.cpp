/*******************************************************************************
*  ImageGrabber.cpp
*
*  (C) 2006 AG Aktives Sehen <agas@uni-koblenz.de>
*           Universitaet Koblenz-Landau
******************************************************************************/

#include <limits.h>
#include "ImageGrabber.h"

#include "../../Workers/Puma2/Y8UV8ToRGB8Operator.h"
#include "Workers/Math/Math.h"

#include <sstream>
#include <exception>
#include "math.h"
#include "stdlib.h"

//#include "GrabbingDeviceManager.h"
#include "../../Workers/Puma2/PumaException.h"

#define THIS ImageGrabber

using namespace std;
using namespace puma2;

THIS::THIS ( string parentName )
{
//  m_ColorFormatId["RGB8"] = RGB8;
//  m_ColorFormatId["UYVY"] = UYVY8;
//  m_GrabDataTimer = Timer ( ProfilerEntry::CODE_SEGMENT, parentName, "Grab raw data" );
//  m_GrabImageTimer = Timer ( ProfilerEntry::CODE_SEGMENT, parentName, "Grab RGB image" );
//  m_ScaleYuvDataTimer = Timer ( ProfilerEntry::CODE_SEGMENT, parentName, "YUV422 -> YUV444" );
//  m_YuvToRgbTimer = Timer ( ProfilerEntry::CODE_SEGMENT, parentName, "YUV444 -> RGB" );

//  GrabbingDeviceManager* myManager = GrabbingDeviceManager:: getGrabbingDeviceManager();
//  myManager->scan();
  //TRACE_SYSTEMINFO( "Connected cameras:\n" + myManager->getDeviceList() )
}

THIS::~THIS()
{

//  map<ImageSources::SourceId, GrabbingDevice*>::iterator it;

//  //delete devices
//  for ( it = m_GrabbingDevices.begin() ; it != m_GrabbingDevices.end(); it++ )
//  {
//    stopCamera ( it->first );
//    //NOTE we should delete the grabbing devices here, but this causes the GrabbingDevices to crash for some reasons (possible puma2 bug)
//    delete it->second;
//  }

}

//bool THIS::setCamera ( ImageSources::SourceId sourceId, Camera cam )
//{
//  m_Cameras[sourceId] = cam;
//  m_Initialized[sourceId] = false;
//  return initCamera ( sourceId );
//}

//bool THIS::stopCamera ( ImageSources::SourceId sourceId )
//{

////  //check if the camera exists and is initialized
////  if ( m_Cameras.find ( sourceId ) != m_Cameras.end() && m_Initialized[sourceId] )
////  {
////    ostringstream stream;
////    stream << "Stopping camera " << sourceId << " (" << m_Cameras[sourceId].getDeviceDescription() << ")";
////    TRACE_WARNING( stream.str() );

////    m_Initialized[sourceId] = false;

////    try
////    {
////      if ( !m_GrabbingDevices[sourceId]->stopCapture() )
////      {
////        TRACE_ERROR( "Stopping capture process failed!" );
////        return false;
////      }
////    }
////    catch ( const char* msg )
////    {
////      TRACE_ERROR( "Caught exception from stopCapture(): " + string ( msg ) );
////      return false;
////    }
////  }

////  usleep ( 50000 );
//  return true;
//}



//bool THIS::setCameraProperty ( ImageSources::SourceId sourceId, string param, double value, bool isAbsolute )
//{
////  GrabbingDevice *grabbingDevice = m_GrabbingDevices[sourceId];
////  if ( !grabbingDevice )
////  {
////    TRACE_ERROR( "Camera not found" );
////    return false;
////  }

////  if ( !isAbsolute )
////  {
////    value += getCameraProperty( sourceId, param );
////  }

////  grabbingDevice->setProperty ( param , value, false );

//  return true;
//}


//double THIS::getCameraProperty ( ImageSources::SourceId sourceId, string param )
//{
////  GrabbingDevice *grabbingDevice = m_GrabbingDevices[sourceId];
////  if ( !grabbingDevice )
////  {
////    TRACE_ERROR( "Camera not found" );
////    return false;
////  }
////  unicap_property_t* _property;

////  grabbingDevice->getProperty ( &_property, param );

//  return _property->value;
//}


//void THIS::cleanBuffers()
//{
//  map<ImageSources::SourceId, GrabbingDevice*>::iterator it;
//  for ( it = m_GrabbingDevices.begin() ; it != m_GrabbingDevices.end(); it++ )
//  {
//    it->second->cleanBuffers();
//  }
//}


//bool THIS::initCamera ( ImageSources::SourceId sourceId )
//{
//  ostringstream stream;

//  stopCamera ( sourceId );

//  stream << "Initializing camera '" << sourceId << "'.";
//  stream << "\nDescription:       '" << m_Cameras[sourceId].getDeviceDescription() << "'";
//  stream << "\nFormat:            " << m_Cameras[sourceId].getFormatId() << "." << m_Cameras[sourceId].getSubFormatId();
//  //stream << "\nCustom properties: '" << m_Cameras[sourceId].getCustomProperties() << "'";

//  stream.str ( "" );

//  //if it has been initialized before, use existing grabbing device
//  if ( m_GrabbingDevices.find ( sourceId ) != m_GrabbingDevices.end() )
//  {

//  }
//  else
//  {
//    // get instance of GrabbingDeviceManager and scan for new devices
//    GrabbingDeviceManager* myManager = GrabbingDeviceManager:: getGrabbingDeviceManager();
//    myManager->scan();

//    //connect to grabbing device (which is identified by a 'device description' string)
//    GrabbingDevice *grabbingDevice;
//    try
//    {
//      myManager->connectGrabbingDevice ( &grabbingDevice, m_Cameras[sourceId].getDeviceDescription() );
//      m_GrabbingDevices[sourceId] = grabbingDevice;
//    }
//    catch ( PumaException pumaException )
//    {
//      stream << "connectGrabbingDevice() has thrown exception: " << pumaException.description();

//      stream.str ( "" );
//      return false;
//    }

//  }

//  m_Initialized[sourceId] = true;



//  //set image format
//  try
//  {
//    m_GrabbingDevices[sourceId]->setFormat ( m_Cameras[sourceId].getFormatId(), m_Cameras[sourceId].getSubFormatId() );
//  }
//  catch ( PumaException pumaException )
//  {
//    stream << "setFormat() has thrown exception: " << pumaException.description();

//    stream.str ( "" );
//    return false;
//  }

//  stream << "Selected format: " << m_Cameras[sourceId].getFormatId() << "." << m_Cameras[sourceId].getSubFormatId();
//  stream << " (" << m_GrabbingDevices[sourceId]->getFrameWidth() << "x" << m_GrabbingDevices[sourceId]->getFrameHeight();
//  stream << ", " << m_GrabbingDevices[sourceId]->getFormatDesc() << ")";

//  stream.str("");

//  //set number of buffers
//  try
//  {
//    m_GrabbingDevices[sourceId]->setNumBuffers ( 2 );
//  }
//  catch ( PumaException pumaException )
//  {
//    stream << "setNumBuffers() has thrown exception: " << pumaException.description();

//    stream.str ( "" );
//    return false;
//  }



//  //set custom properties
//  std::map<string, double> properties = m_Cameras[sourceId].getCustomProperties();
//  std::map<string, double>::iterator propIt;
//  for ( propIt = properties.begin(); propIt != properties.end(); propIt++ )
//  {
//    try
//    {
//      stream.str ( "" );
//      stream << "Setting property " << propIt->first << " = " << propIt->second;

//      m_GrabbingDevices[sourceId]->setProperty ( propIt->first , propIt->second, false );
//    }
//    catch ( PumaException pumaException )
//    {
//      stream << "setProperty(" << propIt->first << propIt->second << ") has thrown exception: " << pumaException.description();

//      stream.str ( "" );
//        //return false;
//    }
//  }

//  // start capture process
//  try
//  {
//    m_GrabbingDevices[sourceId]->startCapture();
//  }
//  catch ( PumaException pumaException )
//  {
//    stream << "startCapture() has thrown exception: " << pumaException.description();

//    stream.str ( "" );
//    return false;
//  }

//  usleep ( 50000 );

//  return true;
//}


//bool THIS::checkCamera ( ImageSources::SourceId sourceId )
//{
//  ostringstream stream;
//  if ( m_Cameras.find ( sourceId ) == m_Cameras.end() )
//  {
//    stream << "Camera " << ( int ) sourceId << " does not exist!";
////    throw exception(stream.str());

//    return false;
//  }
//  if ( !m_Initialized[sourceId] )
//  {
//    stream << "Camera " << ( int ) sourceId << " not initialized!";

//    return false;
//  }
//  return true;
//}


//bool THIS::grabImageRgb ( ImageSources::SourceId sourceId, ScaleFactor scaling, ImageQuality quality, ColorImageRGB8 &image )
//{
//  ostringstream stream;
//  bool grabSuccess = false;

//  if ( !checkCamera ( sourceId ) ) { return false; };

//  stream << "Grabbing RGB camera image " << sourceId << " (" << m_Cameras[sourceId].getDeviceDescription() << ")";

//  stream.str ( "" );


	
//	ColorImageRGB8 tmpImage;

//  switch ( scaling )
//  {
//    case FULL:
//      grabSuccess = m_GrabbingDevices[sourceId]->grabImage ( tmpImage );
//      break;

//    case HALF:
//    {
//      ColorImageRGB8* fullImage = new ColorImageRGB8();
//      grabSuccess = m_GrabbingDevices[sourceId]->grabImage ( *fullImage );

//      tmpImage.resize( fullImage->getWidth()/2, fullImage->getHeight()/2 );

//      //resize image
//      ColorImageRGB8::PixelType** row = fullImage->unsafeRowPointerArray();
//      ColorImageRGB8::PixelType** lowresRow = tmpImage.unsafeRowPointerArray();

//      int lowresHeight=tmpImage.getHeight();
//      int lowresWidth=tmpImage.getWidth();

//      //ptr. to current row in orig. image
//      ColorImageRGB8::PixelType* currentRow;
//      ColorImageRGB8::PixelType* currentLowresRow;
//      int x,y,x2,y2;
//      y2=0;
//      for( y = 0; y < lowresHeight ; y++ ) {
//        currentRow=row[y2];
//        currentLowresRow=lowresRow[y];
//        x2=0;
//        for( x = 0; x < lowresWidth ; x++ ) {
//          currentLowresRow[x][0]=currentRow[x2][0];
//          currentLowresRow[x][1]=currentRow[x2][1];
//          currentLowresRow[x][2]=currentRow[x2][2];
//          x2+=2;
//        }
//        y2+=2;
//      }
//      delete fullImage;
//      break;
//    }

//    case QUARTER:
//    {
//      ColorImageRGB8* fullImage = new ColorImageRGB8();
//      grabSuccess = m_GrabbingDevices[sourceId]->grabImage ( *fullImage );

//      tmpImage.resize( fullImage->getWidth()/4, fullImage->getHeight()/4 );

//      //resize image
//      ColorImageRGB8::PixelType** row = fullImage->unsafeRowPointerArray();
//      ColorImageRGB8::PixelType** lowresRow = tmpImage.unsafeRowPointerArray();

//      int lowresHeight=tmpImage.getHeight();
//      int lowresWidth=tmpImage.getWidth();

//      //ptr. to current row in orig. image
//      ColorImageRGB8::PixelType* currentRow;
//      ColorImageRGB8::PixelType* currentLowresRow;
//      int x,y,x2,y2;
//      y2=0;
//      for( y = 0; y < lowresHeight ; y++ ) {
//        currentRow=row[y2];
//        currentLowresRow=lowresRow[y];
//        x2=0;
//        for( x = 0; x < lowresWidth ; x++ ) {
//          currentLowresRow[x][0]=currentRow[x2][0];
//          currentLowresRow[x][1]=currentRow[x2][1];
//          currentLowresRow[x][2]=currentRow[x2][2];
//          x2+=4;
//        }
//        y2+=4;
//      }
//      delete fullImage;
//      break;
//    }
//  }
	
//	//rotate image if necessary (slow but works)
//	if ( m_Cameras[sourceId].getRotateImage() == false )
//	{
//		image = tmpImage;
//	}
//	else
//	{
//		image.resize( tmpImage.getHeight(), tmpImage.getWidth() );
//		for ( unsigned y=0; y<tmpImage.getHeight(); y++ )
//		{
//			for ( unsigned x=0; x<tmpImage.getWidth(); x++ )
//			{
//// 				image[x][y] = tmpImage[tmpImage.getHeight()-1-y][x];
//				image[x][y] = tmpImage[y][tmpImage.getWidth()-1-x];
//// 				image[x][y] = tmpImage[y][x];
//			}
//		}
//	}



//  if ( !grabSuccess )
//  {
//    stream << "grabImage(" << ( int ) sourceId << ") failed.";

//    return false;
//  }

//  return true;
//}


//bool THIS::grabImageYuv ( ImageSources::SourceId sourceId, ScaleFactor scaling, ImageQuality quality, puma2::GrayLevelImage8 &yImage, puma2::ColorImageUV8 &uvImage )
//{
//  if ( !checkCamera ( sourceId ) ) { return false; };

//  unsigned width = m_GrabbingDevices[sourceId]->getFrameWidth();
//  unsigned height = m_GrabbingDevices[sourceId]->getFrameHeight();

//  unsigned char* buffer;
//  unsigned bufferSize = width * height * 2;
//  buffer = new unsigned char[bufferSize];

//  if ( !grabDataYuv ( sourceId, buffer ) ) {

//      return false;
//  }

//  bool convertSuccess;

//	//rotate image if necessary (slow but works)
//	if ( m_Cameras[sourceId].getRotateImage() == false )
//	{
//		convertSuccess = yuv422To444 ( quality, width, height, scaling, buffer, yImage, uvImage );
//	}
//	else
//	{
//		puma2::GrayLevelImage8 yImage2;
//		puma2::ColorImageUV8 uvImage2;
		
//		convertSuccess = yuv422To444 ( quality, width, height, scaling, buffer, yImage2, uvImage2 );
		
//		yImage.resize( yImage2.getHeight(), yImage2.getWidth() );
//		uvImage.resize( uvImage2.getHeight(), uvImage2.getWidth() );
		
//		for ( unsigned y=0; y<yImage2.getHeight(); y++ )
//		{
//			for ( unsigned x=0; x<yImage2.getWidth(); x++ )
//			{
//				yImage[x][y] = yImage2[yImage2.getHeight()-1-y][x];
//				uvImage[x][y] = uvImage2[uvImage2.getHeight()-1-y][x];
//			}
//		}
//	}

//  delete[] buffer;

//  return convertSuccess;
//}


//bool THIS::grabDataYuv ( ImageSources::SourceId sourceId, unsigned char* buffer )
//{

//  ostringstream stream;

//  if ( !checkCamera ( sourceId ) ) {

//      return false;
//  };

//  stream << "Grabbing YUV camera image " << sourceId << " (" << m_Cameras[sourceId].getDeviceDescription() << ")";

//  stream.str ( "" );

//  //try to grab image data

//  m_GrabbingDevices[sourceId]->grabData ( buffer, "UYVY" );
//  m_GrabDataTimer.pauseMeasure();

//  /*
//    if ( !grabSuccess )
//    {
//      stream << "grabData(" << (int)sourceId << ") failed.";
//      TRACE_ERROR(stream.str());
//      delete[] buffer;
//      return false;
//    }
//    */
//  return true;
//}


//bool THIS::yuv422To444 ( ImageQuality quality, int origWidth, int origHeight, ScaleFactor scaling, unsigned char* buffer, puma2::GrayLevelImage8 &yImage, puma2::ColorImageUV8 &uvImage )
//{
//  puma2::byte** yRow;
//  ColorImageUV8::PixelType** uvRow;
//  puma2::byte* currentYRow;
//  ColorImageUV8::PixelType* currentUvRow;
//  unsigned x, y, width, height;
//  unsigned rowLength, rowLength2, rowLength3;
//  unsigned char* currentPixel;



//  rowLength = origWidth * 2;
//  rowLength2 = rowLength * 2;
//  rowLength3 = rowLength * 3;


//  switch ( scaling )
//  {
//    case FULL:
//    {
//      //make sure that output size is a multiple of 2
//      width = ( origWidth / 2 ) * 2;
//      height = ( origHeight / 2 ) * 2;
//      yImage.resize ( width, height );
//      uvImage.resize ( width, height );
//      yRow = yImage.unsafeRowPointerArray();
//      uvRow = uvImage.unsafeRowPointerArray();

//      //convert image
//      for ( y = 0; y < height ; y++ )
//      {
//        currentYRow = yRow[y];
//        currentUvRow = uvRow[y];
//        currentPixel = buffer + y * rowLength;
//        for ( x = 0; x < width - 1 ; x += 2 )
//        {
//          currentUvRow[x][1] = currentPixel[2];
//          currentUvRow[x+1][1] = currentPixel[2];
//          currentUvRow[x][0] = currentPixel[0];
//          currentUvRow[x+1][0] = currentPixel[0];
//          currentYRow[x] = currentPixel[1];
//          currentYRow[x+1] = currentPixel[3];
//          currentPixel += 4;
//        }
//      }
//    }
//    break;

//    case HALF:
//    {
//      width = origWidth / 2;
//      height = origHeight / 2;
//      yImage.resize ( width, height );
//      uvImage.resize ( width, height );
//      yRow = yImage.unsafeRowPointerArray();
//      uvRow = uvImage.unsafeRowPointerArray();
//      //convert image
//      if ( quality == LOW )
//      {

//        for ( y = 0; y < height ; y++ )
//        {
//          currentYRow = yRow[y];
//          currentUvRow = uvRow[y];
//          currentPixel = buffer + y * rowLength * 2;
//          for ( x = 0; x < width ; x++ )
//          {
//            currentUvRow[x][1] = currentPixel[2];
//            currentUvRow[x][0] = currentPixel[0];
//            currentYRow[x] = currentPixel[1];
//            currentPixel += 4;
//          }
//        }
//      }
//      else
//      {

//        for ( y = 0; y < height ; y++ )
//        {
//          currentYRow = yRow[y];
//          currentUvRow = uvRow[y];
//          currentPixel = buffer + y * rowLength * 2;
//          for ( x = 0; x < width ; x++ )
//          {
//            currentUvRow[x][0] = unsigned ( ( currentPixel[0] + currentPixel[rowLength] ) / 2 );
//            currentUvRow[x][1] = unsigned ( ( currentPixel[2] + currentPixel[2+rowLength] ) / 2 );
//            currentYRow[x] = unsigned ( ( currentPixel[1] + currentPixel[3] + currentPixel[1+rowLength] + currentPixel[3+rowLength] ) / 4 );
//            currentPixel += 4;
//          }
//        }
//      }
//    }
//    break;

//    case QUARTER:
//    {
//      width = origWidth / 4;
//      height = origHeight / 4;
//      yImage.resize ( width, height );
//      uvImage.resize ( width, height );
//      yRow = yImage.unsafeRowPointerArray();
//      uvRow = uvImage.unsafeRowPointerArray();
//      //convert image
//      if ( quality == LOW )
//      {

//        for ( y = 0; y < height ; y++ )
//        {
//          currentYRow = yRow[y];
//          currentUvRow = uvRow[y];
//          currentPixel = buffer + y * rowLength * 4;
//          for ( x = 0; x < width ; x++ )
//          {
//            currentUvRow[x][0] = currentPixel[0];
//            currentUvRow[x][1] = currentPixel[2];
//            currentYRow[x] = currentPixel[1];
//            currentPixel += 8;
//          }
//        }
//      }
//      else
//      {

//        for ( y = 0; y < height ; y++ )
//        {
//          currentYRow = yRow[y];
//          currentUvRow = uvRow[y];
//          currentPixel = buffer + y * rowLength * 4;
//          for ( x = 0; x < width ; x++ )
//          {
//            currentUvRow[x][0] = unsigned ( ( currentPixel[0] + currentPixel[4]
//                                              + currentPixel[rowLength] + currentPixel[4+rowLength]
//                                              + currentPixel[rowLength2] + currentPixel[4+rowLength2]
//                                              + currentPixel[rowLength3] + currentPixel[4+rowLength3] ) / 8 );
//            currentUvRow[x][1] = unsigned ( ( currentPixel[2] + currentPixel[6]
//                                              + currentPixel[2+rowLength] + currentPixel[6+rowLength]
//                                              + currentPixel[2+rowLength2] + currentPixel[6+rowLength2]
//                                              + currentPixel[2+rowLength3] + currentPixel[6+rowLength3] ) / 8 );
//            currentYRow[x] = unsigned ( ( currentPixel[1] + currentPixel[3]
//                                          + currentPixel[5] + currentPixel[7]
//                                          + currentPixel[1+rowLength] + currentPixel[3+rowLength]
//                                          + currentPixel[5+rowLength] + currentPixel[7+rowLength]
//                                          + currentPixel[1+rowLength2] + currentPixel[3+rowLength2]
//                                          + currentPixel[5+rowLength2] + currentPixel[7+rowLength2]
//                                          + currentPixel[1+rowLength3] + currentPixel[3+rowLength3]
//                                          + currentPixel[5+rowLength3] + currentPixel[7+rowLength3] ) / 16 );
//            currentPixel += 8;
//          }
//        }
//      }
//    }
//    break;

//    default:

//      delete[] buffer;
//      return false;
//      break;
//  }

//  m_ScaleYuvDataTimer.pauseMeasure();

//  return true;
//}

////HACK this only works for x700! The x710 cam is set to graylevel capture by doing this!
//bool THIS::doWhiteBalance ( ImageSources::SourceId sourceId )
//{

//  //check if the camera exists and is initialized
//  if ( m_Cameras.find ( sourceId ) != m_Cameras.end() && m_Initialized[sourceId] )
//  {
//    ostringstream stream;
//    stream << "Doing whitebalance for camera " << sourceId << " (" << m_Cameras[sourceId].getDeviceDescription() << ")";


//    string paramName = "white_balance_mode";
//    unicap_property_t *whiteProp;

//    m_GrabbingDevices[sourceId]->getProperty ( &whiteProp, paramName );
//    whiteProp->flags = UNICAP_FLAGS_ONE_PUSH;

//    m_GrabbingDevices[sourceId]->setProperty ( *whiteProp );
//  }
//  else
//  {
//    return false;
//  }

//  usleep ( 200000 );

//  return true;
//}


//double THIS::zoomToAngle ( double zoom )
//{
//  double temp;
//  temp = 0.0;

//  // coefficients
//  double a = 4.8216073551172471E+01;
//  double b = -5.2152243126025878E-02;
//  double c = 1.4976250154467562E-05;

//  temp = c;
//  temp = temp * zoom + b;
//  temp = temp * zoom + a;
//  return temp;
//}

//double THIS::angleToZoom ( double angle )
//{
//  double temp;
//  temp = 0.0;

//  // coefficients
//  double a = 4.8216073551172471E+01;
//  double b = -5.2152243126025878E-02;
//  double c = 1.4976250154467562E-05;

//  temp = - ( b / c ) / 2 - sqrt ( ( ( b * b ) / ( c * c ) ) / 4 - ( a - angle ) / c );
//  return temp;
//}


///*double THIS::factorToAngle ( double factor )
//{
// return 2* atan ( tan ( ( 3.1415/180.0 ) *zoomToAngle ( 40.0 )/2 ) /factor ) * ( 180/3.1415 );
//}

//double THIS::angleToFactor ( double arc )
//{
// return tan ( ( 3.1415/180.0 ) *zoomToAngle ( 40.0 ) /2 ) /tan ( ( 3.1415/180.0 ) *arc/2 );
//}*/

//bool THIS::setCameraZoomProperty ( ImageSources::SourceId sourceId, float factor, bool isAbsolute )
//{
//  if ( !isAbsolute )
//    factor *= Math::angleToPercent ( zoomToAngle ( getCameraProperty ( sourceId, "zoom" ) ), zoomToAngle ( 40.0 ) );

//  return setCameraProperty ( sourceId, "zoom", angleToZoom ( Math::percentToAngle ( 1.0 / factor, zoomToAngle ( 40.0 ) ) ), true );
//}


//void THIS::applySobel ( puma2::GrayLevelImage8* image )
//{
//  puma2::GrayLevelImage8::PixelType** row = image->unsafeRowPointerArray();
//  unsigned width = image->getWidth();
//  unsigned height = image->getHeight();

//  puma2::GrayLevelImage8* imageCopy = new puma2::GrayLevelImage8 ( width, height );
//  memcpy ( imageCopy->unsafeRowPointerArray() [0], image->unsafeRowPointerArray() [0], width * height );

//  puma2::GrayLevelImage8::PixelType** rowCopy = imageCopy->unsafeRowPointerArray();



//  for ( unsigned y = 1;y < height - 1;++y )
//    for ( unsigned x = 1;x < width - 1;++x )
//    {
//      int fx = 3 * rowCopy[y-1][x-1] - 3 * rowCopy[y-1][x+1] +
//               10 * rowCopy[y][x-1] - 10 * rowCopy[y][x+1] +
//               3 * rowCopy[y+1][x-1] - 3 * rowCopy[y+1][x+1];
//      int fy = 3 * rowCopy[y-1][x-1] + 10 * rowCopy[y-1][x] + 3 * rowCopy[y-1][x+1] +
//               -3 * rowCopy[y+1][x-1] - 10 * rowCopy[y+1][x] - 3 * rowCopy[y+1][x+1];
//      fx /= 16;
//      fy /= 16;

//      int strength = abs ( fx ) + abs ( fy );
//      if ( strength > 255 )
//        strength = 255;
//      else if ( strength < 0 )
//        strength = 0;

//      row[y][x] = strength;
//    }
//  delete rowCopy;
//}

//long THIS::measureSharpness ( puma2::GrayLevelImage8* image )
//{
//  puma2::GrayLevelImage8::PixelType** row = image->unsafeRowPointerArray();
//  unsigned width = image->getWidth();
//  unsigned height = image->getHeight();

//  unsigned int sharpness = 0;
//  for ( unsigned y = 1;y < height - 1;++y )
//    for ( unsigned x = 1;x < width - 1;++x )
//    {
//      int fx = 3 * row[y-1][x-1] - 3 * row[y-1][x+1] +
//               10 * row[y][x-1] - 10 * row[y][x+1] +
//               3 * row[y+1][x-1] - 3 * row[y+1][x+1];
//      int fy = 3 * row[y-1][x-1] + 10 * row[y-1][x] + 3 * row[y-1][x+1] +
//               -3 * row[y+1][x-1] - 10 * row[y+1][x] - 3 * row[y+1][x+1];
//      fx /= 4;
//      fy /= 4;

//      sharpness += abs ( fx ) + abs ( fy );
//    }
//  return sharpness;
//}


/*scale RGB images
    m_ResizeTimer.startMeasure();
    lowresImage=new ColorImageRGB8(image.getWidth()/2,image.getHeight()/2);

    //resize image
    ColorImageRGB8::PixelType** row = image.unsafeRowPointerArray();
    ColorImageRGB8::PixelType** lowresRow = lowresImage->unsafeRowPointerArray();
    int lowresHeight=lowresImage->getHeight();
    int lowresWidth=lowresImage->getWidth();

    //ptr. to current row in orig. image
    ColorImageRGB8::PixelType* currentRow;
    ColorImageRGB8::PixelType* currentLowresRow;
    int x,y,x2,y2;
    y2=0;
    for( y = 0; y < lowresHeight ; y++ ) {
      currentRow=row[y2];
      currentLowresRow=lowresRow[y];
      x2=0;
      for( x = 0; x < lowresWidth ; x++ ) {
        currentLowresRow[x][0]=currentRow[x2][0];
        currentLowresRow[x][1]=currentRow[x2][1];
        currentLowresRow[x][2]=currentRow[x2][2];
        x2+=2;
      }
      y2+=2;
    }
    m_ResizeTimer.pauseMeasure();
*/


#undef THIS

