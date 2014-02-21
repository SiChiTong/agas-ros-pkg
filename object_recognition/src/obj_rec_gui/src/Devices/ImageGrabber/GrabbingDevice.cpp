/*******************************************************************************
*  GrabbingDevice.cpp
*
*  (C) 2009 AG Aktives Sehen <agas@uni-koblenz.de>
*           Universitaet Koblenz-Landau
******************************************************************************/


#include "GrabbingDevice.h"

//#include "Architecture/Serializer/ImageCompressor.h"

// PUMA LIBS
#include "../../Workers/Puma2/ColorImageRGB8.h"
#include "../../Workers/Puma2/PumaException.h"

// STL
#include <algorithm>
#include <iostream>
#include <fstream>
#include <sstream>

#include <cstring>
#include <string>

#include <stdio.h>

using namespace puma2;
using namespace std;

//#define DEBUG

#define MAX_NUM_BUFFERS 64


GrabbingDevice::GrabbingDevice ( unicap_handle_t _handle )
{
  m_Handle = _handle;
  unicap_get_device ( m_Handle, &m_Device );

  m_TargetFourCC  = UCIL_FOURCC ( 'R', 'G', 'B', '3' );
  m_NumBuffers = 4;

  detectFormats();
  detectProperties();

#ifdef DEBUG
  printf ( "%s Line[%d] : creating new device with handle: %li \n",
           __FILE__, __LINE__, ( long int ) m_Handle );
#endif
}



GrabbingDevice::~GrabbingDevice()
{
//  TRACE_SYSTEMINFO( "Stopping Capture process.." )
//  unicap_stop_capture ( m_Handle );
//  TRACE_SYSTEMINFO( "Closing device.." )
//  unicap_close ( m_Handle );

//  TRACE_SYSTEMINFO( "Deleting buffers.." )
//// free all buffers
//  for ( uint i = 0; i < m_Buffers.size(); i++ )
//  {
//    free ( m_Buffers.at ( i )->data );
//    delete m_Buffers.at ( i );
//  }

//  TRACE_SYSTEMINFO( "Freeing formats memory.." )
//  for ( unsigned int i = 0; i < m_Formats.size(); i++ )
//  {
//    delete m_Formats[i];
//  }

//  TRACE_SYSTEMINFO( "Freeing properties memory.." )
//  for ( unsigned int i = 0; i < m_Properties.size(); i++ )
//  {
//    delete m_Properties[i];
//  }

}


bool GrabbingDevice::setFormat ( const uint _formatId, const uint _subformatId )
{
  bool result = false;

//  TRACE_INFO( "Setting format " + Tracer::toString( _formatId ) + "." + Tracer::toString( _subformatId ) );

//  // check if some parameters are out of range
//  if ( _formatId > m_Formats.size() ) {
//    TRACE_ERROR( "Invalid format ID" );
//    return false;
//  }
//  if ( ( int ) _subformatId > m_Formats.at ( _formatId )->size_count )
//  {
//    TRACE_ERROR( "Invalid subformat ID" );
//    return false;
//  }

  // if selected format provides multiple resolutions
  if ( m_Formats[_formatId]->size_count != 0 )
  {
    m_Formats[_formatId]->size.width = m_Formats[_formatId]->sizes[_subformatId].width;
    m_Formats[_formatId]->size.height = m_Formats[_formatId]->sizes[_subformatId].height;
  }
  result = setFormat ( m_Formats[_formatId] );
  return result;
}


bool GrabbingDevice::setFormat ( int width, int height, std::string fourcc )
{
  if ( fourcc.size() != 4 )
  {
    string str( "Invalid fourcc" );
    throw PumaException( PumaException::ignorable, str );
  }
  unsigned int ucil_fourcc = UCIL_FOURCC( fourcc[0], fourcc[1], fourcc[2], fourcc[3] );

  for ( uint formatId = 0; formatId < m_Formats.size(); formatId++)
  {
    if ( m_Formats.at(formatId)->size_count == 0 )
    {
      // there are no sub formats, check main format
      if ( ( m_Formats.at(formatId)->size.width == width ) &&
           ( m_Formats.at(formatId)->size.height == height ) &&
           ( m_Formats.at(formatId)->fourcc == ucil_fourcc ))
      {
        return setFormat( formatId, 0 );
      }
    }
    else
    {
      //look if there is a fitting sub format
      for ( int subFormatId = 0; subFormatId < m_Formats.at(formatId)->size_count; subFormatId++ )
      {
        if ( ( m_Formats.at(formatId)->sizes[ subFormatId ].width == width ) &&
               ( m_Formats.at(formatId)->sizes[ subFormatId ].height == height ) &&
               ( m_Formats.at(formatId)->fourcc == ucil_fourcc ) )
        {
          //select subformat size
          m_Formats[formatId]->size.width = m_Formats[formatId]->sizes[subFormatId].width;
          m_Formats[formatId]->size.height = m_Formats[formatId]->sizes[subFormatId].height;
          return setFormat( formatId, subFormatId );
        }
      }
    }
  }

  //TRACE_ERROR( "No format matching " + toString( width ) + "x" + toString( height ) + " / " + fourcc + " found!" );
  return false;
}


bool GrabbingDevice::setFormat ( unicap_format_t* _format )
{
  bool result = true;

  _format->buffer_type = UNICAP_BUFFER_TYPE_USER;

//  TRACE_INFO( "Setting format " + Tracer::toString( _format->size.width ) + "x" + Tracer::toString( _format->size.height ) );

//  if ( SUCCESS ( unicap_set_format ( m_Handle, _format ) ) )
//  {
//    m_CurrentFormat = _format;
//    if ( m_CurrentFormat->fourcc == UCIL_FOURCC('M','J','P','G') )
//    {
//      m_CurrentFormat->buffer_size = _format->size.width*_format->size.height*4+1024;
//      TRACE_INFO( "MJPEG format selected. Setting buffer size to " + Tracer::toString( m_CurrentFormat->buffer_size ) + " bytes." );
//    }
//  }
//  else
//  {
//    string message ( "Unable to set format!" );
//    throw  PumaException ( PumaException::intolerable, message );
//    result = false;
//  }

  // buffer size may have changed
  prepareBuffers();

  // properties may have changed
  result &= detectProperties();

  return result;
}


string GrabbingDevice::getFormatDesc()
{
  string result ( m_CurrentFormat->identifier );
  return result;
}



string GrabbingDevice::getFormatDesc ( const uint _formatId )
{
  if ( _formatId >= m_Formats.size() )
  {
    string message ( "format index out of range" );
    throw  PumaException ( PumaException::ignorable, message );
    string result ( "unknown" );
    return result;
  }

  string result ( m_Formats.at ( _formatId )->identifier );
  return result;
}



uint GrabbingDevice::getFormatDepth()
{
  if ( m_CurrentFormat == 0 ) return 0;
  return ( uint ) m_CurrentFormat->bpp;
}


uint GrabbingDevice::getFrameWidth()
{
  if ( m_CurrentFormat == 0 ) return 0;
  return ( uint ) m_CurrentFormat->size.width;
}

uint GrabbingDevice::getFrameHeight()
{
  if ( m_CurrentFormat == 0 ) return 0;
  return ( uint ) m_CurrentFormat->size.height;
}

uint GrabbingDevice::getFrameSize()
{
  //if ( m_CurrentFormat == 0) return 0;
  return ( m_ConvertedBuffer.format.size.width *
           m_ConvertedBuffer.format.size.height *
           m_ConvertedBuffer.format.bpp / 8 );
}

bool GrabbingDevice::setNumBuffers ( uint _numBuffers )
{
  if ( ( _numBuffers < 2 ) || ( _numBuffers > MAX_NUM_BUFFERS ) ) return false;
  m_NumBuffers = _numBuffers;
  if ( m_CurrentFormat )
  {
    return prepareBuffers();
  }
  else
  {
    return true;
  }
}


uint GrabbingDevice::getNumBuffers()
{
  return m_NumBuffers;
}


bool GrabbingDevice::setProperty ( const string &_name, const string &_value )
{
  property_value_t val;
  strcpy ( val.menuItem, _value.c_str() );
  return setProperty ( _name, val, false );
}



bool GrabbingDevice::setProperty ( const string &_name, double &_value, bool _normalized )
{
  property_value_t val;
  val.value = _value;
  return setProperty ( _name, val, _normalized );
}


bool GrabbingDevice::setProperty ( unicap_property_t &_property )
{
  if ( ! SUCCESS ( unicap_set_property ( m_Handle, &_property ) ) )
  {
    string message ( "unable to set property" );
    throw  PumaException ( PumaException::ignorable, message );
    return false;
  }
  return true;
}



bool GrabbingDevice::getProperty ( unicap_property_t **_property, uint _id )
{
  return findPropertyById ( _property, _id );
}



bool GrabbingDevice::getProperty ( unicap_property_t **_property, string &_desc )
{
  return findPropertyByDesc ( _property, _desc );
}


void GrabbingDevice::cleanBuffers()
{
  int numBuffersReady = 0;
  unicap_data_buffer_t *capturedBuffer = 0;

  // get number of filled buffers
//  if ( ! SUCCESS ( unicap_poll_buffer ( m_Handle, &numBuffersReady ) ) )
//  {
//    numBuffersReady = 1;
//    TRACE_WARNING("Could not poll unicap buffer. Setting buffer count to 1.");
//  }

  // free already filled and outdatet buffers
  for ( int i = 0; i < numBuffersReady; i++ )
  {
    unicap_wait_buffer ( m_Handle, &capturedBuffer );
    unicap_queue_buffer ( m_Handle, capturedBuffer );
  }
}


bool GrabbingDevice::setOutputColorspace ( string fourcc )
{
  if ( fourcc.size() != 4 )
  {
    string str( "Invalid fourcc" );
    throw PumaException( PumaException::ignorable, str );
  }
  return setOutputColorspace( UCIL_FOURCC( fourcc[0], fourcc[1], fourcc[2], fourcc[3] ) );
}

bool GrabbingDevice::setOutputColorspace ( uint _fourcc )
{
  int conversion_status = ucil_conversion_supported ( _fourcc,  m_CurrentFormat->fourcc );

  if ( conversion_status == 0 )
  {
    printf ( "%s Line[%d] : colorspace conversion from %s to %s NOT! supported \n",
             __FILE__,
             __LINE__,
             toString ( _fourcc ).c_str(), toString ( m_CurrentFormat->fourcc ).c_str() );

    string message ( "colorspace with fourcc" );
    message += toString ( _fourcc );
    message += " could not be selected";
    throw  PumaException ( PumaException::ignorable, message );

    return false;
  }

#ifdef DEBUG
  printf ( "%s Line[%d] : colorspace conversion from %s to %s supported  \n",
           __FILE__, __LINE__,  toString ( _fourcc ).c_str(), toString ( m_CurrentFormat->fourcc ).c_str() );
#endif

  // save fourcc code
  m_TargetFourCC = _fourcc;

  // copy format info
  unicap_copy_format ( &m_ConvertedBuffer.format, m_CurrentFormat );

  // apply changes
  m_ConvertedBuffer.format.fourcc = m_TargetFourCC;

  switch ( ucil_get_colorspace_from_fourcc ( m_TargetFourCC ) )
  {
    case UCIL_COLORSPACE_RGB24:
      m_ConvertedBuffer.format.bpp = 24;
      break;

    case UCIL_COLORSPACE_RGB32:
      m_ConvertedBuffer.format.bpp = 32;
      break;

    case UCIL_COLORSPACE_Y8:
      m_ConvertedBuffer.format.bpp = 8;
      break;

    case UCIL_COLORSPACE_YUV:
      m_ConvertedBuffer.format.bpp = 16;
      break;

    case UCIL_COLORSPACE_UNKNOWN:
      printf ( "Colorspace unknown!\n" );
      m_ConvertedBuffer.format.bpp = 32; // should be safe ..
  }

  return true;
}


bool GrabbingDevice::grabImage ( ColorImageRGB8 &_image )
{
  unicap_data_buffer_t *capturedBuffer = 0;
  unicap_data_buffer_t destBuffer;

  cleanBuffers();

  // get current buffer
  unicap_status_t gotBuffer = unicap_wait_buffer ( m_Handle, &capturedBuffer );

  // if current buffer could not be fetched
  if ( !SUCCESS ( gotBuffer ) )
  {
    printf ( "failed to wait for buffer\n" );
    unicap_queue_buffer ( m_Handle, capturedBuffer );
    return false;
  }

  // adapt image to current format
  _image.resize ( m_CurrentFormat->size.width, m_CurrentFormat->size.height );

  if ( m_CurrentFormat->fourcc == UCIL_FOURCC('M','J','P','G') )
  {
    //convert MJPEG 'by hand'
/*    if (capturedBuffer->frame_number == 0)
    {
      //always skip first frame because it contains invalid data (reason unknown)
      unicap_queue_buffer ( m_Handle, capturedBuffer );
      return true;
    }*/
//    TRACE_SYSTEMINFO( "Decompressing MJPEG of size " + Tracer::toString( m_CurrentFormat->size.width ) + "x" + Tracer::toString( m_CurrentFormat->size.height ) );

//    try {
//      ImageCompressor::decompress( capturedBuffer->buffer_size, capturedBuffer->data, _image );
//    }
//    catch ( const char* msg ) {
//      TRACE_ERROR( msg );
//    }

    unicap_queue_buffer ( m_Handle, capturedBuffer );
  /*
    unsigned char tmp1[ capturedBuffer-> ];
    int bytesRead=copyBuffer(tmp1);
    Messages::ImageM* msg=new Messages::ImageM(srcId,camSrc.width,camSrc.height,tmp1,bytesRead);
    bool ret=Worker::MJPG::decompressJPG( tmp1,bytesRead,(unsigned char*)msg->getImage().getPixels());
    if (!ret)
    {
      return false;
    }
  */
  }
  else
  {
    //let UCIL convert the buffer
    ColorImageRGB8::PixelType **ptr = _image.unsafeRowPointerArray();

    memset ( &destBuffer, 0x0, sizeof ( unicap_data_buffer_t ) );

    // adapt destination buffer
    unicap_copy_format ( &destBuffer.format, m_CurrentFormat );
    destBuffer.format.fourcc = UCIL_FOURCC ( 'R', 'G', 'B', '3' );
    destBuffer.buffer_size = m_CurrentFormat->size.width *  m_CurrentFormat->size.height * 3;
    destBuffer.data = ( unsigned char* ) ptr[0];

    unicap_status_t converted = ucil_convert_buffer ( &destBuffer, capturedBuffer );

    // put buffer back to queue
    unicap_queue_buffer ( m_Handle, capturedBuffer );

    if ( !SUCCESS( converted ) )
    {
      printf ( "color conversion failed\n" );
      return false;
    }
  }

  return true;
}



bool GrabbingDevice::grabData ( unsigned char *_data, std::string fourcc )
{
/*
  if ( m_CurrentFormat->fourcc == UCIL_FOURCC('M','J','P','G') )
  {
    if ( fourcc != "UYVY" )
    {
      return false;
    }
    
    ColorImageRGB8 image;
    if ( !grabImage ( image ) )
    {
      return false;
    }
    unsigned char *rgbData = image.unsafeRowPointerArray()[0];
    return true;
  }
*/
  
  bool result = false;
  unicap_data_buffer_t *capturedBuffer = 0;

  cleanBuffers();

  unicap_status_t gotBuffer = unicap_wait_buffer ( m_Handle, &capturedBuffer );

  if ( !SUCCESS ( gotBuffer ) )
  {
    printf ( "failed to wait for buffer\n" );
    unicap_queue_buffer ( m_Handle, capturedBuffer );
    return result;
  }

  setOutputColorspace( fourcc );

  // set location for converted data
  m_ConvertedBuffer.data = _data;

  // conversion required, will be supported, otherwise it would not have been set
  unicap_status_t converted = ucil_convert_buffer ( &m_ConvertedBuffer, capturedBuffer );

  // put buffer back to queue
  unicap_queue_buffer ( m_Handle, capturedBuffer );

  if ( !SUCCESS( converted ) )
  {
    printf ( "color conversion failed\n" );
    return false;
  }
  result = true;

  return result;
}


bool GrabbingDevice::startCapture()
{
//  if(!SUCCESS(unicap_start_capture(m_Handle))) {
//      TRACE_ERROR("Failed to start unicap capture");
//  }

  unicap_status_t status;
  bool result = true;

  for ( uint i = 0; i < m_Buffers.size(); i++ )
  {
    status = unicap_queue_buffer ( m_Handle, m_Buffers.at ( i ) );

//    if(!SUCCESS(status)) {
//        TRACE_WARNING("Failed to queue unicap buffer #" << i);
//    }
    result &= SUCCESS ( status );
  }

  return result;
}



bool GrabbingDevice::stopCapture()
{
  unicap_status_t status;
  bool result;
  status = unicap_stop_capture ( m_Handle );
  result = SUCCESS ( status );
  return result;
}



bool GrabbingDevice::findFormatById ( unicap_format_t** _format, const uint _formatId, const uint _subformatId )
{
  if ( ( _formatId > ( uint ) m_Formats.size() ) || ( _subformatId > ( uint ) m_Formats.at ( _formatId )->size_count ) )
    return false;

  m_Formats.at ( _formatId )->size.width = m_Formats.at ( _formatId )->sizes[_subformatId].width;
  m_Formats.at ( _formatId )->size.height = m_Formats.at ( _formatId )->sizes[_subformatId].height;

  *_format = m_Formats.at ( _formatId );
  return true;
}



bool GrabbingDevice::findPropertyById ( unicap_property_t** _property, const uint _id )
{
  if ( _id > m_Properties.size() ) return false;
  *_property = m_Properties.at ( _id );
  return true;
}



bool GrabbingDevice::findPropertyByDesc ( unicap_property_t** _property, const string &_desc )
{
  unicap_property_t* property = new unicap_property_t();
  string propname ( _desc );

  strncpy ( property->identifier, propname.c_str(), propname.length() );

  unicap_status_t status = unicap_get_property ( m_Handle, property );
  if ( ! SUCCESS ( status ) )
  {
    printf ( "sorry, property %s could not be found\n", propname.c_str() );
    return false;
  }
  *_property = property;
  return true;
}



bool GrabbingDevice::setProperty ( const string &_name, property_value_t &_value, bool _normalized )
{
  unicap_property_t* property;
  bool result = findPropertyByDesc ( &property, _name );

  if ( !result ) return false;

  double value = 0, min = 0, max = 0, tmp = 0;
  bool found = false;

  switch ( property->type )
  {

    case UNICAP_PROPERTY_TYPE_RANGE:
      if ( _normalized )
      {
        min = property->range.min;
        max = property->range.max;
        value = _value.value * ( max - min ) + min;
      }
      else
      {
        value = _value.value;
      }
      property->value = value;
      break;

    case UNICAP_PROPERTY_TYPE_VALUE_LIST:
      if ( _normalized )
      {
        min = property->range.min;
        max = property->range.max;
        tmp = _value.value * ( max - min ) + min;
      }
      else
      {
        tmp = _value.value;
      }

      for ( int i = 0; ( ( i < property->value_list.value_count ) &&
                         ( property->value_list.values[i] <= tmp ) ); i++ )
      {
        value = property->value_list.values[i];
      }
      property->value = value;
      break;

    case UNICAP_PROPERTY_TYPE_MENU:

      for ( int i = 0; ( ( i < property->menu.menu_item_count ) && !found ); i++ )
      {
        if ( strcmp ( property->menu.menu_items[i], _value.menuItem ) == 0 )
        {
          found = true;
          strcpy ( property->menu_item, property->menu.menu_items[i] );
        }
      }
      if ( !found ) return false;
      break;

    case UNICAP_PROPERTY_TYPE_DATA:
      printf ( "UNICAP_PROPERTY_TYPE_DATA:sorry,not implemented yet\n" );
      return false;
      break;

    case UNICAP_PROPERTY_TYPE_FLAGS:
      printf ( "UNICAP_PROPERTY_TYPE_FLAGS:sorry,not implemented yet\n" );
      return false;
      break;


    case UNICAP_PROPERTY_TYPE_UNKNOWN:
      printf ( "unknown property type, dont know what to do\n" );
      return false;
      break;
  }
  // call function to set property
  return setProperty ( *property );
}



bool GrabbingDevice::prepareBuffers()
{
  //TRACE_INFO( "Allocating "+toString( m_NumBuffers ) + " buffers of size " + toString( m_CurrentFormat->buffer_size ) );

  stopCapture();

  if ( ! ( m_CurrentFormat ) )
  {
    return false;
  }

  // delete previous data
  for ( uint i = 0; i < m_Buffers.size(); i++ )
  {
    free ( m_Buffers.at ( i )->data );
    delete m_Buffers.at ( i );
  }
  m_Buffers.clear();

  for ( uint numBuffers  = 0; numBuffers < m_NumBuffers; numBuffers++ )
  {
    #ifdef DEBUG
    printf ( "%s Line[%d] : (re)allocating memory for buffer %i  \n", __FILE__, __LINE__, numBuffers );
    #endif
    unicap_data_buffer_t *newBuffer = new unicap_data_buffer_t();
    newBuffer->data = ( byte* ) malloc ( m_CurrentFormat->buffer_size );
    memset ( newBuffer->data, 0x0, m_CurrentFormat->buffer_size );
    newBuffer->buffer_size = m_CurrentFormat->buffer_size;

    m_Buffers.push_back ( newBuffer );
  }

  return true;
}



bool GrabbingDevice::detectFormats()
{
  // delete previous data
  for ( uint i = 0; i < m_Formats.size(); i++ )
  {
    delete m_Formats.at ( i );
  }
  m_Formats.clear();

  unicap_status_t status = STATUS_SUCCESS;

  for ( uint numFormats = 0; SUCCESS ( status ); numFormats++ )
  {
    unicap_format_t* newFormat = new unicap_format_t();
    status = unicap_enumerate_formats ( m_Handle, NULL, newFormat, numFormats );

//     if ( newFormat->fourcc == UCIL_FOURCC( 'M', 'J', 'P', 'G' ) )
//     {
//       int w = newFormat->size.width;
//       int h = newFormat->size.height;
//       newFormat->buffer_size = w*h*3 + 65535;
//       newFormat->bpp=24;
//     }

    if ( SUCCESS ( status ) )
    {
      m_Formats.push_back ( newFormat );
    }
    else
    {
      delete newFormat;
    }
  }

  if (m_Formats.size() == 0)
//    TRACE_ERROR("Failed to get video format");

#ifdef DEBUG
  printf ( "%s Line[%d] : %i supported formats detected! \n", __FILE__, __LINE__, ( int ) m_Formats.size() );
#endif

  detectProperties();
  return true;
}



bool GrabbingDevice::detectProperties()
{
  // delete previous data
  for ( uint i = 0; i < m_Properties.size(); i++ )
  {
    delete m_Properties.at ( i );
  }
  m_Properties.clear();

  unicap_status_t status = STATUS_SUCCESS;

  for ( uint numProperties = 0; SUCCESS ( status ); numProperties++ )
  {
    unicap_property_t* newProperty = new unicap_property_t();
    status = unicap_enumerate_properties ( m_Handle, NULL, newProperty, numProperties );

    if ( SUCCESS ( status ) )
    {
      m_Properties.push_back ( newProperty );
    }
    else
    {
      delete newProperty;
    }
  }

#ifdef DEBUG
  printf ( "%s Line[%d] : %i supported properties detected! \n",
           __FILE__, __LINE__, ( int ) m_Properties.size() );
#endif

  return true;
}




string GrabbingDevice::getFormatsList()
{
  int subformat = 0;

  ostringstream s;
  char str[65535];

  char blank[] = " ";
  sprintf ( str, "\n%5s | %25s | %17s | %15s | \n", "Index", "Identifier", "Resolution", "BitPerPixel" );
  s << str;
  sprintf ( str, "%5s | %25s | %17s | %15s | \n", blank, blank, blank, blank );
  s << str;

  for ( uint format = 0;  format < m_Formats.size(); format++ )
  {
    subformat = 0;

    if ( m_Formats.at ( format )->size_count <= 0 )
    {
      sprintf ( str, " %2i.%i | %25s | %10i x %4i | %15i | \n",
                format,
                subformat,
                m_Formats.at ( format )->identifier,
                               m_Formats.at ( format )->size.width,
                                              m_Formats.at ( format )->size.height,
                                                  m_Formats.at ( format )->bpp );
      s << str;
    }

    for ( subformat = 0; subformat < m_Formats.at ( format )->size_count; subformat++ )
    {
      sprintf ( str, " %2i.%i | %25s | %10i x %4i | %15i | \n",
                format,
                subformat,
                m_Formats.at ( format )->identifier,
                               m_Formats.at ( format )->sizes[subformat].width,
                                              m_Formats.at ( format )->sizes[subformat].height,
                                                  m_Formats.at ( format )->bpp );
      s << str;
    }
  }

  return s.str();
}


string GrabbingDevice::getPropertiesList()
{
  const string modes[] = { "range", "value list", "menu", "data", "flags" };

  ostringstream s;
  char str[65535];

  sprintf ( str, "\n%5s|%25s|%25s|%10s|%15s|\n", "index", "identifier", "category", "relations", "type" );
  s << str;
  sprintf ( str, "%5s|%25s|%25s|%10s|%15s|\n", "", "", "", "", "" );
  s << str;

  for ( uint pid = 0; pid < m_Properties.size(); pid++ )
  {
    sprintf ( str, "%5i|%25s|%25s|%10i|%15s|\n",
              pid,
              m_Properties.at ( pid )->identifier,
                                m_Properties.at ( pid )->category,
                                    m_Properties.at ( pid )->relations_count,
                                        modes[m_Properties.at ( pid )->type].c_str() );
    s << str;
  }

  return s.str();
}


string GrabbingDevice::getPropertyInfo ( const string &_desc )
{
  unicap_property_t* property;

  bool result = findPropertyByDesc ( &property, _desc );
  if ( !result ) return "";

  ostringstream s;

  char* propertyName = property->identifier;
  s << "property name : " << propertyName << endl;

  switch ( property->type )
  {
    case UNICAP_PROPERTY_TYPE_RANGE:
      s << "property range: " << property->range.min << " - " << property->range.max << endl;
      s << " current value: " << property->value;
      break;

    case UNICAP_PROPERTY_TYPE_VALUE_LIST:
      s << "valid values are: ";
      for ( int i = 0; i < property->value_list.value_count; i++ )
      {
        if ( i > 0 ) {
          s << ", ";
        }
        s << property->value_list.values[i];
      }
      s << endl;
      s << " current value: " << property->value;
      break;

    case UNICAP_PROPERTY_TYPE_MENU:
      s << "valid values are: ";
      for ( int i = 0; i < property->value_list.value_count; i++ )
      {
        if ( i > 0 ) {
          s << ", ";
        }
        s << property->menu.menu_items[i];
      }
      s << endl;
      s << " current value: " << property->menu_item;
      break;

    case UNICAP_PROPERTY_TYPE_DATA:
    case UNICAP_PROPERTY_TYPE_FLAGS:
    case UNICAP_PROPERTY_TYPE_UNKNOWN:
    default:
      s << "Value type unsupported.";
      break;
  }

  return s.str();
}

