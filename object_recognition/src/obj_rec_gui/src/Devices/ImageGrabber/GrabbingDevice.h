#ifndef GRABBING_DEVICE_H
#define GRABBING_DEVICE_H


// unicap bindings
extern "C"
{
#include <unicap/unicap.h>
#include <unicap/ucil.h>
}

#include <string>
#include "GrabberUtils.h"
// check ..
#include <sys/types.h>
#include <linux/types.h>


namespace puma2
{

  // ************ FORWARDS ********************************
  class Image;
  class ColorImageRGBa8;
  class ColorImageRGB8;

  // ************ TYPEDEFS ********************************
  typedef std::vector<unicap_format_t*> format_ptr_vec;
  typedef std::vector<unicap_property_t*> property_ptr_vec;
  typedef std::vector<unicap_data_buffer_t*> data_buffer_ptr_vec;

  union property_value_t
  {
    char menuItem[128];
    double value;
    void* data;
    u_int64_t flags;
  };

  /**
   * @brief represents a grabbing-device supported by unicap
   * @author Benjamin Knopp <bknopp@uni-koblenz.de>, David Gossow (R12)
   */
  class GrabbingDevice
  {
    public:

      /**
       * only valid constructor of this class
       * @param _handle a unicap handle, identifying the connected device
       * required for internal communication
       */
      GrabbingDevice ( unicap_handle_t _handle );

      ~GrabbingDevice();

      //*********** DEVICE SETUP *****************************

      /**
       * @brief set format according to image size and FOURCC color code
       * @return true is format has been successfully set
       */
      bool setFormat ( int width, int height, std::string fourcc );

      /**
       * @brief set format according to format ID
       * @return true is format has been successfully set
       */
      bool setFormat ( const uint _formatId, const uint _subformatId );

      /// @brief set number of buffers to allocate for the queue
      bool setNumBuffers ( uint _numBuffers );

      /**
       * @brief set string value of a property
       * @param _name name of the property e.g.: "trigger"
       * @param _value value off the property e.g.: "free running"
       * @return true if property was successfully set
       */
      bool setProperty ( const std::string &_name, const std::string &_value );

      /**
       * @brief set numeric value of a property
       * @param _name name of the property e.g.: "brightness"
       * @param _value value off the property e.g.: "0.5"
       * @param _normalized if normalized, range is between 0.0 and 1.0, otherwise device specific values are used
       * @return true if property was successfully set
       */
      bool setProperty ( const std::string &_name, double &_value, bool _normalized = true );

      /// @brief directly set unicap property
      bool setProperty ( unicap_property_t &_property );


      //*********** CAPTURING **********************************

      /**
       * @brief starts the capturing process
       * @return returns true if process could be started
       */
      bool startCapture();

      /// @brief stops the capturing process
      bool stopCapture();

      /**
       * convenience function for grabbing images
       * frames are converted to puma2 rgb-images, colorspace conversion must be available
       * @param _image pointer to the target rgb image
       * @retrun true if capture process was successful
       */
      bool grabImage ( ColorImageRGB8 &_image );

      /**
       * @brief grab raw image data
       * @param _data byte stream containing image information
       * @param fourcc FOURCC code of desired output color space
       */
      bool grabData ( unsigned char  *_data, std::string fourcc );

      /**
       * @brief Re-enqueue all but newest buffer. Should be called in short intervals
       * @brief (at least the framerate of the fastest camera connected) so the queue never gets full.
       */
      void cleanBuffers();

      //*********** DEVICE INFO *****************************

      /// @return the model name of this device
      std::string getModelName() { return std::string( m_Device.model_name ); }

      /// @return the vendor name of this device
      std::string getVendorName() { return std::string( m_Device.vendor_name ); }

      /// @return the identifier (unique) of this device
      std::string getIdentifier() { return std::string( m_Device.identifier ); }

      /// @return the device path (/dev/videoX)
      std::string getDevice() { return std::string( m_Device.device ); }

      /// @return the bus type
      std::string getBusType() { return std::string( m_Device.cpi_layer ); }

      /// @return the serial number of this device
      unsigned long long getSerialNumber() { return m_Device.model_id; }

      uint getNumBuffers();

      //*********** FORMATS INFO *********************************

      /// @return a textual list with all supported formats
      std::string getFormatsList();

      /// @return a textual description of the current color format
      std::string getFormatDesc();

      /// @returns bits per pixel of the current input format
      uint getFormatDepth();

      /// @return frame width of the current format
      uint getFrameWidth();

      /// @return frame height of the current format
      uint getFrameHeight();

      /// @return frame size in bytes (width * height * bpp) of the current format
      uint getFrameSize();


      //*********** PROPERTIES *****************************

      /// @brief print informations about supported properties to std::out
      std::string getPropertiesList();

      /// @brief print more detailed information about property _desc
      std::string getPropertyInfo( const std::string &_desc );

      bool getProperty ( unicap_property_t **_property, uint _id );

      bool getProperty ( unicap_property_t **_property, std::string &_desc );

    private:

      bool setOutputColorspace ( std::string fourcc );

      /// @see above
      bool setOutputColorspace ( uint _fourcc );

      /// @brief set format by providing a unicap format description
      bool setFormat ( unicap_format_t* _format );

      /**
       * find unicap format by id
       */
      bool findFormatById ( unicap_format_t** _format, const uint _formatId, const uint _subformatId );

      /**
       * find unicap property by id
       */
      bool findPropertyById ( unicap_property_t** _property, const uint _id );

      /// @return a description of the format selected by _formatId
      std::string getFormatDesc ( const uint _formatId );

      /**
       * find unicap property by description
       */
      bool findPropertyByDesc ( unicap_property_t** _property, const std::string &_desc );

      bool setProperty ( const std::string &_name, property_value_t &_value, bool _normalized = true );

      /**
       * prepare buffers for capturing, set size, enqueue etc..
       */
      bool prepareBuffers();

      /**
       * update list of supported formats
       */
      bool detectFormats();

      /**
       * update list of supported properties, depends on selected format !
       */
      bool detectProperties();

      unicap_handle_t m_Handle;
      unicap_device_t m_Device;

      unsigned int m_TargetFourCC;
      unicap_data_buffer_t m_ConvertedBuffer;

      // store formats & properties infos
      format_ptr_vec m_Formats;
      property_ptr_vec m_Properties;

      // one elemt of m_Format
      unicap_format_t* m_CurrentFormat;

      // ******* buffers **************
      uint m_NumBuffers;
      data_buffer_ptr_vec m_Buffers;
  };


}

#endif
