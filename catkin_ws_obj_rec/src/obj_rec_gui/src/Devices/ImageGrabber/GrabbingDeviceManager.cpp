#include <stdio.h>

#include "GrabbingDeviceManager.h"
#include "GrabbingDevice.h"
#include "GrabberUtils.h"

// #include "PumaLogger.h"
#include "../../Workers/Puma2/PumaException.h"


using namespace puma2;
using namespace std;


GrabbingDeviceManager* GrabbingDeviceManager::mTheManager =0;

GrabbingDeviceManager* GrabbingDeviceManager::getGrabbingDeviceManager()
{
  //check if an instance already exists
    if ( mTheManager == 0)
    {
      #ifdef DEBUG
      printf("%s Line[%d] : creating new instance \n", __FILE__, __LINE__);
      #endif
      mTheManager = new GrabbingDeviceManager();
    }
    return mTheManager;
}



bool GrabbingDeviceManager::connectGrabbingDevice(GrabbingDevice **_device, const uint _id)
{
  unicap_device_t *udevice;

  // search handle for device
  bool found = findDeviceById( &udevice, _id );
  if ( !found ) return false;

  // try to connect to device
  bool connected = connectGrabbingDevice( _device, udevice );
  return connected;
}



bool GrabbingDeviceManager::connectGrabbingDevice(GrabbingDevice **_device, const string &_description)
{
  unicap_device_t *udevice;

  // search handle for device
  bool found = findDeviceByDesc( &udevice, _description );
  if ( !found )
    {
      string message("GrabbingDeviceManager::connectGrabbingDevic nothing found");
      throw PumaException(PumaException::intolerable, message);
      return false;
    }

  // try to connect to device
  bool connected = connectGrabbingDevice( _device, udevice );
  return connected;
}




uint GrabbingDeviceManager::getNumDevices()
{
  return mDevices.size();
}




string GrabbingDeviceManager::getDeviceInfo(const uint _id)
{
  unicap_device_t *device;
  string result ="";

  // search handle for device
  bool found = findDeviceById( &device, _id );

  if ( !found ) return result ;

  result.append(device->identifier);

  return result;
}



string GrabbingDeviceManager::getDeviceInfo(const string &_description)
{
  unicap_device_t *device;
  string result ="";

  // search handle for device
  bool found = findDeviceByDesc( &device, _description );

    if ( !found ) return result ;

  result.append(device->identifier);

  return result;
}



string GrabbingDeviceManager::getDeviceList()
{
  char str[65535];
  ostringstream s;

  sprintf( str, "%5s|%50s|%20s|%20s|%20s|\n", "Index", "Identifier", "Model", "Vendor", "Serial");
  s << str;

  sprintf( str, "%5s|%50s|%20s|%20s|%20s|\n", "", "", "", "", "");
  s << str;

  for ( uint i = 0; i < mDevices.size(); i++ )
    {
        sprintf( str, "%5i|%50s|%20s|%20s|%20i|\n",
	       i,
	       mDevices.at(i)->identifier,
	       mDevices.at(i)->model_name,
	       mDevices.at(i)->vendor_name,
	       (int) mDevices.at(i)->model_id
	       );
        s << str;
    }
  return s.str();
}



GrabbingDeviceManager::GrabbingDeviceManager()
{
#ifdef HAVE_LOG4CPLUS
  mLogger = log4cplus::PumaLogger::getInstance();
#endif
  scan();
}



GrabbingDeviceManager::GrabbingDeviceManager( const GrabbingDeviceManager&)
{
  if ( mTheManager ) delete mTheManager;
#ifdef HAVE_LOG4CPLUS
  if ( mLogger ) delete mLogger;
#endif
}



GrabbingDeviceManager::~GrabbingDeviceManager()
{
  // clean up device vector
  for ( uint i = 0; i < mDevices.size(); i++)
    {
      delete mDevices.at(i);
    }
  mDevices.clear();

}



bool GrabbingDeviceManager::scan()
{
  bool result = true;
  int status = 1;

  //clean up
  for ( uint i = 0; i < mDevices.size(); i++)
    {
      delete mDevices.at(i);
    }
  mDevices.clear();

  for ( uint numDevices = 0; SUCCESS( status ); numDevices++ )
    {
      unicap_device_t *newDevice = new unicap_device_t();
      status = unicap_enumerate_devices( NULL, newDevice, numDevices);

      if ( SUCCESS( status ))
	{
	mDevices.push_back( newDevice );
	} else
	{
	  delete newDevice;
	  if ( numDevices == 0 ) result = false;
	  break;
	}
    }

#ifdef DEBUG
  printf("%s Line[%d] : found %i supported devices \n", __FILE__, __LINE__, getNumDevices());
#endif

  return result;
}



bool GrabbingDeviceManager::findDeviceById(unicap_device_t** _device, const uint _id)
{
  if ( _id > getNumDevices() )
    {
      return false;
    }

  *_device = mDevices.at(_id);

  return true;
}



bool GrabbingDeviceManager::findDeviceByDesc(unicap_device_t** _device, const string &_description)
{
  vector<string> words = splitString(_description, " ");


  string description;
  int position =0, targetId = -1;
  bool stringFound;

  for ( uint deviceId = 0; deviceId < mDevices.size(); deviceId++ )
    {
      description = mDevices.at( deviceId )->identifier;
      description.append( mDevices.at( deviceId )->model_name);
      description.append( mDevices.at( deviceId )->vendor_name);
      description.append( toString(mDevices.at( deviceId )->model_id));

      stringFound = true;

      //printf(" search string is %s \n", description.c_str());

      for ( uint i = 0; i < words.size(); i++ )
	{
	  position = description.find( words.at(i));
	  stringFound &= ( position > -1 );
	}
      if ( stringFound )
	{
	  targetId = deviceId;
	  break;
	}
    }

  if ( targetId == -1 )
    {
      printf("no device matching description %s was found\n", _description.c_str());
      return false;
    }
  *_device = mDevices.at( targetId );

  return true;
}



bool GrabbingDeviceManager::connectGrabbingDevice(GrabbingDevice **_device, unicap_device_t *_udevice)
{
  unicap_handle_t *handle = new unicap_handle_t();

  int status = unicap_open( handle, _udevice );
  if ( !SUCCESS( status ))
    {
      printf("could not connect to device\n");
      return false;
    }

  GrabbingDevice* newDevice = new GrabbingDevice( *handle );

  if ( newDevice == 0 ) return false;

  *_device = newDevice;
  return true;
}
