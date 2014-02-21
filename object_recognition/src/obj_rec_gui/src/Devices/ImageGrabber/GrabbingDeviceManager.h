#ifndef GRABBING_DEVICE_MANAGER_H
#define GRABBING_DEVICE_MANAGER_H


// unicap bindings
extern "C"
{
  #include <unicap/unicap.h>
}

#include <string>
#include <vector>



namespace puma2{


  typedef std::vector<unicap_device_t*> device_ptr_vec;

  // forwards
  class GrabbingDevice;
  class PumaLogger;
  /**
   * \brief central instance managing all connected (unicap-)video devices
   *
   * DeviceManager provides access to all devices
   * supported by unicap, implemented as singleton
   * \author Benjamin Knopp <bknopp@uni-koblenz.de>
   */
  class GrabbingDeviceManager
  {
  public:

    /**
     * returns a pointer to the only valid instance of this Class
     */
    static GrabbingDeviceManager* getGrabbingDeviceManager();


    /**
     * connect _device with the GrabbingDevice specified by _id
     * \param _device _pointer to the GrabbingDevice
     * \param _id Id of the device, as shown by printDeviceList
     * \return true if successfully connected, false otherwise
     */
    bool connectGrabbingDevice(GrabbingDevice **_device, const uint _id);


    /**
     * connect _device with the GrabbingDevice specified by _id
     * \param _device _pointer to the GrabbingDevice
     * \param _description descriptor of the device, full name not required
     * \return true if successfully connected, false otherwise
     */
    bool connectGrabbingDevice(GrabbingDevice **_device, const std::string &_description);


    /**
     * \return returns the number of currently connected devices
     */
    uint getNumDevices();


    /**
     * returns some  information about the selected device
     * \param _id Id of the devie, as shown py printDeviceList()
     * \return string containing device description
     */
    std::string getDeviceInfo(const uint _id);


    /**
     * returns some  information about the selected device
     * \param _description description of the device
     * \return string containing device description
     */
    std::string getDeviceInfo(const std::string &_description);


    /**
     * scan for new devices, update database
     */
    bool scan();

    /**
     * get a list with all devices and additional information
     */
    std::string getDeviceList();


  private:
    GrabbingDeviceManager();
    GrabbingDeviceManager( const GrabbingDeviceManager&);
    ~GrabbingDeviceManager();


    bool findDeviceById(unicap_device_t **_device, const uint _id);
    bool findDeviceByDesc(unicap_device_t **_device, const std::string &_description);

    bool connectGrabbingDevice(GrabbingDevice **_device, unicap_device_t *_udevice);

    static GrabbingDeviceManager* mTheManager;
    device_ptr_vec mDevices;

    PumaLogger *mLogger;
  };





  }
#endif
