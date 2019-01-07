//////////////////////////////////////////////////////////////////////////////
//    Copyright 2004-2019, SenseGraphics AB
//
//    This file is part of HAPI.
//
//    HAPI is free software; you can redistribute it and/or modify
//    it under the terms of the GNU General Public License as published by
//    the Free Software Foundation; either version 2 of the License, or
//    (at your option) any later version.
//
//    HAPI is distributed in the hope that it will be useful,
//    but WITHOUT ANY WARRANTY; without even the implied warranty of
//    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//    GNU General Public License for more details.
//
//    You should have received a copy of the GNU General Public License
//    along with HAPI; if not, write to the Free Software
//    Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
//
//    A commercial license is also available. Please contact us at 
//    www.sensegraphics.com for more information.
//
//
/// \file EntactHapticsDevice.h
/// \brief Header file for EntactHapticsDevice.
///
//
//////////////////////////////////////////////////////////////////////////////

#ifndef __ENTACTHAPTICSDEVICE_H__
#define __ENTACTHAPTICSDEVICE_H__

#include <HAPI/HAPIHapticsDevice.h>

#ifdef HAVE_ENTACTAPI
#include <EntactAPI.h>

namespace HAPI {

  /// \ingroup HapticsDevices
  /// \class EntactHapticsDevice
  /// \brief Interface to all haptics devices from Entact.
  /// Note that all EntactHapticsDevices in one application have to
  /// use the same way to connect. Either serial number for all, or ip
  /// address for all.
  class HAPI_API EntactHapticsDevice: public HAPIHapticsDevice {
  public:
    /// Constructor.
    /// \param _serial_number The serial number is the serial number of
    /// the Entact device to use. Only the Entact device with the given
    /// serial number will be initialized. If -1 any available Entact device
    /// will be used. All EntactHapticsDevices in one application have to
    /// use the same way to connect. Either serial number for all, or ip
    /// address for all.
    /// \param _ip_address The ip address of the device to initialize. Only used if not empty
    /// and serial number is -1.
    EntactHapticsDevice( int _serial_number = -1,
                         std::string _ip_address = "" );

    /// Destructor.
    virtual ~EntactHapticsDevice();

    /// Returns the Entact device id for this device. -1 of not
    /// initialized.
    inline int getDeviceId() { 
      return device_id;
    }

    /// Returns the serial number of the device.
    inline int getSerialNumber() {
      return serial_number;
    }
    
    /// Returns true if the device needs to be calibrated.
    bool needsCalibration();

    /// Calibrate the device. The calibration procedure depends on
    /// the device type. Returns true if the calibration sequence is
    /// started.
    bool calibrateDevice();

    /// Returns the number of Entact devices that are currently
    /// connected to the computer.
    static int getNumberConnectedEntactDevices();

    /// Register this renderer to the haptics renderer database.
    static HapticsDeviceRegistration device_registration;
  protected:
    /// Implementation of updateDeviceValues using DHD API to get the values.
    virtual void updateDeviceValues( DeviceValues &dv, HAPITime dt );

    /// Implementation of sendOutput using DHD API to send forces.
    virtual void sendOutput( DeviceOutput &dv,
                             HAPITime dt );

    /// Initialize the haptics device. Use the HapticThread class in Threads.h
    /// as the thread for haptic rendering.
    /// \param _thread_frequency is the desired haptic frequency. 
    /// 1000 is the maximum allowed frequency that can be specified. Setting
    /// this parameter to -1 means run as fast as possible. It is recommended
    /// to use the default value for most users.
    virtual bool initHapticsDevice( int _thread_frequency = 1000 );

    /// Releases all resources allocated in initHapticsDevice. 
    virtual bool releaseHapticsDevice();

    /// The EntactAPI device id for this device.
    int device_id;

    /// Callback function for communication thread
    static H3DUtil::PeriodicThread::CallbackCode com_func( void *data );

    /// Callback handle to the com_func callback that is set up
    int com_func_cb_handle;

    /// Thread used to do communication with the haptics device
    H3DUtil::PeriodicThread *com_thread;

    /// Lock for exchanging data with the communication thread.
    H3DUtil::MutexLock com_lock;

    /// The current device values updated in the communicataion thread.
    /// Access to this structure must be contained within locking with 
    /// com_lock.
    DeviceValues current_values;
    
    /// The serial number of the device
    int serial_number;

    /// The ip adress of the device. Only used if serial number is -1.
    std::string ip_address;

    /// The nr of EntactHapticsDevice class instances that are currently
    /// in use.
    static unsigned int nr_device_instances;

    /// The number of entact devices connected to the computer. Only 
    /// valid if EAPI_initialized is not UNINITIALIZED.
    static int nr_entact_devices;

    typedef enum {
      ENTACT_UNINITIALIZED = 0,
      ENTACT_SERIAL_NUMBER = 1,
      ENTACT_IP_ADDRESS = 2
    } EAPIInitializedStyle;

    /// Flag to indicate of the EAPI used to control Entact devices
    /// has been initialized and in what way since only one way is allowed.
    static EAPIInitializedStyle EAPI_initialized;

  };
}

#endif

#endif
