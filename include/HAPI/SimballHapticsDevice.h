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
/// \file SimballHapticsDevice.h
/// \brief Header file for SimballHapticsDevice.
///
//
//////////////////////////////////////////////////////////////////////////////

#ifndef __SIMBALLHAPTICSDEVICE_H__
#define __SIMBALLHAPTICSDEVICE_H__

#include <HAPI/HAPIHapticsDevice.h>

#ifdef HAVE_SIMBALLMEDICAL_API

namespace HAPI {

  /// \ingroup HapticsDevices
  /// \class SimballHapticsDevice
  /// \brief Interface to all devices that are connected via the SimballMedical
  /// inteface. For more information about which devices that are supported by
  /// this see http://www.g-coder.com/
  /// Please note that these devices does not have haptics feedback and as such
  /// are not really haptics devices but can be seen as tracking devices.
  class HAPI_API SimballHapticsDevice: public HAPIHapticsDevice {
  public:
    /// Constructor.
    SimballHapticsDevice( int _device_nr = 0 ) {
      device_nr = 0;
    }

    /// Destructor.
    virtual ~SimballHapticsDevice() {}

    /// Enable the device. Positions can be read and force can be sent.
    inline virtual ErrorCode enableDevice() {
      return HAPIHapticsDevice::enableDevice();
    }

    /// Get the angle of the handle of the connected device. Can be used
    /// to change the appearance of the stylus.
    inline double getHandleAngle() {
      return handle_angle;
    }

    /// Register this device to the haptics device database.
    static HapticsDeviceRegistration device_registration;

  protected:

    /// Implementation of updateDeviceValues.
    virtual void updateDeviceValues( DeviceValues &dv, HAPITime dt );

    /// Implementation of sendOutput.
    virtual void sendOutput( DeviceOutput &dv,
                             HAPITime dt );

    /// Implementation of initHapticsDevice
    virtual bool initHapticsDevice( int _thread_frequency = 1000 );

    /// Releases all resources allocated in initHapticsDevice. 
    virtual bool releaseHapticsDevice();

    /// The number of found devices on the system
    static int nr_found_devices;
    static int nr_initialized_devices;

    /// The number of the device for this instance, can be seen as a device
    /// handle;
    int device_nr;

    /// The angle of the handle of the SimballHapticsDevice.
    HAPIFloat handle_angle;
  };
}

#endif // HAVE_SIMBALLMEDICAL_API

#endif
