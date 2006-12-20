//////////////////////////////////////////////////////////////////////////////
//    Copyright 2004, SenseGraphics AB
//
//    This file is part of H3D API.
//
//    H3D API is free software; you can redistribute it and/or modify
//    it under the terms of the GNU General Public License as published by
//    the Free Software Foundation; either version 2 of the License, or
//    (at your option) any later version.
//
//    H3D API is distributed in the hope that it will be useful,
//    but WITHOUT ANY WARRANTY; without even the implied warranty of
//    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//    GNU General Public License for more details.
//
//    You should have received a copy of the GNU General Public License
//    along with H3D API; if not, write to the Free Software
//    Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
//
//    A commercial license is also available. Please contact us at 
//    www.sensegraphics.com for more information.
//
//
/// \file AnyHapticsDevice.h
/// \brief Header file for AnyHapticsDevice.
///
//
//////////////////////////////////////////////////////////////////////////////

#ifndef __ANYHAPTICSDEVICE_H__
#define __ANYHAPTICSDEVICE_H__

#include <HAPIHapticsDevice.h>

namespace HAPI {

  /// \class AnyHapticsDevice
  /// \brief Interface to all Any haptics devices from SensAble Technologies.
  /// Uses the HD API of OpenHaptics to interface to the haptics devices, 
  /// so every device supported by OpenHaptics is supported by this class.
  class HAPI_API AnyHapticsDevice: public HAPIHapticsDevice {
  public:
    /// Constructor.
    /// device_name is the name of the device, as defined in the 
    /// "Any Configuration" tool. A device_name of "" will use the first
    /// available device.
    AnyHapticsDevice(){}

    /// Destructor.
    virtual ~AnyHapticsDevice() {}

    /// Returns the HD device handle for this device.
    inline HAPIHapticsDevice *getActualHapticsDevice() { 
      return hd.get();
    }

    /// Register this renderer to the haptics renderer database.
    static HapticsDeviceRegistration device_registration;
  protected:
    /// Implementation of updateDeviceValues using HD API to get the values.
    virtual void updateDeviceValues( DeviceValues &dv, HAPITime dt ) {
      if( hd.get() ) return hd->updateDeviceValues( dv, dt );
    }

    /// Implementation of sendOutput using HD API to send forces.
    virtual void sendOutput( DeviceOutput &dv,
                             HAPITime dt ) {
      if( hd.get() ) return hd->sendOutput( dv, dt );
    }

    /// Implementation of initHapticsDevice using HD API.
    virtual bool initHapticsDevice();

    /// Releases all resources allocated in initHapticsDevice. 
    virtual bool releaseHapticsDevice() {
      if( hd.get() ) {
        bool b = hd->releaseHapticsDevice();
        hd.reset( NULL );
        thread = NULL;
        return b;
      }
      return true;
    }

    /// Enable the device. Positions can be read and force can be sent.
    inline virtual ErrorCode enableDevice() {
      ErrorCode e = HAPIHapticsDevice::enableDevice();
      if(hd.get() ) hd->enableDevice();
      return e;
    }

    /// Enable the device. Positions can be read and force can be sent.
    inline virtual ErrorCode disableDevice() {
      ErrorCode e = HAPIHapticsDevice::disableDevice();
      if(hd.get() ) hd->disableDevice();
      return e;
    }

    /// The haptics device actually used.
    auto_ptr< HAPIHapticsDevice > hd;

  };
}

#endif
