//////////////////////////////////////////////////////////////////////////////
//    Copyright 2004-2007, SenseGraphics AB
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
  /// \brief The AnyHapticsDevice node uses the first connected haptics
  /// device that it can successfully initialize.
  ///
  /// This can be used if you do not care which haptics device to
  /// use, just any one connected to your computer. It will try all 
  /// registered haptics devices such as PhantomHapticsDevice and 
  /// ForceDimensionHapticsDevice until it finds one that works.
  class HAPI_API AnyHapticsDevice: public HAPIHapticsDevice {
  public:
    /// Constructor.
    AnyHapticsDevice(){}

    /// Destructor.
    virtual ~AnyHapticsDevice() {}

    /// Returns the HD device handle for this device.
    inline HAPIHapticsDevice *getActualHapticsDevice() { 
      return hd.get();
    }

    /// Enable the device. Positions can be read and force can be sent.
    inline virtual ErrorCode enableDevice() {
      ErrorCode e = HAPIHapticsDevice::enableDevice();
      if(hd.get() ) hd->enableDevice();
      return e;
    }

    /// Disable the device. 
    inline virtual ErrorCode disableDevice() {
      ErrorCode e = HAPIHapticsDevice::disableDevice();
      if(hd.get() ) hd->disableDevice();
      return e;
    }

    /// Register this renderer to the haptics renderer database.
    static HapticsDeviceRegistration device_registration;
  protected:
    /// Implementation of updateDeviceValues using HD API to get the values.
    virtual void updateDeviceValues( DeviceValues &dv, HAPITime dt ) {
      if( hd.get() ) hd->updateDeviceValues( dv, dt );
    }

    /// Implementation of sendOutput using HD API to send forces.
    virtual void sendOutput( DeviceOutput &dv,
                             HAPITime dt ) {
      if( hd.get() ) hd->sendOutput( dv, dt );
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


    /// The haptics device actually used.
    auto_ptr< HAPIHapticsDevice > hd;

  };
}

#endif
