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
/// \file AnyHapticsDevice.h
/// \brief Header file for AnyHapticsDevice.
///
//
//////////////////////////////////////////////////////////////////////////////

#ifndef __ANYHAPTICSDEVICE_H__
#define __ANYHAPTICSDEVICE_H__

#include <HAPI/HAPIHapticsDevice.h>

namespace HAPI {

  /// \ingroup HapticsDevices
  /// \class AnyHapticsDevice
  /// \brief This class represents any haptics device connected to the system.
  ///
  /// The class uses the first connected haptics device that it can
  /// successfully initialize. This can be used if you do not care which
  /// haptics device to use, just any one connected to your computer. It will
  /// try all registered haptics devices such as PhantomHapticsDevice and
  /// ForceDimensionHapticsDevice until it finds one that works.
  class HAPI_API AnyHapticsDevice: public HAPIHapticsDevice {
  public:
    /// Constructor.
    AnyHapticsDevice(){}

    /// Destructor.
    virtual ~AnyHapticsDevice() {}

    /// Returns a pointer to the device that is actually used.
    inline HAPIHapticsDevice *getActualHapticsDevice() { 
      return hd.get();
    }

    /// Enable the device. Positions can be read and force can be sent.
    inline virtual ErrorCode enableDevice() {
      if( hd.get() ) {
        ErrorCode e = hd->enableDevice();
        device_state = hd->getDeviceState();
        return e;
      } else {
        return NOT_INITIALIZED;
      }
    }

    /// Disable the device. 
    inline virtual ErrorCode disableDevice() {
      if( hd.get() ) {
        ErrorCode e = hd->disableDevice();
        device_state = hd->getDeviceState();
        return e;
      } else {
        return NOT_INITIALIZED;
      }
    }

    virtual void setErrorHandler( ErrorHandler *_error_handler ) {
      error_handler.reset( _error_handler );
      if( hd.get() ) hd->setErrorHandler( _error_handler );
    }

    /// Register this renderer to the haptics renderer database.
    static HapticsDeviceRegistration device_registration;
  protected:
    /// Implementation of updateDeviceValues using the contained device
    /// to get the values.
    /// \param dv Contains values that should be updated.
    /// \param dt Time since last call to this function.
    virtual void updateDeviceValues( DeviceValues &dv, HAPITime dt ) {
      if( hd.get() ) {
        hd->updateDeviceValues( dv, dt );
        // Needed to correctly calculate device velocity for devices that
        // base this on current_raw_device_values, for example
        // FalconHapticsDevice
        hd->current_raw_device_values = last_raw_device_values;
      }
    }

    /// Implementation of sendOutput, calling sendOutput of the contained
    /// device.
    /// \param dv Contains force values to send to the haptics device.
    /// \param dt Time since last call to this function.
    virtual void sendOutput( DeviceOutput &dv,
                             HAPITime dt ) {
      if( hd.get() ) {
        hd->output.force = output.force;
        hd->output.torque = output.torque;
        hd->output.dof7_force = output.dof7_force;
        hd->sendOutput( dv, dt );
      }
    }

    /// Calls initHapticsDevice of the contained device.
    virtual bool initHapticsDevice( int _thread_frequency = 1000 );

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
    std::auto_ptr< HAPIHapticsDevice > hd;

  };
}

#endif
