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
/// \file DHDHapticsDevice.h
/// \brief Header file for DHDHapticsDevice.
///
//
//////////////////////////////////////////////////////////////////////////////

#ifndef __DHDHAPTICSDEVICE_H__
#define __DHDHAPTICSDEVICE_H__

#include <HAPIHapticsDevice.h>

#ifdef HAVE_DHDAPI

namespace HAPI {

  /// \class DHDHapticsDevice
  /// \brief Interface to all haptics devices from ForceDimension, including the 
  /// Delta and Omega devices.
  /// Uses the DHD API to interface to the haptics devices, so every
  /// device supported by DHD API is supported by this class.
  class HAPI_API DHDHapticsDevice: public HAPIHapticsDevice {
  public:
    /// Constructor.
    DHDHapticsDevice():
      device_id( -1 ) {
    }

    /// Destructor.
    virtual ~DHDHapticsDevice() {}

    /// Returns the DHD API device id for this device.
    inline int getDeviceId() { 
      return device_id;
    }

    /// Returns the device type of the Force Dimension device.
    /// Possible values are:
    /// - DHD_DEVICE_3DOF - the Delta Haptic Device 3DOF
    /// - DHD_DEVICE_6DOF - the Delta Haptic Device 6DOF
    /// - DHD_DEVICE_OMEGA - the OMEGA Haptic Device
    /// - DHD_DEVICE_OMEGA3, DHD_DEVICE_OMEGA33, DHD_DEVICE_OMEGA331 - the 
    /// second generation OMEGA.X Haptic Devices
    /// - DHD_DEVICE_CONTROLLER - the Force Dimension stand-alone USB 2.0
    ///  controller (DHD_DEVICE_CONTROLLER) 
    /// - DHD_DEVICE_SIMULATOR - the Force Dimension haptic device
    /// simulator
    /// - DHD_DEVICE_CUSTOM - Unknown devices.
    int getDeviceType();

    /// Puts the device in RESET mode. In this mode, the user is expected
    /// to put the device end-effector at its rest position. This is how
    /// the device performs its calibration. 
    void reset();

    /// Puts the device in RESET mode and wait for the user to calibrate
    /// the device.  Optionally, a timeout can be defined after which the 
    /// call returns even if calibration has not occured.
    void waitForReset( int timeout = 0 );

    /// Enable/disable gravity compensation. A value of true enables it.
    /// When gravity compensation is enabled, the weights of the arms and of
    /// the end-effector are taken into account and a vertical force is 
    /// dynamically applied to the end-effector on top of the user command.
    /// By default, gravity compensation is enabled
    void useGravityCompensation( bool b );

    /// Define the mass of the end-effector in order
    /// to provide accurate gravity compensation when custom-made or 
    /// modified end-effectors are used.
    void setEffectorMass( double mass );
    
    /// Enable/disable the device electromagnetic brakes. If enabled
    /// the device motor circuits are shortcut to produce electromagnetic
    /// viscosity. The viscosity is sufficient to prevent the device from
    /// falling too hard onto if forces are disabled abruptly, either by
    /// pressing the force button or by action of a safety feature.
    void useBrakes( bool enable );

    /// Register this renderer to the haptics renderer database.
    static HapticsDeviceRegistration device_registration;
  protected:
    /// Implementation of updateDeviceValues using HD API to get the values.
    virtual void updateDeviceValues( DeviceValues &dv, HAPITime dt );

    /// Implementation of sendOutput using HD API to send forces.
    virtual void sendOutput( DeviceOutput &dv,
                             HAPITime dt );

    /// Implementation of initHapticsDevice using HD API.
    virtual bool initHapticsDevice();

    /// Releases all resources allocated in initHapticsDevice. 
    virtual bool releaseHapticsDevice();

    /// The DHD API device id for this device.
    int device_id;
  };
}

#endif

#endif
