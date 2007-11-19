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
/// \file HapticMasterDevice.h
/// \brief Header file for HapticMasterDevice.
///
//
//////////////////////////////////////////////////////////////////////////////

#ifndef __HAPI_HAPTICMASTERDEVICE_H__
#define __HAPI_HAPTICMASTERDEVICE_H__

#include <HAPI/HAPIHapticsDevice.h>
#include <H3DUtil/DynamicLibrary.h>

#ifdef HAVE_HAPTIC_MASTER_API

namespace HAPI {

  /// Interface to the HapticMaster haptics device from Moog/FCS.
  class HAPI_API HapticMasterDevice: public HAPIHapticsDevice {
  public:
    /// Constructor.
    HapticMasterDevice( const string &_device_name = "vmd" ):
      device_handle( -1 ),
      device_name( _device_name ) {}

    /// Destructor.
    virtual ~HapticMasterDevice() {}

    /// Register this renderer to the haptics renderer database.
    static HapticsDeviceRegistration device_registration;
  
  protected:
    /// Get the device values(position, orientation, etc. )
    virtual void updateDeviceValues( DeviceValues &dv, 
                                     HAPITime dt );

    /// Send forces and torques to render
    virtual void sendOutput( DeviceOutput &dv,
                             HAPITime dt );

    /// Initialize the haptics device
    virtual bool initHapticsDevice();

    /// Releases all resources allocated in initHapticsDevice. 
    virtual bool releaseHapticsDevice();

  protected:
    // dll handle to the HapticMasterDriver.dll
    static H3DUtil::DynamicLibrary::LIBHANDLE dll_handle;
    // nr of references to the dll handle.
    static unsigned int dll_references;
    // the device handle for the haptics device.
    int device_handle;
    // the name of this device as specified in servers.db.
    string device_name;
    
  };
}

#endif

#endif
