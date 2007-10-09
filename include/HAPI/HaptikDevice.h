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
/// \file HaptikDevice.h
/// \brief Header file for HaptikDevice.
///
//
//////////////////////////////////////////////////////////////////////////////

#ifndef __HAPI_HAPTIKDEVICE_H__
#define __HAPI_HAPTIKDEVICE_H__

#include <HAPI/HAPIHapticsDevice.h>

#ifdef HAVE_HAPTIK_LIBRARY
#include <RSLib/Haptik.hpp>

namespace HAPI {

  /// \class HaptikDevice
  /// Base class for all haptic devices. 
  class HAPI_API HaptikDevice: public HAPIHapticsDevice {
  public:

    /// Constructor.
    HaptikDevice(): 
      haptik_device( NULL ) {
      changeHaptikDevice( HAPTIK_DEFAULT_DEVICE );
    }

    /// Destructor. Stops haptics rendering and remove callback functions.
    virtual ~HaptikDevice() {
      disableDevice();
      releaseDevice();
    }

    /// Does all the initialization needed for the device before starting to
    /// use it.
    virtual void initDevice();

    virtual void enableDevice();

    virtual void disableDevice();

    /// Perform cleanup and let go of all device resources that are allocated.
    /// After a call to this function no haptic rendering can be performed on
    /// the device until the initDevice() function has been called again.
    virtual void releaseDevice();

    virtual Vec3 getPosition();

    /// Get the velocity of the haptics device. Only to be called in the 
    /// haptics loop.
    virtual Vec3 getVelocity();

    /// Get the orientation of the haptics device. Only to be called in the 
    /// haptics loop.
    virtual Rotation getOrientation();

    /// Returns true if main button on haptics device pressed. Only to be called in the 
    /// haptics loop.
    inline bool getButtonStatus( unsigned int button_nr );

    virtual HAPIInt32 getButtonStatus();

    void changeHaptikDevice( UINT32 device_id );

  protected:

    /// Send the force to render on the haptics device. Only to be called in the 
    /// haptics loop.
    virtual void sendForceToDevice( const Vec3 &f );

    /// Send the torque to render on the haptics device. Only to be called in the 
    /// haptics loop.
    virtual void sendTorqueToDevice( const Vec3 &f );

    RSLib::Haptik haptik;
    /// The device currently in use.
    RSLib::IHaptikDeviceInterface haptik_device;
  };
}

#endif
#endif
