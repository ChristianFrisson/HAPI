//////////////////////////////////////////////////////////////////////////////
//    Copyright 2010-2019, SenseGraphics AB
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
/// \file MLHIHapticsDevice.h
/// \brief Header file for MLHIHapticsDevice
/// Code contributed by Sebastian Bozlee and Jennifer McFatridge
/// University of Hawaii at Manoa Human-Robot Interface Laboratory and then
/// adapted to the HAPI code structure.
//
//////////////////////////////////////////////////////////////////////////////

#ifndef __MLHIHAPTICSDEVICE_H__
#define __MLHIHAPTICSDEVICE_H__

#include <HAPI/HAPIHapticsDevice.h>

#ifdef HAVE_MLHI

#include <MLHI/ml_api.h>
#include <pthread.h>

namespace HAPI {

  /// \ingroup HapticsDevices
  /// \class MLHIHapticsDevice
  /// \brief Interface to MLHI haptics devices.
  /// That is, devices from Butterfly haptics. Note that this
  /// is preliminary support
  /// 
  /// There are some things that are important to know about this
  /// implementation. The handle on the Butterfly Maglev 200 moves
  /// around freely prior to initialization. It is pretty easy to move
  /// it of range of the position sensors, which causes the device to fault.
  /// The device will not "take off" when there is a fault. For this reason a
  /// loop is introduced which is used to to delay initialization while
  /// continually clearing faults until the user moves the handle in range, no
  /// longer causing faults. Without this loop the user has to reinitialize
  /// the device over and over again until it works. This is pretty
  /// annoying keep readjusting the handle and rerunning the program until it
  /// works, which is pretty annoying.
  /// SenseGraphics have yet to test this device, and has therefore not tried
  /// to figure out a different way to handle this, nor correctly set errors
  /// for this device and cerr is used at the moment to report errors.
  class HAPI_API MLHIHapticsDevice : public HAPIHapticsDevice {
  public:
    /// Constructor
    MLHIHapticsDevice(string _server_ip_addr = "" )
      : device_hdl(NULL), current_position(Vec3(0,0,0)),
      current_rotation(Rotation()), velocity(Vec3(0,0,0)),
      server_ip_addr(_server_ip_addr)
    {
      // No need for mutex yet.
      for (int i = 0; i < 6; ++i)
        forces.values[i] = 0;
    }

    virtual ~MLHIHapticsDevice() {}

    /// Initializes haptics device.
    /// \param _thread_frequency ignored. Always 1000.
    bool initHapticsDevice(int _thread_frequency = 1000);

    /// Releases resources allocated by initHapticsDevice.
    bool releaseHapticsDevice();

    /// Turns rotation gains on (not allowed) or low (allowed).
    /// Being able to rotate makes the device less stable.
    void allowRotation(bool allow);

    /// Registers the device so that it may be used with the AnyHapticsClass
    static HapticsDeviceRegistration device_registration;

  protected:
    /// Updates device information for the rest of the program
    void updateDeviceValues(DeviceValues &dv, HAPITime dt);

    /// Sets forces to send to maglev device
    void sendOutput(DeviceOutput &dv, HAPITime dt);

  private:
    int connect(ml_device_handle_t * device_hdl);
    int checkFault(ml_device_handle_t device_hdl);

    // returns false on failure
    bool takeoffWhenReady();
    void defyGravity();
    // The waitForButtonPress() is needed to ensure that the user has a
    // grip on the handle prior to releasing the gains that keep the handle
    // levitating after takeoff. Otherwise, it drops.
    void waitForButtonPress();
    void initGains();
    void inline storeInDevices();
    void inline removeFromDevices();
    void inline registerCallbacks();
    void inline unregisterCallbacks();
    void renderForces(ml_position_t position);
    void updateButtonStatus(int _button_status);
    void updatePositionAndVelocity(ml_position_t position);

    static inline MLHIHapticsDevice* getDeviceByHandle(ml_device_handle_t device_hdl);

    ml_device_handle_t device_hdl;
    ml_forces_t forces;
    ml_gain_vec_t initial_gains;*/
    H3DUtil::MutexLock forces_lock;
    H3DUtil::MutexLock position_and_velocity_lock;
    Vec3 current_position;
    Rotation current_rotation;
    Vec3 velocity;
    int button_status;
    string server_ip_addr;

    static map<ml_device_handle_t, MLHIHapticsDevice*> devices;

    static int tickCallback(ml_device_handle_t device_hdl,
                            ml_position_t* position);
    static int checkFaultCallback(ml_device_handle_t device_hdl,
                                  ml_fault_t fault);
    static int buttonCallback(ml_device_handle_t device_hdl,
                              ml_button_t button);
  };

}

#endif

#endif
