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
/// \file ForceDimensionHapticsDevice.h
/// \brief Header file for ForceDimensionHapticsDevice.
///
//
//////////////////////////////////////////////////////////////////////////////

#ifndef __FORCEDIMENSIONHAPTICSDEVICE_H__
#define __FORCEDIMENSIONHAPTICSDEVICE_H__

#include <HAPI/HAPIHapticsDevice.h>

#ifdef HAVE_DHDAPI

namespace HAPI {

  /// \ingroup HapticsDevices
  /// \class ForceDimensionHapticsDevice
  /// \brief Interface to all haptics devices from ForceDimension, including
  /// the Delta and Omega devices.
  ///
  /// Uses the DHD API to interface to the haptics devices, so every
  /// device supported by DHD API is supported by this class.
  class HAPI_API ForceDimensionHapticsDevice: public HAPIHapticsDevice {
  public:
    /// Constructor.
    ForceDimensionHapticsDevice();

    /// Destructor.
    virtual ~ForceDimensionHapticsDevice() {}

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
    /// the device. Optionally, a timeout can be defined after which the 
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
    /// \returns true if succeeded.
    virtual bool initHapticsDevice( int _thread_frequency = 1000 );

    /// Releases all resources allocated in initHapticsDevice. 
    virtual bool releaseHapticsDevice();

    /// The DHD API device id for this device.
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

    /// The number of force dimension devices connected to the system.
    /// Needed because dhdOpen() does not work for multiple devices.
    /// dhdOpenID( char id ) have to be used.
    static int nr_of_connected_dhd_devices;

    /// Ids of devices that can be initialized. Needed because dhdOpen
    /// does not work for multiple devices. dhdOpenID have to be used.
    static std::vector< int > free_dhd_ids;
  public:
    /// Enable/disable the device forces. Useful if the device has no
    /// button to control this.
    void enableForce( bool enable );

    Rotation (*rotation_func)( double &rx, double &ry, double &rz );

    void setVibration( const HAPIFloat &frequency, const HAPIFloat &amplitude );

  protected:
    // Write access to these variables should only be done when locking with com_lock.
    // The main reason for specifying them twice is simply to do obtain them in the same way
    // as other values of the device are obtained.
    // They are used to get the gripper angle;
    bool is_autocalibrated, is_autocalibrated_com_thread;

  public:
    Vec3 (*angular_velocity_func)( double &rx, double &ry, double &rz );

    /// Run auto calibration procedure.
    /// Will only do something if HAVE_DRDAPI is defined.
    bool autoCalibrate();

    inline bool isAutoCalibrated() {
      return is_autocalibrated;
    }

    inline void setComThreadFrequency( const HAPIInt32 &_com_thread_frequency ) {
      com_thread_frequency = _com_thread_frequency;
    }

  protected:
    HAPIInt32 com_thread_frequency;
    bool auto_calibration_mode;
    bool has_gripper_support;
  };
}

#endif

#endif
