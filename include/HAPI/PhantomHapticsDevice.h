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
/// \file PhantomHapticsDevice.h
/// \brief Header file for PhantomHapticsDevice.
///
//
//////////////////////////////////////////////////////////////////////////////

#ifndef __PHANTOMHAPTICSDEVICE_H__
#define __PHANTOMHAPTICSDEVICE_H__

#include <HAPI/HAPIHapticsDevice.h>
#ifdef HAVE_OPENHAPTICS

#include <HD/hd.h>

namespace HAPI {

  /// Interface to all Phantom haptics devices from SensAble Technologies.
  /// Uses the HD API of OpenHaptics to interface to the haptics devices,
  /// so every device supported by OpenHaptics is supported by this class.
  class HAPI_API PhantomHapticsDevice: public HAPIHapticsDevice {
  public:
    /// Constructor.
    /// device_name is the name of the device, as defined in the 
    /// "Phantom Configuration" tool. A device_name of "" will use the first
    /// available device.
    PhantomHapticsDevice( string _device_name = "" ):
        device_name( _device_name ) {
      hdapi_version = hdGetString( HD_VERSION );
      setup_haptic_rendering_callback = false;
    }

    /// Destructor.
    virtual ~PhantomHapticsDevice() {}


    /// Returns the HD device handle for this device.
    inline HHD getDeviceHandle() { 
      return device_handle;
    }

    /// Returns the name of the device.
    virtual const string &getDeviceName() {
      return device_name;
    }

    /// Register this device to the haptics device database.
    static HapticsDeviceRegistration device_registration;

    /// Get the firmware version for the device. Undefined behavior
    /// if device not initialized.
    inline double getDeviceFirmwareVersion() { 
      return device_firmware_version; 
    }

    /// Get the HDAPI software version, in the form major.minor.build
    inline string getHDAPIVersion() { return hdapi_version; }

    /// Get the device model of the device. Undefined if device not
    /// initialized.
    inline string getDeviceModelType() { return device_model_type; }

    /// Get the device driver version of the device. Undefined if device not
    /// initialized.
    inline string getDeviceDriverVersion() { return device_driver_version; }

    /// Get the vendor of the device. Undefined if device not
    /// initialized.
    inline string getDeviceVendor() { return device_vendor; }

    /// Get the serial number of the device. Undefined if device not
    /// initialized.
    inline string getDeviceSerialNumber() { return device_serial_number; }
   
    /// \brief Get the maximum workspace dimensions of the device, i.e. the
    /// mechanical limits of the device. Undefined if
    /// device not initialized.
    inline void getMaxWorkspaceDimensions( Vec3 &min, Vec3&max ) {
      min = max_workspace_min;
      max = max_workspace_max;
    }
      
    /// \brief Get the usable workspace dimensions of the device, i.e. the 
    /// workspace in which forces are guaranteed to be reliably render.
    /// Undefined if device not initialized.
    inline void getUsableWorkspaceDimensions( Vec3 &min, Vec3&max ) {
      min = usable_workspace_min;
      max = usable_workspace_max;
    }

    /// Get the mechanical offset of the device end-effector in y from
    /// the table top. Undefined if device not initialized.
    inline HAPIFloat getTabletopOffset() { return tabletop_offset; }
    
    /// \brief Get the maximum force, i.e. the amount of force that the
    /// device can sustain when the motors are at room temperature
    /// Undefined if device not initialized.
    inline HAPIFloat getMaxForce() { return max_force; }

    /// \brief Get the maximum continuous force, i.e. the amount of force that the
    /// device can sustain through a period of time.
    inline HAPIFloat getMaxContinuousForce() { return max_cont_force; }

    /// Get the number of input degrees of freedom.
    /// Undefined if device not initialized.
    inline int getInputDOF() { return input_dof; }

    /// Get the number of output degrees of freedom.
    /// Undefined if device not initialized.
    inline int getOutputDOF() { return output_dof; }

    /// Get current joint angles for the device (in radians).
    /// x - turret, + = left
    /// y - thigh,  + = up
    /// z - shin,   + = up
    inline Vec3 getJointAngles() { return joint_angles; }

    /// Get current gimbal angles for the device (in radians).
    /// From neutral position:
    /// x - Right     = +
    /// y - Up        = -
    /// z - Clockwise = +
    inline Vec3 getGimbalAngles() { return gimbal_angles; }
    
    /// Returns true if the device needs to be calibrated.
    bool needsCalibration();

    /// Calibrate the device. The calibration procedure depends on
    /// the device type.
    bool calibrateDevice();

    /// Enable the device. Positions can be read and force can be sent.
    inline virtual ErrorCode enableDevice() {
      ErrorCode e = HAPIHapticsDevice::enableDevice();
      // Starting scheduler here instead of in initHapticsDevice since because
      // of random crashes when using OpenHapticsRenderer. Apparantly HL API does
      // not like the scheduler to have already been started when creating a new
      // HL context.
      if( enable_start_scheduler ) {
        startScheduler();
      }
      return e;
    }

    // set the enable_start_scheduler variable.
    static inline void setEnableStartScheduler( bool new_value ) {
      enable_start_scheduler = new_value;
    }

    // start the hdScheduler.
    static inline void startScheduler() {
      hdStartScheduler();
      scheduler_started = true;
    }

  protected:
    double device_firmware_version;
    string hdapi_version;
    string device_model_type;
    string device_driver_version;
    string device_vendor;
    string device_serial_number;
    Vec3 usable_workspace_max;
    Vec3 usable_workspace_min;
    Vec3 max_workspace_max;
    Vec3 max_workspace_min;
    HAPIFloat max_force, max_cont_force, tabletop_offset;
    int input_dof, output_dof;
    Vec3 joint_angles, gimbal_angles;
    
    /// Implementation of updateDeviceValues using HD API to get the values.
    virtual void updateDeviceValues( DeviceValues &dv, HAPITime dt );

    /// Implementation of sendOutput using HD API to send forces.
    virtual void sendOutput( DeviceOutput &dv,
                             HAPITime dt );

    /// Implementation of initHapticsDevice using HD API.
    /// PCI and EPP support 500, 1000, and 2000 Hz.
    /// Firewire supports 500, 1000, 1600 Hz, plus some increments in between
    /// based on the following expression: floor(8000/N + 0.5).
    /// The first successful initialization of PhantomHapticsDevice will decide
    /// the haptics thread rate since only one scheduler is used by
    /// OpenHaptics even for dual device configurations.
    virtual bool initHapticsDevice( int _thread_frequency = 1000 );

    /// Releases all resources allocated in initHapticsDevice. 
    virtual bool releaseHapticsDevice();
    
    static HDCallbackCode HDCALLBACK endFrameCallback( void *data );

    /// The device name for this device.
    string device_name;
    /// The OpenHaptics device handle for this device.
    HHD device_handle;

    /// Handle for the callback for rendering ForceEffects.  
    vector< HDCallbackCode > hd_handles;

    // If true the scheduler will be started when enableDevice is called.
    // If not then the startScheduler() function has to be called in order
    // to have haptics rendering function. Default value is true.
    static bool enable_start_scheduler;

    // True if the OpenHaptics scheduler is started.
    static bool scheduler_started;

    /// Counts the number of PhantomHapticsDevices for which the scheduler
    /// should be running and the HLThread should be active. Used in
    /// releaseHapticsDevice to know if the OpenHaptics threading should be
    /// shut off.
    static int nr_of_scheduled;
  };
}

#endif

#endif
