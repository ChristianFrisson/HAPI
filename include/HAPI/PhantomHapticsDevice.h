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

  /// \ingroup HapticsDevices
  /// \class PhantomHapticsDevice
  /// \brief Interface to all Phantom haptics devices from SensAble
  /// Technologies.
  ///
  /// Uses the HD API of OpenHaptics to interface to the haptics devices,
  /// so every device supported by OpenHaptics is supported by this class.
  class HAPI_API PhantomHapticsDevice: public HAPIHapticsDevice {
  public:
    /// Constructor.
    /// \param _device_name is the name of the device, as defined in the 
    /// "Phantom Configuration" tool. A device_name of "" will use the first
    /// available device.
    PhantomHapticsDevice( std::string _device_name = "" ):
      device_name( _device_name ),
      in_calibration_mode( false ),
      in_inkwell( false ),
      motor_temperatures( 6, 0 ),
      encoder_values( 6, 0 ),
      device_firmware_version( 0.0 ),
      device_handle( -1 ),
      input_dof( -1 ),
      output_dof( -1 ) {
      hdapi_version = "NOT INITIALIZED YET";
      setup_haptic_rendering_callback = false;
    }

    /// Destructor.
    virtual ~PhantomHapticsDevice() {}


    /// Returns the HD device handle for this device.
    inline HHD getDeviceHandle() { 
      return device_handle;
    }

    /// Returns the name of the device.
    virtual const std::string &getDeviceName() {
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
    inline std::string getHDAPIVersion() { return hdapi_version; }

    /// Get the device model of the device. Undefined if device not
    /// initialized.
    inline std::string getDeviceModelType() { return device_model_type; }

    /// Get the device driver version of the device. Undefined if device not
    /// initialized.
    inline std::string getDeviceDriverVersion() { return device_driver_version; }

    /// Get the vendor of the device. Undefined if device not
    /// initialized.
    inline std::string getDeviceVendor() { return device_vendor; }

    /// Get the serial number of the device. Undefined if device not
    /// initialized.
    inline std::string getDeviceSerialNumber() { return device_serial_number; }

    /// Get the motor temperatures from the temperature model of the device.
    /// The returned vector contains 6 values, where the first three are the
    /// motors for positional control and the last three are for torques(if available).
    /// The values are normalized between 0 and 1 where 1 means that a temperature
    /// error will be generated and the device temporarily be shut down by the driver
    /// a few seconds to cool down.
    inline const std::vector< HAPIFloat > &getMotorTemperatures(){ return motor_temperatures; }

    /// Get the encoder values
    /// The returned vector contains 6 values, where the first three are
    /// corresponding with the joint angles and the last three gimbal angles.
    inline const std::vector< HAPIFloat > &getEncoderValues(){ return encoder_values; }
   
    /// Returns true if the device is in the inkwell
    inline bool isInInkwell() { return in_inkwell; }
    
    /// \brief Get the maximum workspace dimensions of the device, i.e. the
    /// mechanical limits of the device. Undefined if
    /// device not initialized.
    /// \param min The minimum values of the workspace dimensions.
    /// \param max The maximum values of the workspace dimensions.
    inline void getMaxWorkspaceDimensions( Vec3 &min, Vec3&max ) {
      min = max_workspace_min;
      max = max_workspace_max;
    }
      
    /// \brief Get the usable workspace dimensions of the device, i.e. the 
    /// workspace in which forces are guaranteed to be reliably render.
    /// Undefined if device not initialized.
    /// \param min The minimum values of the usable workspace dimensions.
    /// \param max The maximum values of the usable workspace dimensions.
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
    /// the device type. Returns true if the calibration sequence is
    /// started.
    bool calibrateDevice();

    /// Check if the device is in a state where all the forces
    /// are disabled by hardware. For example, the handle is in
    /// the inkwell of an omni or the safety switch is on for a Phantom
    /// desktop. NOTE: not tested with desktop.
    /// \returns true if hardware forces are disabled.
    bool hardwareForceDisabled();

    /// Enable the device. Positions can be read and force can be sent.
    inline virtual ErrorCode enableDevice() {
      ErrorCode e = HAPIHapticsDevice::enableDevice();
      if( e != SUCCESS ) {
        return e;
      } else {
        // Starting scheduler here instead of in initHapticsDevice since because
        // of random crashes when using OpenHapticsRenderer. Apparantly HL API does
        // not like the scheduler to have already been started when creating a new
        // HL context.
        if( enable_start_scheduler ) {
          startScheduler();
          if( !scheduler_started ) {
            device_state = INITIALIZED;
            return NOT_ENABLED;
          }
        }
        return SUCCESS;
      }
    }

    /// Returns true if the device is currently updating its calibration.
    inline bool inCalibrationMode() { return in_calibration_mode; }

    // set the enable_start_scheduler variable.
    static inline void setEnableStartScheduler( bool new_value ) {
      enable_start_scheduler = new_value;
    }

    // start the hdScheduler.
    static void startScheduler();

    // Check if scheduler is started.
    static inline bool isSchedulerStarted() {
      return scheduler_started;
    }

    // Stop the scheduler. Sometimes the scheduler
    // is already stopped (when all hl_contexts are deleted )
    // and then just the flag needs to be set.
    static void stopScheduler( bool call_hd = true );

  protected:
    double device_firmware_version;
    std::string hdapi_version;
    std::string device_model_type;
    std::string device_driver_version;
    std::string device_vendor;
    std::string device_serial_number;
    Vec3 usable_workspace_max;
    Vec3 usable_workspace_min;
    Vec3 max_workspace_max;
    Vec3 max_workspace_min;
    HAPIFloat max_force, max_cont_force, tabletop_offset;
    int input_dof, output_dof;
    Vec3 joint_angles, gimbal_angles;
    std::vector< HAPIFloat > motor_temperatures;
    /// Raw encoder values for this device. I.e. the output from
    /// calling hdGetLongv with HD_CURRENT_ENCODER_VALUES as input.
    /// See the OpenHaptics SDK documentation for more information.
    std::vector< HAPIFloat > encoder_values;
    
    
    /// Implementation of updateDeviceValues using HD API to get the values.
    virtual void updateDeviceValues( DeviceValues &dv, HAPITime dt );

    /// Implementation of sendOutput using HD API to send forces.
    virtual void sendOutput( DeviceOutput &dv,
                             HAPITime dt );

    /// Implementation of initHapticsDevice using HD API.
    /// \param _thread_frequency The desired haptic frequency.
    /// Frequencies of 500, 1000, and 2000 Hz are valid when using a device
    /// connecting through PCI and EPP. Frequencies of 500, 1000, 1600Hz plus
    /// values in between based on the formula floor(8000/N + 0.5) are valid
    /// for devices connecting through Firewire.
    /// The first successful initialization of PhantomHapticsDevice will decide
    /// the haptics thread rate since only one scheduler is used by
    /// OpenHaptics even for dual device configurations.
    /// \returns true if device was initialized properly.
    virtual bool initHapticsDevice( int _thread_frequency = 1000 );

    /// Releases all resources allocated in initHapticsDevice. 
    virtual bool releaseHapticsDevice();

    /// Internal function for calibrating the device.
    void calibrateDeviceInternal();

    static HDCallbackCode HDCALLBACK endFrameCallback( void *data );

    /// The device name for this device.
    std::string device_name;
    /// The OpenHaptics device handle for this device.
    HHD device_handle;

    /// Handle for all setup callbacks.
    std::vector< HDCallbackCode > hd_handles;

    /// If true the device is in calibration_mode and will try to be
    /// calibrated. It is up to the user to decide if haptics should be
    /// shut off or not.
    bool in_calibration_mode;

    /// True if the device is currently in the inkwell
    bool in_inkwell;

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
