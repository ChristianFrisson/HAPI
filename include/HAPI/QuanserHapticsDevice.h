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
/// \file QuanserHapticsDevice.h
/// \brief Header file for QuanserHapticsDevice.
///
//
//////////////////////////////////////////////////////////////////////////////

#if !defined (__QUANSERHAPTICSDEVICE_H__)
#define __QUANSERHAPTICSDEVICE_H__

#include <HAPI/HAPIHapticsDevice.h>

#ifdef HAVE_QUANSERAPI

#include <quanser_stream.h>
#include <quanser_messages.h>

namespace HAPI {

  /// \ingroup HapticsDevices
  /// \class QuanserHapticsDevice
  /// \brief Interface to haptics devices from Quanser Inc. 
  ///
  /// See www.quanser.com for more info on the devices.
  class HAPI_API QuanserHapticsDevice: public HAPIHapticsDevice {
  public:
    /// Constructor.
    QuanserHapticsDevice( const string &_uri = "shmem://foobar:1"): 
    uri( _uri ),
      device_id( -1 ){}

    /// Destructor.
    virtual ~QuanserHapticsDevice() {}

    /// Return the Quanser device device_id for this device.
    inline int getDeviceId() { 
      return device_id;
    }

    inline const string &getURI() { return uri; }

    /// Register this device to the haptics device database.
    static HapticsDeviceRegistration device_registration;
  
    /// Schedule the haptic device to be calibrated if the flag argument is
    /// true.
    /// Do not schedule the haptic device to be calibrated if the flag argument
    /// is false.
    void schedule_calibration( bool calibrate );

    /// Send the calibration flag to the haptic device.
    void send_calibration( );

    /// If scheduled, run (or skip) the haptic device calibration procedure
    /// (i.e., reset of the encoders).
    /// Block/wait until completed. Optional timeout argument.
    void do_calibration( int timeout );

    /// Enable (or disable) the haptic device power amplifiers.
    void enable_amplifiers( bool enable );

    /// Enable (or disable) the haptic device position watchdog.
    void enable_position_watchdog( bool enable );

    /// Set the Damping Gains.
    void set_damping_gains( Vec3 gain );

    /// Set the Stiffness Gains.
    void set_stiffness_gains( Vec3 gain );

    /// Set the Stiffness Position Setpoints.
    void set_stiffness_position_setpoints( Vec3 position );

    /// Return true is one of the "fatal errors" happened, false otherwise.
    /// A fatal error flag requires the user to restart his/her application.
    bool is_fatal_error();

    /// Return true if the shmem checksum does enable the haptic device power
    /// amplifiers  (i.e., if the shmem communication is still valid).
    /// Return false otherwise. Used for monitoring.
    bool does_checksum_enable();

    /// Return true if the shmem read timeout watchdog does enable the haptic
    /// device power amplifiers 
    /// (i.e., if the shmem communication is still valid).
    /// Return false otherwise. Used for monitoring.
    bool does_write_timeout_enable();

    /// Return true if the shmem write timeout watchdog does enable the haptic
    /// device power amplifiers 
    /// (i.e., if the shmem communication is still valid).
    /// Return false otherwise. Used for monitoring.
    bool does_read_timeout_enable();

  protected:
    /// Get the device values(position, orientation, etc. )
    virtual void updateDeviceValues( DeviceValues &dv, 
                                     HAPITime dt );

    /// Send forces and torques to render
    virtual void sendOutput( DeviceOutput &dv,
                             HAPITime dt );

    /// Initialize the haptics device. Use the HapticThread class in Threads.h
    /// as the thread for haptic rendering.
    /// \param _thread_frequency is the desired haptic frequency. 
    /// 1000 is the maximum allowed frequency that can be specified. Setting
    /// this parameter to -1 means run as fast as possible. It is recommended
    /// to use the default value for most users.
    virtual bool initHapticsDevice( int _thread_frequency = 1000 );

    /// Releases all resources allocated in initHapticsDevice. 
    virtual bool releaseHapticsDevice();

    /// The Quanser Haptic API device IDentification number for this device.
    int device_id;

  
    /// characteristics on Quanser Haptic API outputs
    // number of doubles to read
    const static int   shmem_API_read_num_doubles  = 9;
    // size in bytes of shared memory to read
    const static DWORD shmem_API_read_size         = shmem_API_read_num_doubles
      * sizeof(double);
    /// characteristics on Quanser Haptic API inputs
    // number of doubles to write
    const static int   shmem_API_write_num_doubles = 12;
    // size in bytes of shared memory to write
    const static DWORD shmem_API_write_size       = shmem_API_write_num_doubles
      * sizeof(double);
    /// loop timing (ms)
    const static int   api_update_dt_ms    = 1;
    /// shmem timeout value (ms)
    const static int   api_integer_timeout = (int)(4 * api_update_dt_ms); 
    /// Quanser Haptic API data arrays
    // contain the Quanser Haptic API inputs
    double             write_API_inputs[ shmem_API_write_num_doubles ];
    // contain the Quanser Haptic API outputs
    double             API_outputs_read[ shmem_API_read_num_doubles ];

    /// Compute the current shared memory checksum value.
    void compute_checksum();

    string uri;
    t_stream client;

  };
}

#endif  // HAVE_QUANSERAPI

#endif  // __QUANSERHAPTICSDEVICE_H__
