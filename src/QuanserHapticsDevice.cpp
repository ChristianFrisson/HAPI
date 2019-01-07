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
/// \file QuanserHapticsDevice.cpp
/// \brief Cpp file for QuanserHapticsDevice.
///
//
//////////////////////////////////////////////////////////////////////////////

#include <HAPI/QuanserHapticsDevice.h>

#ifdef HAVE_QUANSERAPI

using namespace HAPI;

HAPIHapticsDevice::HapticsDeviceRegistration 
QuanserHapticsDevice::device_registration(
                            "Quanser",
                            &(newInstance< QuanserHapticsDevice >)
                            );


bool QuanserHapticsDevice::initHapticsDevice( int _thread_frequency ) {
  const t_boolean nonblocking         = false;
  const t_int     send_buffer_size    = 80;
  const t_int     receive_buffer_size = 80;
  const char *    locale              = NULL;
  
  t_error  result;
 
  
  //This function attempts to connect to the server using the specified URI.
  result = stream_connect(uri.c_str(), nonblocking, 
        send_buffer_size, 
        receive_buffer_size, 
        &client);
  
  if( result != 0 ) {
    stringstream s;
    s << "Could not connect to Quanser haptics device using URI " << uri << ". Internal error code: " << result << endl;
    if( -result ==  QERR_LIBRARY_NOT_FOUND ) 
      s << "Dynamic librares not found. " << endl; 
    setErrorMsg( s.str() );
    return false;
  }

  return true;
  
  /*
    // initialize device, set success to true if 

    bool success = false;

    // Open shared memory: Haptics API
    // open shmem to read: no handshaking
    shmem_API_read = SHMEM_Create(_T("Quanser_Haptic_API_To_H3D"), 
        (DWORD)shmem_API_read_size, SHMEM_NO_HANDSHAKING);
    // open shmem to write: no handshaking
    shmem_API_write = SHMEM_Create(_T("Quanser_Haptic_API_From_H3D"), 
        (DWORD)shmem_API_write_size, SHMEM_NO_HANDSHAKING);

    // Quanser Haptic API inputs: 
    // [calibrate, enable_amps, Fx, Fy, KDx, KDy, KSx, KSy, SSx, SSy, 
    //     checksum_api_w, enable_pos_watch]
    // initialize all of the API inputs to zero
    for (int i = 0; i < shmem_API_write_num_doubles; ++i)
        write_API_inputs[i] = 0.0;
    // force calibration to be run
    schedule_calibration( true );
    // disable the haptic device power amplifiers
    enable_amplifiers( false );
    // disable the haptic device position watchdog
    enable_position_watchdog( false );
    if (SHMEM_Write(shmem_API_write, 0, (DWORD)shmem_API_write_size, &write_API_inputs, (DWORD)api_integer_timeout))
    {
        device_id = 0;
        success = true;
        return success;
    }
    else
    {
        stringstream s;
        s << "Error writing to Quanser Haptic API shared memory. "
          << "Warning: Failed to initialize Quanser haptic device. ";
        setErrorMsg( s.str() );
        setErrorMsg( s.str() );
        success = false;
        return success;
    }
  */
}

bool QuanserHapticsDevice::releaseHapticsDevice() 
{
  // release all resources allocated in initHapticsDevice.
  // disconnecting the device.
  if( device_state != UNINITIALIZED ) {
    stream_close( client );
  }
  return true;

/*    
    if (device_id != -1) 
    {
        /// clean-up the shared memory
        // reset 'checksum_api_w' in API shmem to 0 
        for (int i = 0; i < shmem_API_write_num_doubles; ++i)
            write_API_inputs[i] = 0.0;
        // force calibration to be run
        schedule_calibration( true );
        // disable the haptic device power amplifiers
        enable_amplifiers( false );
        if (!SHMEM_Write(shmem_API_write, 0, (DWORD)shmem_API_write_size, &write_API_inputs, (DWORD)api_integer_timeout))
        {
            stringstream s;
            s << "Warning: Failed to reset the Quanser haptic device. ";
            setErrorMsg( s.str() );
        }
        // reset the device_id
        device_id = -1;

        // do not free the shared memory
    }
    */
}

void QuanserHapticsDevice::updateDeviceValues( DeviceValues &dv,
                                               HAPITime dt ) {
  HAPIHapticsDevice::updateDeviceValues( dv, dt );
  
  if( device_state == UNINITIALIZED ) return;
  
  const t_int RCV_STREAM_SIZE = 7;
  t_double rcv_temp[RCV_STREAM_SIZE];
  t_int result = stream_receive_doubles(client, rcv_temp, RCV_STREAM_SIZE); 
  
  // error in communication, return.
  //if( result != 0 ) return;  

  // Cartesian position and velocity in metres
  //dv.position = Vec3( rcv_temp[1], rcv_temp[2], rcv_temp[0] );
  dv.position = Vec3( -rcv_temp[0], rcv_temp[2], rcv_temp[1] );
  // current linear velocity in world coordinate frame (m/s)
  //  dv.velocity = Vec3(API_outputs_read[2], API_outputs_read[3], 0);
  // Device Rotation
  Vec3 rotation_offset( 0,0,0);
  // 0 180 -90

  /*dv.orientation = Rotation(1,0,0,0);
  dv.orientation = Rotation( 0, 0, 1, (rcv_temp[3] + rotation_offset.z) ); // * H3DUtil::Constants::pi / 180  ); 
  dv.orientation = Rotation( dv.orientation * Vec3( 0, 1, 0 ), (rcv_temp[5] + rotation_offset.y) ) * dv.orientation;
  dv.orientation = Rotation( dv.orientation * Vec3( 1, 0, 0 ), (rcv_temp[4] + rotation_offset.x) ) * dv.orientation;
  */

  dv.orientation = Rotation(1,0,0,0);
  dv.orientation = Rotation( 0, 0, 1, (rcv_temp[4] + rotation_offset.z) ); //* H3DUtil::Constants::pi / 180  ); 
  dv.orientation = Rotation( Vec3( 0, 1, 0 ), (rcv_temp[5] + rotation_offset.y) ) * dv.orientation;
  dv.orientation = Rotation( Vec3( 1, 0, 0 ), (-rcv_temp[3] + rotation_offset.x) ) * dv.orientation;


  // bitmask for buttons. bit 0 is button 0, bit 1 button 1 and so on.
  // value of 1 indicates button pressed.
  //dv.button_status = 0;
}

bool QuanserHapticsDevice::is_fatal_error()
{
    if (API_outputs_read[5] == 0)   // fatal_error
        return false;
    else
        return true;
}

bool QuanserHapticsDevice::does_checksum_enable()
{
    if (API_outputs_read[6] == 0)   // checksum_enable
        return false;
    else
        return true;
}

bool QuanserHapticsDevice::does_write_timeout_enable()
{
    if (API_outputs_read[7] == 0)   // timeout_w_enable
        return false;
    else
        return true;
}

bool QuanserHapticsDevice::does_read_timeout_enable()
{
    if (API_outputs_read[8] == 0)   // timeout_r_enable
        return false;
    else
        return true;
}

void QuanserHapticsDevice::sendOutput( DeviceOutput &dv,
                                       HAPITime dt ) {

  if( device_state == UNINITIALIZED ) return;

  // send force to render: dv.force.x, dv.torque.x, etc.
  const t_int SND_STREAM_SIZE = 24; 

  t_double snd_temp[SND_STREAM_SIZE];
  
  for (int i = 0; i < SND_STREAM_SIZE; ++i)
    snd_temp[i] = 0;

  snd_temp[0] = -dv.force.x;
  snd_temp[1] = dv.force.z;
  snd_temp[2] = dv.force.y;

  snd_temp[3] = -dv.torque.x;
  snd_temp[4] = dv.torque.z;
  snd_temp[5] = dv.torque.y;

  // dv.force, dv.torque
  t_int result = stream_send_doubles(client, snd_temp, SND_STREAM_SIZE); 
}

void QuanserHapticsDevice::set_damping_gains( Vec3 gain )
{
    write_API_inputs[4] = gain.x;   // KDx
    write_API_inputs[5] = gain.y;   // KDy
    // this then relies on sendOutput.
}

void QuanserHapticsDevice::set_stiffness_gains( Vec3 gain )
{
    write_API_inputs[6] = gain.x;   // KSx
    write_API_inputs[7] = gain.y;   // KSy
    // this then relies on sendOutput.
}

void QuanserHapticsDevice::set_stiffness_position_setpoints( Vec3 position )
{
    write_API_inputs[8] = position.x;   // SSx
    write_API_inputs[9] = position.y;   // SSy
    // this then relies on sendOutput.
}

void QuanserHapticsDevice::schedule_calibration( bool calibrate )
{
    if (device_id != -1) 
    {
        if (calibrate == true)
            write_API_inputs[0] = 1;    // calibrate
        else
            write_API_inputs[0] = 0;
    }
}

void QuanserHapticsDevice::send_calibration()
{
  /*
    if (device_id != -1) 
    {
        // write only the calibration flag to the shared memory of the Quanser Haptics API 
        if (!SHMEM_Write(shmem_API_write, 0, (DWORD)sizeof(double), &write_API_inputs[0], (DWORD)api_integer_timeout))
        {
            stringstream s;
            s << "Warning: Failed to send the calibration flag to the Quanser haptic device. ";
            setErrorMsg( s.str() );
        }
    }
    */
}

void QuanserHapticsDevice::do_calibration( int timeout )
{
    if ((device_id != -1) && (write_API_inputs[0] == 1))
    {
        // the Quanser haptic device calibration procedure has been requested.
        DWORD answer = MessageBox(
            HWND_DESKTOP,
            TEXT("Position the Quanser Haptic Device into its calibration pose. ")
            TEXT("Press OK when ready for calibration."),
            TEXT("Calibration Procedure"),
            MB_OKCANCEL | MB_ICONQUESTION
        );
        if (answer == IDOK)
        {
            // do calibration
            // set the shmem to calibrate
            schedule_calibration( true );
            // send calibration flag by writing to shmem
            send_calibration();
            // leave enough time to reset the encoders
            Sleep(50); 
            // reset calibrate flag
            schedule_calibration( false );
            // send calibration flag by writing to shmem
            send_calibration();
            // status message
            DWORD answer = MessageBox(
                HWND_DESKTOP,
                TEXT("The Quanser haptic device has been calibrated. "),
                TEXT("End Of Calibration Procedure"),
                MB_OK
            );
        }
        else
        {
            // operation cancelled. Do not do calibration
            schedule_calibration( false );
            // send calibration flag by writing to shmem
            send_calibration();
            // status message
            DWORD answer = MessageBox(
                HWND_DESKTOP,
                TEXT("The Quanser haptic device has not been calibrated. "),
                TEXT("End Of Calibration Procedure"),
                MB_OK
            );
        }
    }
}

void QuanserHapticsDevice::enable_amplifiers( bool enable )
{
    if (enable == true)
        write_API_inputs[1] = 1;    // enable_amps
    else
        write_API_inputs[1] = 0;
    // this then relies on sendOutput.
}

void QuanserHapticsDevice::compute_checksum()
{
    // shmem checksum to write to API: checksum_api_w  == write_API_inputs[10]
    // read API shmem checksum:  checksum_api_r == API_outputs_read[4]
    // setting of output checksum with: checksum_wrap = 2000 and checksum_offset = 1000
    write_API_inputs[10] = fmod( API_outputs_read[4] - 1000 + 1, 2000 ) + 1000;
    // this then relies on sendOutput.
}

void QuanserHapticsDevice::enable_position_watchdog( bool enable )
{
    if (enable == true)
        write_API_inputs[11] = 1;   // enable_pos_watch
    else
        write_API_inputs[11] = 0;
    // this then relies on sendOutput.
}

#endif  // HAVE_QUANSERAPI

