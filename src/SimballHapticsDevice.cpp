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
/// \file SimballHapticsDevice.cpp
/// \brief Cpp file for SimballHapticsDevice.
///
//
//////////////////////////////////////////////////////////////////////////////


#include <HAPI/SimballHapticsDevice.h>
#include <H3DUtil/DynamicLibrary.h>

#ifdef HAVE_SIMBALLMEDICAL_API
#include <Simball/SimballMedicalHID.h>

using namespace HAPI;

namespace SimballHapticsDeviceInternal {
  std::string libs_array[1] = {"SimballMedicalHID.dll"};
  std::list< std::string > simball_device_libs(libs_array, libs_array + 1 );
}

HAPIHapticsDevice::HapticsDeviceRegistration 
SimballHapticsDevice::device_registration(
                       "SimballHapticsDevice",
                       &(newInstance< SimballHapticsDevice >),
                       SimballHapticsDeviceInternal::simball_device_libs );

int SimballHapticsDevice::nr_found_devices = -1;
int SimballHapticsDevice::nr_initialized_devices = 0;

bool SimballHapticsDevice::initHapticsDevice( int _thread_frequency ) {
#ifdef H3D_WINDOWS
  /// need to go check if the dll to support this haptic device can be correctly
  /// loaded
  std::list<std::string>::iterator it = device_registration.libs_to_support.begin();
  for( ; it!= device_registration.libs_to_support.end();++it ) {
    if( !H3DUtil::DynamicLibrary::load(*it) ) {
      setErrorMsg("Warning: can not load required DLL for "+ device_registration.name+ "device");
      return false; // if required lib can not be loaed, do not register this device
    }
  }
#endif //H3D_WINDOWS
  int nr_devices = -1;
  int error_ms = SBM_Init( nr_devices );
  if( error_ms != SBMError_NoError ) {
    std::stringstream s;
    s << "Could not initialize SimballHapticsDevice. "
      << "Make sure one is connected properly. ";
    setErrorMsg( s.str() );
    return false;
  } else if( device_nr > nr_devices - 1 ) {
    std::stringstream s;
    s << "Could not initialize SimballHapticsDevice with device nr "
      << device_nr << " because the number of connected devices are "
      << nr_devices << " Make sure all are connected properly. ";
    setErrorMsg( s.str() );
    return false;
  }
  if( nr_found_devices < nr_devices )
    nr_found_devices = nr_devices;
  ++nr_initialized_devices;
  return true;
}

bool SimballHapticsDevice::releaseHapticsDevice() {
  HAPIHapticsDevice::disableDevice();
  if( nr_initialized_devices == 1 ) {
    int error_ms = SBM_Exit();
    if( error_ms != SBMError_NoError ) {
      std::stringstream s;
      s << "Error when closing connection to SimballHapticsDevice. ";
      setErrorMsg( s.str() );
      return false;
    }
  }
  --nr_initialized_devices;
  return true;
}

void SimballHapticsDevice::updateDeviceValues( DeviceValues &dv,
                                               HAPITime dt ) {
  HAPIHapticsDevice::updateDeviceValues( dv, dt );
  if( device_nr < nr_found_devices ) {
    SBMDeviceValues simball_values;
    int error_ms = SBM_GetValues( device_nr, simball_values );
    if( error_ms != SBMError_NoError ) {
      std::stringstream s;
      s << "Could not update device values for SimballHapticsDevice. "
        << "Make sure one is connected properly. ";
      setErrorMsg( s.str() );
      return;
    }
    
    HAPIFloat half_pi = H3DUtil::Constants::pi / 2;
    Rotation about_x_and_y =
      Rotation( 1, 0, 0, (float)(simball_values.theta - half_pi) ) *
      Rotation( 0, -1, 0, (float)(simball_values.phi - half_pi) );

    if( simball_values.flags == SBMFlag_AInserted ) {
      dv.position = about_x_and_y *
                    Vec3( 0, 0, -simball_values.insertionA * 1e-3 );
      handle_angle = simball_values.handleA;
    } else if( simball_values.flags == SBMFlag_BInserted ) {
      dv.position = about_x_and_y *
                    Vec3( 0, 0, -simball_values.insertionB * 1e-3 );
      handle_angle = simball_values.handleB;
    }

    dv.orientation = about_x_and_y *
                     Rotation( 0, 0, 1, (float)simball_values.gamma );
    
    dv.button_status = 0;
    if( simball_values.flags == SBMFlag_Button1 )
      dv.button_status = dv.button_status | 0x01;
    if( simball_values.flags == SBMFlag_Button2 )
      dv.button_status = dv.button_status | 0x02;
    if( simball_values.flags == SBMFlag_Button3 )
      dv.button_status = dv.button_status | 0x04;
    if( simball_values.flags == SBMFlag_PedalALeft )
      dv.button_status = dv.button_status | 0x08;
    if( simball_values.flags == SBMFlag_PedalARight )
      dv.button_status = dv.button_status | 0x10;
    if( simball_values.flags == SBMFlag_PedalBLeft )
      dv.button_status = dv.button_status | 0x20;
    if( simball_values.flags == SBMFlag_PedalBRight )
      dv.button_status = dv.button_status | 0x40;
    calculateVelocity( dv, dt );
  }
}

void SimballHapticsDevice::sendOutput( DeviceOutput &dv,
                                       HAPITime dt ) {
}

#endif //HAVE_SIMBALLMEDICAL_API
