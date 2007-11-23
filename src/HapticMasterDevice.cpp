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
/// \file HapticMasterDevice.cpp
/// \brief Cpp file for HapticMasterDevice.
///
//
//////////////////////////////////////////////////////////////////////////////

#include <HAPI/HapticMasterDevice.h>

#ifdef HAVE_HAPTIC_MASTER_API
// This header file was empty, so we skip including it.
//#include <HapticMasterDriver.h>
#include <fstream>

using namespace HAPI;

namespace HapticMasterDeviceInternal {
  string libs_array[2] = {"HapticAPI.dll", "HapticMasterDriver.dll"};
  list< string > haptic_master_device_libs(libs_array, libs_array + 2 );
}

enum FCSSTATE
{
   FCSSTATE_UNKNOWN,
   FCSSTATE_FAIL = 100,
   FCSSTATE_OFF,
   FCSSTATE_INITIALISING,
   FCSSTATE_INITIALISED,
   FCSSTATE_NORMAL,
   FCSSTATE_FREE,
   FCSSTATE_FIXED,
};

//---------------------------------------------------------------------------


typedef int		(*LPFNDLLOpenHapticMaster)			(char *DeviceName);
typedef void	(*LPFNDLLCloseHapticMaster)			(int Device);
typedef int		(*LPFNDLLInitialiseHapticMaster)	(int Device);
typedef void	(*LPFNDLLSetForceGetPosition)		(int Device, double Force[3], double CurrentPosition[3]);
typedef void	(*LPFNDLLSetForceGetPV)				(int Device, double Force[3], double CurrentPosition[3], double CurrentVelocity[3]);
typedef void	(*LPFNDLLSetInertia)				(int Device, double Inertia);
typedef int		(*LPFNDLLGetState)					(int Device);
typedef void	(*LPFNDLLSetState)					(int Device, int State);;

LPFNDLLOpenHapticMaster			OpenHapticMaster; 
LPFNDLLCloseHapticMaster		CloseHapticMaster; 
LPFNDLLInitialiseHapticMaster	InitialiseHapticMaster; 
LPFNDLLSetForceGetPosition		SetForceGetPosition;
LPFNDLLSetForceGetPV			SetForceGetPV;
LPFNDLLSetInertia				SetInertia;
LPFNDLLGetState					GetState;
LPFNDLLSetState					SetState;

HAPIHapticsDevice::HapticsDeviceRegistration 
HapticMasterDevice::device_registration(
                       "HapticMaster",
                       &(newInstance< HapticMasterDevice >),
                       HapticMasterDeviceInternal::haptic_master_device_libs
                                        );

H3DUtil::DynamicLibrary::LIBHANDLE HapticMasterDevice::dll_handle;
unsigned int HapticMasterDevice::dll_references = 0;

bool HapticMasterDevice::initHapticsDevice( int _thread_frequency ) {
  if (dll_references == 0 ) {
    // Load the DLL library in memory
	dll_handle = H3DUtil::DynamicLibrary::load("HapticMasterDriver.dll");
    
    if( dll_handle ) {
      // Get DLL Function pointers
      OpenHapticMaster = 
        (LPFNDLLOpenHapticMaster) GetProcAddress(dll_handle,
                                                 "OpenHapticMaster"); 
      CloseHapticMaster =
        (LPFNDLLCloseHapticMaster) GetProcAddress(dll_handle,
                                                  "CloseHapticMaster"); 
      SetForceGetPosition	= 
        (LPFNDLLSetForceGetPosition) GetProcAddress(dll_handle,
                                                    "SetForceGetPosition"); 
      SetForceGetPV	= 
        (LPFNDLLSetForceGetPV) GetProcAddress(dll_handle,
                                              "SetForceGetPV"); 
      InitialiseHapticMaster	= 
        (LPFNDLLInitialiseHapticMaster)	GetProcAddress(dll_handle,
                                                       "InitialiseHapticMaster"); 
      SetInertia = 
        (LPFNDLLSetInertia)	GetProcAddress(dll_handle,"SetInertia"); 
      GetState = (LPFNDLLGetState) GetProcAddress(dll_handle,"GetState"); 
      SetState = (LPFNDLLSetState) GetProcAddress(dll_handle,"SetState"); 
    }

    // If we failed to load the library or any of the functions, quit...
    if (dll_handle==NULL) {
       stringstream s;
       s << "Could not initialize HapticMasterDevice. "
         << "HapticMasterDriver.dll not found. ";
       setErrorMsg( s.str() );
       return false;
    }
    dll_references++;
  }

  

  device_handle = OpenHapticMaster( (char * )device_name.c_str() ); 

  if (device_handle != -1) {
    // TODO: what is the return values.
    int result = InitialiseHapticMaster(device_handle);
    return true;
  } else {
    // check for the servers.db file
    ifstream is( "servers.db");
    stringstream s;
    if( is.fail() ) {
      s << "Could not initialize HapticMasterDevice. No servers.db file "
        << "in the current directory. The servers.db file specifies the "
        << "ip address of the HapticMaster."; 
    } else {
      s << "Could not initialize HapticMasterDevice. "
        << "Make sure one is connected properly. ";
    }
    is.close();
    setErrorMsg( s.str() );
    return false;
  }	
}

bool HapticMasterDevice::releaseHapticsDevice() {
  SetState(device_handle, FCSSTATE_INITIALISED);
  CloseHapticMaster(device_handle);    
  if( dll_references > 0 ) {
    dll_references--;
    if( dll_references == 1 )
      H3DUtil::DynamicLibrary::close( dll_handle );
  }
  return true;
}

void HapticMasterDevice::updateDeviceValues( DeviceValues &dv,
                                             HAPITime dt ) {
  HAPIHapticsDevice::updateDeviceValues( dv, dt );
  double force[] = { dv.force.z, dv.force.x, dv.force.y };
  double pos[3] = { 0.0, 0.0, 0.0};
  double vel[3] = { 0.0, 0.0, 0.0};
  SetForceGetPV(device_handle, force, pos, vel );
  
  dv.position = Vec3(pos[1], pos[2], pos[0]) * 1000;
  dv.velocity = Vec3(vel[1], vel[2], vel[0]) * 1000;
       
  dv.orientation = Rotation( 1, 0, 0, 0 );
  
  // TODO: button
  // bitmask for buttons. bit 0 is button 0, bit 1 button 1 and so on.
  // value of 1 indicates button pressed.
  //dv.button_status = 0;
    
}

void HapticMasterDevice::sendOutput( DeviceOutput &dv,
                                     HAPITime dt ) {

  // this is sent in updateDeviceValues in order to minimize the number of
  // calls to the haptic master.
}

#endif  // HAVE_HAPTIC_MASTER_API

