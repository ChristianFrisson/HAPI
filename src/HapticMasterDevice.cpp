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
/// \file HAPI/src/HapticMasterDevice.cpp
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
  std::string libs_array[2] = {"HapticAPI.dll", "HapticMasterDriver.dll"};
  std::list< std::string > haptic_master_device_libs(libs_array, libs_array + 2 );
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


typedef int  (*LPFNDLLOpenHapticMaster)      (char *DeviceName);
typedef void (*LPFNDLLCloseHapticMaster)     (int Device);
typedef int  (*LPFNDLLInitialiseHapticMaster)(int Device);
typedef void (*LPFNDLLSetForceGetPosition)   (int Device,
                                              double Force[3],
                                              double CurrentPosition[3]);
typedef void (*LPFNDLLSetForceGetPV)         (int Device,
                                              double Force[3],
                                              double CurrentPosition[3],
                                              double CurrentVelocity[3]);
typedef void (*LPFNDLLSetForceGetPVR)         (int Device,
                                              double Force[3],
                                              double CurrentPosition[3],
                                              double CurrentVelocity[3],
                                              double CurrentRotation[3]);
typedef void (*LPFNDLLSetInertia)            (int Device,
                                              double Inertia);
typedef int  (*LPFNDLLGetState)              (int Device);
typedef void (*LPFNDLLSetState)              (int Device,
                                              int State);
typedef int  (*LPFNDLLCreateSphere)          (int Device,
                                              double center[3],
                                              double Radius,
                                              double ExtSpringStiffness,
                                              double IntSpringStiffness,
                                              double ExtDampingFactor,
                                              double IntDampingFactor,
                                              double ExtThickness,
                                              double IntThickness);
typedef int  (*LPFNDLLDeleteSphere)          (int Device, int Sphere);
typedef int  (*LPFNDLLSetSphereRadius)       (int Sphere, double radius);
typedef int  (*LPFNDLLSetSpherePosition)     (int Sphere, double pos[3]);
typedef void (*LPFNDLLGetCurrentForce)       (int Device, double force[3]);

LPFNDLLOpenHapticMaster       OpenHapticMaster; 
LPFNDLLCloseHapticMaster      CloseHapticMaster; 
LPFNDLLInitialiseHapticMaster InitialiseHapticMaster; 
LPFNDLLSetForceGetPosition    SetForceGetPosition;
LPFNDLLSetForceGetPV          SetForceGetPV;
LPFNDLLSetForceGetPVR          SetForceGetPVR;
LPFNDLLSetInertia             SetInertia;
LPFNDLLGetState               GetState;
LPFNDLLSetState               SetState;
LPFNDLLCreateSphere           CreateSphere;
LPFNDLLDeleteSphere           DeleteSphere;
LPFNDLLSetSphereRadius        SetSphereRadius;
LPFNDLLSetSpherePosition      SetSpherePosition;
LPFNDLLGetCurrentForce        GetCurrentForce;

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
      SetForceGetPosition = 
        (LPFNDLLSetForceGetPosition) GetProcAddress(dll_handle,
                                                    "SetForceGetPosition"); 
      SetForceGetPV =
        (LPFNDLLSetForceGetPV) GetProcAddress(dll_handle,
                                              "SetForceGetPV"); 
      SetForceGetPVR =
        (LPFNDLLSetForceGetPVR) GetProcAddress(dll_handle,
                                              "SetForceGetPVR"); 
      InitialiseHapticMaster  = 
        (LPFNDLLInitialiseHapticMaster) GetProcAddress(dll_handle,
                                                     "InitialiseHapticMaster");
      SetInertia =
        (LPFNDLLSetInertia) GetProcAddress(dll_handle,"SetInertia"); 
      GetState = (LPFNDLLGetState) GetProcAddress(dll_handle,"GetState"); 
      SetState = (LPFNDLLSetState) GetProcAddress(dll_handle,"SetState"); 

      CreateSphere = 
        (LPFNDLLCreateSphere) GetProcAddress(dll_handle,"CreateSphere");

      DeleteSphere = 
        (LPFNDLLDeleteSphere) GetProcAddress(dll_handle,"DeleteSphere"); 

      SetSphereRadius = 
        (LPFNDLLSetSphereRadius) GetProcAddress(dll_handle,"SetSphereRadius");

      SetSpherePosition = 
        (LPFNDLLSetSpherePosition) GetProcAddress(dll_handle,
                                                  "SetSpherePosition");
      GetCurrentForce = 
        (LPFNDLLGetCurrentForce) GetProcAddress(dll_handle,"GetCurrentForce");
    }

    // If we failed to load the library or any of the functions, quit...
    if (dll_handle==NULL) {
       std::stringstream s;
       s << "Could not initialize HapticMasterDevice. "
         << "HapticMasterDriver.dll or HapticAPI.dll not found. ";
       setErrorMsg( s.str() );
       return false;
    }
    ++dll_references;
  }

  

  device_handle = OpenHapticMaster( (char * )device_name.c_str() ); 

  if (device_handle != -1) {
    int result = InitialiseHapticMaster(device_handle);

    // initalize the thread used for communication.
    com_thread = 
      new H3DUtil::PeriodicThread( H3DUtil::ThreadBase::HIGH_PRIORITY, 1000 );
    com_thread->setThreadName( "HapticMASTER com thread" );

    com_func_cb_handle = com_thread->asynchronousCallback( com_func, this );
    
    return true;
  } else {
    // check for the servers.db file
    std::ifstream is( "servers.db");
    std::stringstream s;
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
  disableDevice();

  if( device_handle != -1 ) {
    if( com_thread ) {
      if( com_func_cb_handle != -1 ) {
        com_thread->removeAsynchronousCallback( com_func_cb_handle );
        com_func_cb_handle = -1;
      }
      delete com_thread;
      com_thread = NULL;
    }
    SetState(device_handle, FCSSTATE_INITIALISED);
    CloseHapticMaster(device_handle);    
    if( dll_references > 0 ) {
      --dll_references;
      if( dll_references == 1 )
        H3DUtil::DynamicLibrary::close( dll_handle );
    }
    device_handle = -1;
  }
  return true;
}

// Doxygen documentation reasons for adding HAPI::
void HAPI::HapticMasterDevice::updateDeviceValues( DeviceValues &dv,
                                             HAPITime dt ) {
  HAPIHapticsDevice::updateDeviceValues( dv, dt );
  
  if( device_handle != -1 ) {
    com_lock.lock();
    dv.position = current_values.position;
    dv.velocity = current_values.velocity;
    dv.orientation = current_values.orientation;
    dv.button_status = current_values.button_status;
    dv.force = current_values.force;
    com_lock.unlock();
  }
    
}

void HapticMasterDevice::sendOutput( DeviceOutput &dv,
                                     HAPITime dt ) {
  if( device_handle != -1 ) {
    //com_lock.lock();
    //current_values.force = dv.force;
    //current_values.torque = dv.torque;
    //com_lock.unlock();
  }
}

int HapticMasterDevice::createSphere( Vec3 pos, double radius,
                                      double ext_spring_stiffness,
                                      double int_spring_stiffness,
                                      double ext_damping_factor,
                                      double int_damping_factor,
                                      double ext_thickness,
                                      double int_thickness ) {
 /* Matrix4 position_calibration_inverse = Matrix4( 0.5, 0, 0, 0,
    0, 0.5, 0, 0,
    0, 0, 0.5, 0,
    0, 0, 0, 1 );
*/
  Vec3 local_pos = position_calibration_inverse * pos;

  Vec3 scaling = position_calibration_inverse.getScalePart();
  double local_radius = radius * H3DMax( scaling.x, H3DMax( scaling.y, scaling.z ) );
  double p[] = { local_pos.z, local_pos.x, local_pos.y };
  if( device_handle == -1 ) return -1;
  else {
    driver_lock.lock();
    int res = CreateSphere( device_handle, p, local_radius, 
                            ext_spring_stiffness,
                            int_spring_stiffness,
                            ext_damping_factor,
                            int_damping_factor,
                            ext_thickness,
                            int_thickness );
    driver_lock.unlock();
    return res;
  }
}



int HapticMasterDevice::deleteSphere( int sphere ) {
  if( device_handle == -1 ) return -1;

  driver_lock.lock();
  int res =  DeleteSphere( device_handle, sphere );
  driver_lock.unlock();
  return res;
}

int HapticMasterDevice::setSphereRadius( int sphere, double radius ) {
  if( device_handle == -1 ) return -1;
  Vec3 scaling = position_calibration_inverse.getScalePart();
  double local_radius = radius * H3DMax( scaling.x, H3DMax( scaling.y, scaling.z ) );
  driver_lock.lock();
  int res = SetSphereRadius( sphere, radius );
  driver_lock.unlock();
  return res;
}
int HapticMasterDevice::setSpherePosition( int sphere, Vec3 pos ) {
  if( device_handle == -1 ) return -1;
  Vec3 local_pos = position_calibration_inverse * pos;
  double p[] = { local_pos.z, local_pos.x, local_pos.y };
  driver_lock.lock();
  int res = SetSpherePosition( sphere, p ); 
  driver_lock.unlock();
  return res;
}

H3DUtil::PeriodicThread::CallbackCode
HapticMasterDevice::com_func( void *data ) {
  HapticMasterDevice *hd = 
    static_cast< HapticMasterDevice * >( data );
  
  if( hd->device_handle != -1 ) {

    hd->com_lock.lock();
    double force[] = { hd->output.force.z, 
                       hd->output.force.x, 
                       hd->output.force.y };

    hd->com_lock.unlock();

    double pos[3] = { 0.0, 0.0, 0.0};
    double vel[3] = { 0.0, 0.0, 0.0};
    double orn[3] = { 0.0, 0.0, 0.0};
    
    hd->driver_lock.lock();
    SetForceGetPVR(hd->device_handle, force, pos, vel, orn );
    GetCurrentForce( hd->device_handle, force );
    hd->driver_lock.unlock();

    Vec3 position = Vec3(pos[1], pos[2], pos[0]);
    Vec3 velocity = Vec3(vel[1], vel[2], vel[0]);
    Rotation rot = Rotation( 0, 1, 0, orn[2] ) * Rotation( 1, 0, 0, orn[1] ) * Rotation( 0, 0, 1, orn[0] );
   
    hd->com_lock.lock();
    hd->current_values.position = position;
    hd->current_values.velocity = velocity;
    hd->current_values.force = Vec3( force[1], force[2], force[0] );
    hd->current_values.orientation = rot;
    hd->com_lock.unlock();
  }
                          
  return H3DUtil::PeriodicThread::CALLBACK_CONTINUE;
}
#endif


