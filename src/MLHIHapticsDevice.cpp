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
/// \file MLHIHapticsDevice.cpp
/// \brief cpp file for MLHIHapticsDevice
/// Code contributed by Sebastian Bozlee and Jennifer McFatridge
/// University of Hawaii at Manoa Human-Robot Interface Laboratory
//
//////////////////////////////////////////////////////////////////////////////
#include "HAPI/MLHIHapticsDevice.h"

#ifdef HAVE_MLHI
#include <string>
#include <iostream>
#include <cstdlib>
#include <cstring>
#ifndef H3D_WINDOWS
#include <unistd.h>
#endif

using namespace HAPI;

namespace MLHIHapticsDeviceInternal
{
  inline double length(float* vec)
  {
    return H3DUtil::H3DSqrt( vec[0] * vec[0]
                            + vec[1] * vec[1]
                            + vec[2] * vec[2]);
  }

  const float MAX_FORCE = 40;
  const int A_FEW_CYCLES = 20000;
  const int DAMPING_AMT = 5;
}

map<ml_device_handle_t, MLHIHapticsDevice*> MLHIHapticsDevice::devices;

HAPIHapticsDevice::HapticsDeviceRegistration
MLHIHapticsDevice::device_registration( "MLHIHapticsDevice",
                                        &(newInstance<MLHIHapticsDevice>) );

MLHIHapticsDevice* MLHIHapticsDevice::getDeviceByHandle(
  ml_device_handle_t device_hdl)
{
  return devices[device_hdl];
}

bool MLHIHapticsDevice::initHapticsDevice(int _thread_frequency)
{
  if( ml_Connect(&device_hdl, server_ip_addr.c_str()) != ML_STATUS_OK )
  {
    stringstream s;
    s << "Warning: Connection to " << server_ip_addr
      << " failed for MLHIHapticsDevice. "
      << "Please check the address and try again.".
    setErrorMsg( s.str() );
    return false;
  }

  if (!takeoffWhenReady())
    return false;

  defyGravity();
  waitForButtonPress();
  initGains();
  storeInDevices();
  registerCallbacks();

  return true;
}

bool MLHIHapticsDevice::releaseHapticsDevice()
{
  HAPIHapticsDevice::disableDevice(); // Called by Phantom device, so I'm copying that here.

  unregisterCallbacks();
  removeFromDevices();
  ml_Land(device_hdl);
  ml_Disconnect(device_hdl);

  return true;
}

void MLHIHapticsDevice::updateDeviceValues(DeviceValues &dv, HAPITime dt)
{
  position_and_velocity_lock.lock();
  dv.position = current_position;
  dv.orientation = current_rotation;
  dv.velocity = velocity;
  position_and_velocity_lock.unlock();
  dv.button_status = button_status;
}

void MLHIHapticsDevice::renderForces(ml_position_t position)
{
  ml_forces_t temp_forces;

  forces_lock.lock();
  temp_forces.values[0] = forces.values[0];
  temp_forces.values[1] = forces.values[1];
  temp_forces.values[2] = forces.values[2];
  forces_lock.unlock();

  float force_length = MLHIHapticsDeviceInternal::length(temp_forces.values);
  float r = MLHIHapticsDeviceInternal::length(position.values);
  ml_gain_vec_t gains;

  // Copy feedforward forces
  for (int i = 0; i < 6; ++i)
    gains.values[i].ff = initial_gains.values[i].ff;

  // Copy torque gains
  for (int i = 3; i < 6; ++i)
  {
    gains.values[i].p = initial_gains.values[i].p;
    gains.values[i].i = initial_gains.values[i].i;
    gains.values[i].d = initial_gains.values[i].d;
  }

  if(force_length == 0)
  {
    gains.values[0].p = 0;
    gains.values[0].d = DAMPING_AMT;
    gains.values[1].p = 0;
    gains.values[1].d = DAMPING_AMT;
    gains.values[2].p = 0;
    gains.values[2].d = DAMPING_AMT;
  }
  else
  {   
    if(force_length > MAX_FORCE)
    {
      temp_forces.values[0] = MAX_FORCE*temp_forces.values[0]/force_length;
      temp_forces.values[1] = MAX_FORCE*temp_forces.values[1]/force_length;
      temp_forces.values[2] = MAX_FORCE*temp_forces.values[2]/force_length;
    }
    gains.values[0].p = 1000;
    gains.values[0].d = DAMPING_AMT;
    gains.values[1].p = 1000;
    gains.values[1].d = DAMPING_AMT;
    gains.values[2].p = 1000;
    gains.values[2].d = DAMPING_AMT;

    ml_position_t desired_position;
    desired_position.values[0] = 1.0/(gains.values[0].p)*temp_forces.values[0] + position.values[0];
    desired_position.values[1] = 1.0/(gains.values[1].p)*temp_forces.values[1] + position.values[1];
    desired_position.values[2] = 1.0/(gains.values[2].p)*temp_forces.values[2] + position.values[2];
    desired_position.values[3] = 0;
    desired_position.values[4] = 0;
    desired_position.values[5] = 0;

    ml_SetDesiredPosition(device_hdl, desired_position);
  }
  ml_SetGainVecAxes(device_hdl, ML_GAINSET_TYPE_NORMAL, gains);
}

void MLHIHapticsDevice::sendOutput(DeviceOutput &dv, HAPITime dt)
{
  forces_lock.lock();
  forces.values[0] = dv.force.x;
  forces.values[1] = -dv.force.z;
  forces.values[2] = dv.force.y;
  forces_lock.unlock();
}

bool MLHIHapticsDevice::takeoffWhenReady()
{
  ml_fault_t fault;
  ml_GetFault(device_hdl, &fault);


  if (fault.value) {
    s << "There is a fault in MLHIHapticsDevice. "
      << "Try moving the flotor and reinitialize.";
    setErrorMsg( s.str() );
    return false;
  }
  // SG comment: This code is odd. Wonder why a loop is needed. Will
  // uncomment for now. Need to handle this better in HAPI.
  *while (fault.value)
  {
    int errorValue = ml_ResetFault(device_hdl);
    usleep(A_FEW_CYCLES);   // Let it check for faults again.
    ml_GetFault(device_hdl, &fault);
  }*/

  ml_Takeoff(device_hdl);     // attempt to takeoff if the fault value is OK

  ml_GetFault(device_hdl, &fault);
  if (fault.value)
  {
    s << "Failed to initialize MLHIHapticsDevice.";
    setErrorMsg( s.str() );
    return false;
  }
  return true;
}

void MLHIHapticsDevice::defyGravity()
{
  ml_forces_t gravity;
  ml_FindGravity(device_hdl, &gravity);
  ml_SetGravity(device_hdl, gravity);
  ml_DefyGravity(device_hdl);
}

void MLHIHapticsDevice::waitForButtonPress()
{
  cerr << "Press button when ready." << endl;
  ml_button_t button;
  ml_GetButtonStatus(device_hdl, &button);
  while (!(button.left || button.right))
  {
    usleep(A_FEW_CYCLES);
    ml_GetButtonStatus(device_hdl, &button);
  }
}

void MLHIHapticsDevice::allowRotation(bool allow)
{
  if(allow)
  {
    initial_gains.values[3].p = 1.5;
    initial_gains.values[4].p = 1.5;
    initial_gains.values[5].p = 1.5;
  }
  else
  {
    initial_gains.values[3].p = 50;
    initial_gains.values[4].p = 50;
    initial_gains.values[5].p = 10;
  }
}

void MLHIHapticsDevice::initGains()
{
  ml_GetGainVecAxes(device_hdl, ML_GAINSET_TYPE_NORMAL, &initial_gains);

  initial_gains.values[0].p = 0;
  initial_gains.values[0].d = 2;
  initial_gains.values[1].p = 0;
  initial_gains.values[1].d = 2;
  initial_gains.values[2].p = 0;
  initial_gains.values[2].d = 2;
  allowRotation(false);

  ml_SetGainVecAxes(device_hdl, ML_GAINSET_TYPE_NORMAL, initial_gains);
  ml_ConstrainAxis(device_hdl, ML_AXIS_3, -7, 7);
  ml_ConstrainAxis(device_hdl, ML_AXIS_4, -7, 7);
  ml_ConstrainAxis(device_hdl, ML_AXIS_5, -7, 7);
}

void MLHIHapticsDevice::storeInDevices()
{
  devices.insert(
    pair<ml_device_handle_t, MLHIHapticsDevice*>(device_hdl, this));
}

void MLHIHapticsDevice::removeFromDevices()
{
  devices.erase(device_hdl);
}

void MLHIHapticsDevice::registerCallbacks()
{
  ml_RegisterCallbackTick(device_hdl, tickCallback);
  ml_RegisterCallbackButtonPressed(device_hdl, buttonCallback);
  ml_RegisterCallbackFault(device_hdl, checkFaultCallback);
}

void MLHIHapticsDevice::unregisterCallbacks()
{
  ml_UnregisterCallbackTick(device_hdl);
  ml_UnregisterCallbackButtonPressed(device_hdl);
  ml_UnregisterCallbackFault(device_hdl);
}

void MLHIHapticsDevice::updateButtonStatus(int _button_status)
{
  button_status = _button_status;
}

void MLHIHapticsDevice::updatePositionAndVelocity(ml_position_t position)
{
  Vec3 new_position = Vec3( position.values[0],
    position.values[2],
    -position.values[1] );
  Rotation new_rotation = Rotation( Vec3(5 * position.values[3],
    5 * position.values[5],
    -5 * position.values[4]) );
  position_and_velocity_lock.lock();
  // Calculating velocity this way causes instability.
  // I am setting it to zero for now.
  velocity = Vec3(0,0,0); //(newPosition - currentPosition) * 1000; // m/s
  //cerr << "vel: " << velocity << endl;
  current_position = new_position;
  current_rotation = new_rotation;
  position_and_velocity_lock.unlock();
}

// Callback functions

int MLHIHapticsDevice::checkFaultCallback( ml_device_handle_t device_hdl,
                                          ml_fault_t fault )
{
  if ((fault.value & ML_FAULT_TYPE_SENSOR_OUT_OF_RANGE) != 0)
  {
    cerr << "ML_FAULT_TYPE_SENSOR_OUT_OF_RANGE" << endl;
    fault.value &= ~ML_FAULT_TYPE_SENSOR_OUT_OF_RANGE;
  }
  if ((fault.value & ML_FAULT_TYPE_COIL_OVERTEMP) != 0)
  {
    cerr << "ML_FAULT_TYPE_COIL_OVERTEMP" << endl;
    fault.value &= ~ML_FAULT_TYPE_COIL_OVERTEMP;
  }
  if ((fault.value & ML_FAULT_TYPE_COIL_OVERCURRENT) != 0)
  {
    cerr << "ML_FAULT_TYPE_COIL_OVERCURRENT" << endl;
    fault.value &= ~ML_FAULT_TYPE_COIL_OVERCURRENT;
  }
  if ((fault.value & ML_FAULT_TYPE_FLOTOR_OVERSPEED) != 0)
  {
    cerr << "ML_FAULT_TYPE_FLOTOR_OVERSPEED" << endl;
    fault.value &= ~ML_FAULT_TYPE_FLOTOR_OVERSPEED;
  }

  if (fault.value)
  {
    cerr << "UNRECOGNIZED FAULT!" << endl;
  }

  return -1;
}

int MLHIHapticsDevice::tickCallback(ml_device_handle_t device_hdl,
                                    ml_position_t* position)
{
  MLHIHapticsDevice* dev = MLHIHapticsDevice::getDeviceByHandle(device_hdl);
  dev->renderForces(*position);
  dev->updatePositionAndVelocity(*position);
  return 0;
}

int MLHIHapticsDevice::buttonCallback(ml_device_handle_t device_hdl,
                                      ml_button_t button)
{
  MLHIHapticsDevice::getDeviceByHandle(device_hdl)->
    updateButtonStatus(button.left + 2 * button.right);
  return 0;
}

#endif
