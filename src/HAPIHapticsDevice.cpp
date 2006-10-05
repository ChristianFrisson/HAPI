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
/// \file HAPIHapticsDevice.cpp
/// \brief Cpp file for HAPIHapticsDevice.
///
//
//////////////////////////////////////////////////////////////////////////////

#include <HAPIHapticsDevice.h>

using namespace H3D;


// Callback function for rendering force effects on the 
// HLHapticsDevice.  
PeriodicThread::CallbackCode HAPIHapticsDevice::hapticRenderingCallback( void *data ) {
  HAPIHapticsDevice *hd = 
    static_cast< HAPIHapticsDevice * >( data );
  hd->shape_lock.lock();
  hd->current_shapes = hd->tmp_shapes;
  hd->shape_lock.unlock();

  hd->updateDeviceValues();
  
  DeviceValues dv = hd->getDeviceValues();
  
  Rotation rot = dv.orientation;
  Vec3d pos = dv.position;
  Vec3d vel = dv.velocity;
  H3DInt32 b = dv.button_status;
  
  TimeStamp now = TimeStamp();
  TimeStamp dt = now - hd->last_loop_time;
  hd->last_loop_time = now;
  HapticForceEffect::EffectInput input( pos, vel, rot, dt );
  HapticForceEffect::EffectOutput output;
  
  // calculate the forces generated by the force effects from the
  // last loop that are to be interpolated. 
  for( HapticEffectVector::const_iterator i = hd->last_force_effects.begin();
       i != hd->last_force_effects.end();
       i++ ) {
    if( (*i)->isInterpolated() )
      output = output + (*i)->calculateForces( input );
  }
  
  double weighting = 1; //dt / hd->last_loop_time;
  if( weighting > 1 ) weighting = 1;
  // the previous force effects are to be decreased as time goes by.
  output = output * ( 1 - weighting );
  
  // calculate the forces generated by the active force effects
  for( HapticEffectVector::const_iterator i = hd->current_force_effects.begin();
       i != hd->current_force_effects.end();
       i++ ) {
    if( (*i)->isInterpolated() )
      output = output + (*i)->calculateForces( input ) * weighting;
    else
      output = output + (*i)->calculateForces( input );
  }

  if( hd->haptics_renderer.get() )
    output = output + hd->haptics_renderer->renderHapticsOneStep( input, hd->current_shapes );

  // add the resulting force and torque to the rendered force.
  hd->sendForce( output.force );
  hd->sendTorque( output.torque );

  hd->sendOutput();
  return PeriodicThread::CALLBACK_CONTINUE;
}

