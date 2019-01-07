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
/// \file HAPIFrictionSurface.cpp
/// \brief cpp file for HAPIFrictionSurface
///
//
//////////////////////////////////////////////////////////////////////////////

#include <HAPI/HAPIFrictionSurface.h>
#include <HAPI/HAPIHapticsDevice.h>

using namespace HAPI;

HAPIFrictionSurface::HAPIFrictionSurface( HAPIFloat _stiffness,
                                          HAPIFloat _damping,
                                          HAPIFloat _static_friction,
                                          HAPIFloat _dynamic_friction,
                                          bool _use_relative_values,
                                          bool _use_ref_count_lock ) :
HAPISurfaceObject( _use_ref_count_lock ),
stiffness( _stiffness ),
damping( _damping ),
static_friction( _static_friction ),
dynamic_friction( _dynamic_friction ),
use_relative_values( _use_relative_values ),
in_static_contact( true ) {
}

void HAPIFrictionSurface::setLocalProxyMovement( ContactInfo &contact_info,
                                                 HAPIFloat t_force_length,
                                                 HAPIFloat n_force_length,
                                                 Vec2 &proxy_movement,
                                                 HAPIFloat _static_friction,
                                                 HAPIFloat _dynamic_friction,
                                                 HAPIFloat *proxy_mov_length ) {
  if( in_static_contact ) {
    // Determine if the movement should start.
    if( t_force_length <= _static_friction * n_force_length ) {
      contact_info.setLocalProxyMovement(Vec2( 0, 0 ));
    } else {
      in_static_contact = false;
    }
  } 

  if( !in_static_contact ) {
    HAPIFloat b = 1;
    HAPIFloat dt = 1.0 / contact_info.hapticsDevice()->getHapticsRate();
    HAPIFloat velocity = 
      ( t_force_length - _dynamic_friction * n_force_length ) / b;

    if( velocity < Constants::epsilon ) {
      // If the velocity is negative then the dynamic friction term is
      // sufficient to resist all movement and therefore we do not move the
      // proxy.
      in_static_contact = true;
      velocity = 0;
      contact_info.setLocalProxyMovement( Vec2( 0, 0 ) ); 
    } else {
      // The max_movement should be in m. The velocity is calculated in m/s.
      // The maximum movement in one haptic frame is therefore
      // velocity * 0.001 where 0.001 is the time in seconds for one haptic
      // frame.
      HAPIFloat max_movement = velocity * dt;
      if( !proxy_mov_length ) {
        HAPIFloat tmp_proxy_mov_length = proxy_movement.length();
        if( tmp_proxy_mov_length > max_movement )
          proxy_movement *= max_movement / tmp_proxy_mov_length;
      }
      else if( *proxy_mov_length > max_movement ) {
        proxy_movement *= max_movement / *proxy_mov_length;
      }
      contact_info.setLocalProxyMovement( proxy_movement );
    }
  }
}

void HAPIFrictionSurface::getForcesInternal( ContactInfo &contact_info,
                                             const Vec3 &point_on_surface,
                                             HAPIFloat stiffness,
                                             HAPIFloat damping,
                                             bool use_relative_values ) {
  Vec3 probe_to_origin = point_on_surface - contact_info.globalProbePosition();

  Vec3 n_probe_to_origin = probe_to_origin;
  n_probe_to_origin.normalizeSafe();
  HAPIFloat local_stiffness = stiffness;
  if( use_relative_values ) {
    local_stiffness = stiffness *
      contact_info.hapticsDevice()->getMaxStiffness();
  }
  contact_info.setGlobalForce(  probe_to_origin * local_stiffness -
                                ( n_probe_to_origin *
                                  contact_info.globalProbeVelocity() *
                                  damping ) * n_probe_to_origin );
}


void HAPIFrictionSurface::setLocalProxyMovement( ContactInfo &contact_info,
                                                 HAPIFloat t_force_length,
                                                 HAPIFloat n_force_length,
                                                 Vec2 &proxy_movement,
                                                 bool have_proxy_mov_length,
                                                 HAPIFloat proxy_mov_length ) {
  setLocalProxyMovement( contact_info, t_force_length, n_force_length,
                         proxy_movement, static_friction, dynamic_friction,
                         &proxy_mov_length );
}

void HAPIFrictionSurface::getForcesInternal( ContactInfo &contact_info,
                                             const Vec3 &point_on_surface ) {
  getForcesInternal( contact_info, point_on_surface, stiffness,
                     damping, use_relative_values );
}

