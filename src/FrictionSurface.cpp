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
/// \file FrictionSurface.cpp
/// \brief cpp file for FrictionSurface
///
//
//////////////////////////////////////////////////////////////////////////////

#include <HAPI/FrictionSurface.h>
#include <HAPI/HAPIHapticsDevice.h>

using namespace HAPI;

FrictionSurface::FrictionSurface( HAPIFloat _stiffness,
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

FrictionSurface::~FrictionSurface() {
};


void FrictionSurface::getProxyMovement( ContactInfo &contact_info ) {
  if( H3DUtil::H3DAbs( static_friction ) < Constants::epsilon &&
      H3DUtil::H3DAbs( dynamic_friction ) < Constants::epsilon ) {
    // SmoothSurface
    Vec3 local_probe = contact_info.localProbePosition();
    contact_info.setLocalProxyMovement( Vec2( local_probe.x , local_probe.z ) );
  } else {
    // FrictionalSurface
    // Friction model calculation modeled after the information in
    // section 4 of "Haptic Interaction in Virtual Environments" by
    // Diego C. Ruspini, Krasimir Kolarov and Oussama Khatib.
    // Published 1997.
    Vec3 local_probe = contact_info.localProbePosition();
    HAPIFloat local_stiffness = stiffness;
    if( use_relative_values ) {
      local_stiffness = stiffness *
        contact_info.hapticsDevice()->getMaxStiffness();
    }
    Vec3 force = local_probe * -local_stiffness;
    Vec2 force_t( force.x, force.z );

    if( in_static_contact ) {
      // Determine if the movement should start.
      if( force_t.length() <= static_friction *  force.y ) {
        contact_info.setLocalProxyMovement(Vec2( 0, 0 ));
      } else {
        in_static_contact = false;
      }
    } 

    if( !in_static_contact ) {
      HAPIFloat b = 1;
      HAPIFloat dt = 1e-3;
      HAPIFloat velocity = 
        ( force_t.length() - dynamic_friction * force.y ) / b;

      if( velocity < Constants::epsilon ) {
        in_static_contact = true;
        velocity = 0;
        contact_info.setLocalProxyMovement( Vec2( 0, 0 ) ); 
      } else {
        // The max_movement should be in m. The velocity gotten is in m/s.
        // The maximum movement in one haptic frame is therefore
        // velocity * 0.001 where 0.001 is the time in seconds for one haptic
        // frame. TODO: make this correct even for other frame rates.
        HAPIFloat max_movement = velocity * dt;
        Vec2 proxy_movement = Vec2( local_probe.x, local_probe.z );
        HAPIFloat l = proxy_movement.length();
        if( l > max_movement ) {
        proxy_movement *= max_movement / l; 
        }
        contact_info.setLocalProxyMovement( proxy_movement );
      }
    }
  }
}

void FrictionSurface::getForces( ContactInfo &contact_info ) {
  Vec3 probe_to_origin = 
    contact_info.globalOrigin() - contact_info.globalProbePosition();

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
