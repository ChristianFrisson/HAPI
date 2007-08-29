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

#include "FrictionSurface.h"

using namespace HAPI;

FrictionSurface::FrictionSurface( HAPIFloat _stiffness,
                                  HAPIFloat _damping,
                                  HAPIFloat _static_friction,
                                  HAPIFloat _dynamic_friction ) :
stiffness( _stiffness ),
damping( _damping ),
static_friction( _static_friction ),
dynamic_friction( _dynamic_friction ),
in_static_contact( true ) {
}

FrictionSurface::~FrictionSurface() {
};


void FrictionSurface::getProxyMovement( ContactInfo &contact_info ) {
  if( H3DUtil::H3DAbs( static_friction ) < Constants::epsilon &&
      H3DUtil::H3DAbs( dynamic_friction ) < Constants::epsilon ) {
    // SmoothSurface
    Vec3 local_probe = contact_info.localProbePosition();
    contact_info.proxy_movement_local = Vec2( local_probe.x , local_probe.z );
  } else {
    // FrictionalSurface
    Vec3 local_probe = contact_info.localProbePosition();
    Vec3 force = local_probe * -stiffness;
    Vec2 force_t( force.x, force.z );

    if( in_static_contact ) {
      if( force_t.length() <= static_friction *  force.y ) {
        contact_info.proxy_movement_local = Vec2( 0, 0 );
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
        contact_info.proxy_movement_local = Vec2( 0, 0 ); 
      } else {
        HAPIFloat max_movement = velocity * 1e-3;
        Vec2 proxy_movement = Vec2( local_probe.x, local_probe.z );
        // TODO: review this algorithm, is this part needed?
        /*HAPIFloat l = proxy_movement.length();
        if( l > max_movement ) {
        proxy_movement *= max_movement / l; 
        }*/
        contact_info.proxy_movement_local = proxy_movement;
      }
    }
  }
}

void FrictionSurface::getForces( ContactInfo &contact_info ) {
  Vec3 probe_to_origin = 
    contact_info.globalOrigin() - contact_info.globalProbePosition();
  contact_info.setGlobalForce(  probe_to_origin * stiffness );
}
