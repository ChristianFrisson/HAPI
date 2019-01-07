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
HAPIFrictionSurface( _stiffness, _damping, _static_friction,
                     _dynamic_friction, _use_relative_values,
                     _use_ref_count_lock ) {
}

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
    //This part is needed to compile on linux.
    Vec2 local_probe_2d =  Vec2( local_probe.x, local_probe.z );

    setLocalProxyMovement( contact_info,
                           force_t.length(),
                           force.y,
                           local_probe_2d,
                           static_friction,
                           dynamic_friction );

  }
}

void FrictionSurface::getForces( ContactInfo &contact_info ) {
  getForcesInternal( contact_info, contact_info.globalOrigin(),
                     stiffness, damping, use_relative_values );
}
