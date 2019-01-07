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
/// \file HAPIFrictionSurface.h
/// \brief Header file for HAPIFrictionSurface
///
//
//////////////////////////////////////////////////////////////////////////////
#ifndef __HAPIFRICTIONSURFACE_H__
#define __HAPIFRICTIONSURFACE_H__

#include <HAPI/HAPISurfaceObject.h>

namespace HAPI {
  /// \ingroup AbstractClasses
  /// \class HAPIFrictionSurface
  /// \brief Base class for surfaces that have friction parameters.
  /// Contains some functions that could be used for calculating
  /// friction behaviour.
  class HAPI_API HAPIFrictionSurface : public HAPISurfaceObject {
  public:

    /// Constructor
    HAPIFrictionSurface( HAPIFloat _stiffness = 0.35,
                         HAPIFloat _damping = 0,
                         HAPIFloat _static_friction = 0,
                         HAPIFloat _dynamic_friction = 0,
                         bool _use_relative_values = true,
                         bool _use_ref_count_lock = true );

    /// The stiffness of the surface.
    HAPIFloat stiffness;

    /// The damping parameter of the surface.
    HAPIFloat damping;
    
    /// The static friction of the surface.
    HAPIFloat static_friction;
    
    /// The dynamic friction of the surface
    HAPIFloat dynamic_friction;

    /// If false then values (such as stiffness) is in absolute values with
    /// SI units or equivalent. If true the units are relative to the maximum
    /// values that the haptics device, on which the surface is rendered on,
    /// can handle.
    bool use_relative_values;

    /// Common way of calculating the force for a friction surface.
    /// Can be used by subclasses to HAPIFrictionSurface.
    /// \param contact_info A struct containing info about the contact on the
    /// surface
    /// \param point_on_surface The point towards which the force should be
    /// calculated, quite often this is the point of the virtual proxy.
    /// \param stiffness The stiffness parameter.
    /// \param damping The damping parameter.
    /// \param use_relative_values Same functionality as
    /// HAPIFrictionSurface::use_relative_values.
    static void getForcesInternal( ContactInfo &contact_info,
                                   const Vec3 &point_on_surface,
                                   HAPIFloat stiffness,
                                   HAPIFloat damping,
                                   bool use_relative_values );

  protected:
    /// Used to determine if the proxy is in static contact with
    /// the surface. In that case there is no proxy movement.
    bool in_static_contact;

    /// Common way of calculating the proxy movement for a friction surface.
    /// If friction parameters are non-zero. This function is deprecated, use
    /// the static public function setLocalProxyMovement instead.
    /// Can be used by subclasses to HAPIFrictionSurface.
    /// \param contact_info A struct containing info about the contact on the
    /// surface
    /// \param t_force_length The magnitude of the tangential force.
    /// \param n_force_length The magnitude of the normal force.
    /// \param proxy_movement The estimated proxy movement.
    /// \param _static_friction The static friction parameter.
    /// \param _dynamic_friction The dynamic friction parameter.
    /// \param proxy_mov_length The magnitude of the proxy movement. This
    /// parameter can be omitted if the function should calculate the length.
    void setLocalProxyMovement( ContactInfo &contact_info,
                                HAPIFloat t_force_length,
                                HAPIFloat n_force_length,
                                Vec2 &proxy_movement,
                                HAPIFloat _static_friction,
                                HAPIFloat _dynamic_friction,
                                HAPIFloat *proxy_mov_length = 0 );

    /// Common way of calculating the proxy movement for a friction surface.
    /// If friction parameters are non-zero. THIS FUNCTION IS DEPRECATED, use
    /// the other version of the function setLocalProxyMovement instead.
    /// Can be used by subclasses to HAPIFrictionSurface.
    /// \param contact_info A struct containing info about the contact on the
    /// surface
    /// \param t_force_length The magnitude of the tangential force.
    /// \param n_force_length The magnitude of the normal force.
    /// \param proxy_movement The estimated proxy movement.
    /// \param have_proxy_mov_length Should be true if the next parameter is
    /// set.
    /// \param proxy_mov_length The magnitude of the proxy movement, used if
    /// have_proxy_mov_length is true.
    void setLocalProxyMovement( ContactInfo &contact_info,
                                HAPIFloat t_force_length,
                                HAPIFloat n_force_length,
                                Vec2 &proxy_movement,
                                bool have_proxy_mov_length = false,
                                HAPIFloat proxy_mov_length = 0 );

    /// Common way of calculating the force for a friction surface.
    /// Can be used by subclasses to HAPIFrictionSurface. THIS FUNCTION IS
    /// DEPRECATED, use the static public function getForcesInternal instead.
    /// \param contact_info A struct containing info about the contact on the
    /// surface
    /// \param point_on_surface The point towards which the force should be
    /// calculated, quite often this is the point of the virtual proxy.
    void getForcesInternal( ContactInfo &contact_info,
                            const Vec3 &point_on_surface );
  };
}

#endif
