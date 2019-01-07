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
/// \file HAPIVariableDepthSurface.h
/// \brief Header file for HAPIVariableDepthSurface
///
//
//////////////////////////////////////////////////////////////////////////////
#ifndef __HAPIVARIABLEDEPTHSURFACE_H__
#define __HAPIVARIABLEDEPTHSURFACE_H__

#include <HAPI/HAPIFrictionSurface.h>
#include <H3DUtil/Image.h>
#include <H3DUtil/AutoRef.h>

namespace HAPI {
  /// \ingroup AbstractClasses
  /// \class HAPIVariableDepthSurface
  /// \brief Base class for all surfaces which has a "depth".
  ///
  /// Sufficient force is needed by the user to go from a deep area to a
  /// shallow area on the surface. The depth is calculated by a function
  /// using local space of the point of contact.
  /// For OpenHapticsRenderer this surface will not work.
  class HAPI_API HAPIVariableDepthSurface: public HAPIFrictionSurface {

  public:

    // Constructor
    HAPIVariableDepthSurface(
          HAPIFloat _stiffness = 0.35,
          HAPIFloat _damping = 0,
          HAPIFloat _static_friction = 0.1,
          HAPIFloat _dynamic_friction = 0.4,
          HAPIFloat (*_func)( const Vec2 &local_point, void *data ) = 0,
          int _max_iterations = 35,
          HAPIFloat _minimization_epsilon = 1e-4,
          bool _use_relative_values = true,
          bool _use_ref_count_lock = true );


    /// Calculates the movement of the proxy depending on the depth returned
    /// from getDepth() function and a minimization algorithm. Friction 
    /// is also applied if the friction parameters are above 0.
    virtual void getProxyMovement( HAPISurfaceObject::ContactInfo &contact );

    /// Calculates the force on the haptics device depending on the depth
    /// returned from getDepth() function.
    virtual void getForces( HAPISurfaceObject::ContactInfo &contact );

  protected:
    /// Lock used when getting depth for the surface.
    H3DUtil::MutexLock depth_get_lock;

    /// Function to minimize to find out where to move proxy.
    static HAPIFloat localPtToDist( Vec2 local_point, void *user_data );

    inline HAPIFloat getDepth( const Vec2 &local_point ) {
      if( depth_invert )
        return -func( local_point, this );
      else
        return func( local_point, this );
    }

    /// Function used to calculate depth on the surface.
    /// \param local_point is a point in local coordinates around the point
    /// of contact. Transformation information is taken from ContactInfo.
    /// \param data User data, in this case a pointer to
    /// HAPIVariableDepthSurface.
    HAPIFloat (*func)( const Vec2 &local_point, void *data );

    /// Needed by amoeba minimization function
    HAPIFloat minimization_epsilon;
    /// Needed by amoeba minimization function
    int max_iterations;

    /// Can be used by the depth function to access the current ContactInfo
    /// class.
    ContactInfo *this_contact_info;
    bool depth_invert;
  };
}

#endif
