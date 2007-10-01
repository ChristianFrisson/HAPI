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
/// \file FrictionSurface.h
/// \brief Header file for FrictionSurface
///
//
//////////////////////////////////////////////////////////////////////////////
#ifndef __FRICTIONSURFACE_H__
#define __FRICTIONSURFACE_H__

#include <HAPISurfaceObject.h>

namespace HAPI {

  class HAPI_API FrictionSurface : public HAPISurfaceObject {
  public:

    // default parameter approximately the same as for SmoothSurface in H3DAPI.
    FrictionSurface( HAPIFloat _stiffness = 0.35,
                    HAPIFloat _damping = 0,
                    HAPIFloat _static_friction = 0,
                    HAPIFloat _dynamic_friction = 0,
                    bool _use_ref_count_lock = true );

    virtual ~FrictionSurface();

    virtual void getProxyMovement( ContactInfo &contact_info );
    virtual void getForces( ContactInfo &contact_info );

    HAPIFloat stiffness, damping, static_friction, dynamic_friction;
  protected:
    bool in_static_contact;
  };
}

#endif
