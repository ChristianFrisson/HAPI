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
/// \file HAPIHapticShape.h
/// \brief Header file for HAPIHapticShape
///
//
//////////////////////////////////////////////////////////////////////////////
#ifndef __HAPIHAPTICSHAPE_H__
#define __HAPIHAPTICSHAPE_H__

#include <HAPIHapticObject.h>
#include <HAPISurfaceObject.h>
#include <RefCountedClass.h>
#include <CollisionObjects.h>
#include <vector>

namespace H3D {


  /// Base class for haptic shapes, i.e. geometrical objects that are rendered
  /// by letting their surfaces constrain the proxy. A HAPIHapticShape has a Surface
  /// object associated to it that defines the properties of the surface, e.g.
  /// stiffness and friction properties.
  ///
  class HAPI_API HAPIHapticShape: public HAPIHapticObject,
                                  public Bounds::CollisionObject {
  public:
    /// Constructor.
   HAPIHapticShape( void *_userdata,
                     HAPISurfaceObject *_surface,
                     const H3D::ArithmeticTypes::Matrix4f & _transform ):
      HAPIHapticObject( _transform ),
      surface( _surface ),
      userdata( _userdata ) {}
    
    virtual Vec3d closestPoint( const Vec3d &p ) {
      return Vec3d();
    }

    virtual bool lineIntersect( const Vec3d &from, 
                                const Vec3d &to,
                                Bounds::IntersectionInfo &result ) { 
      return false;
    }

    /// The Surface object describing the properties of the surface.
    HAPISurfaceObject *surface;

    void *userdata;
      
  };
}

#endif
