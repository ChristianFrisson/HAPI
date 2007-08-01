//////////////////////////////////////////////////////////////////////////////
//    Copyright 2004-2007, SenseGraphics AB
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
// TODO: 
// Base code that this code was built upon was contributed by .....
//
//
/// \file CollisionStructures.h
/// \brief Header file for CollisionStructures, 
///
//
//////////////////////////////////////////////////////////////////////////////
#ifndef __PLANECONSTRAINT_H__
#define __PLANECONSTRAINT_H__

#include <IntersectionInfo.h>
#include <HAPIHapticShape.h>

namespace HAPI {

  namespace Bounds {

    class GeometryPrimitive;

    class HAPI_API PlaneConstraint {

    public:
      PlaneConstraint( const Vec3 &p, const Vec3 &n, const Vec3 &tc, 
        HAPIHapticShape *shape = NULL,
        GeometryPrimitive *_primitive = NULL ):
      point( p ), normal( n ), tex_coord( tc ), 
        haptic_shape( shape ), primitive( _primitive )  {}

      bool lineIntersect( const Vec3 &from, 
        const Vec3 &to,    
        Bounds::IntersectionInfo &result );
      Vec3 point, normal;
      Vec3 tex_coord;
      H3DUtil::AutoRef< HAPIHapticShape > haptic_shape;
      GeometryPrimitive * primitive;
    };
  }
}
#endif
