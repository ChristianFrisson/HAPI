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
#ifndef __INTERSECTIONINFO_H__
#define __INTERSECTIONINFO_H__
#include <HAPITypes.h>

namespace HAPI {

  namespace Bounds {

    //class GeometryPrimitive;
     typedef enum {
        BACK,
        FRONT,
        FRONT_AND_BACK
      } FaceType;

    /// \brief The IntersectionInfo struct contains information about an 
    /// intersection.
    struct IntersectionInfo {
     

      /// Constructor.
      IntersectionInfo( const Vec3 &_point = Vec3(), 
                        const Vec3 &_normal = Vec3(),
                        const Vec3 &_tex_coord = Vec3(),
                        FaceType intersected_face = FRONT,
                        int _id = -1 ) :
      point( _point ),
      normal( _normal ),
      tex_coord( _tex_coord ),
      id( _id ) { intersection = false; }

      /// The intersection point.
      Vec3 point;

      /// The normal at the intersection point.
      Vec3 normal;

      /// The texture coordinate at the intersection point.
      Vec3 tex_coord;

      /// The face that was intersected. BACK or FRONT.
      FaceType face;

      bool intersection;

      /// Gradient describing the change of tex coord depending on
      /// change in pos at the intersection point.
      Matrix3 pos_to_tex_coord_gradient;
      
      /// \brief The id of the primitive that was intersected if applicable,
      /// e.g. triangle index. -1 if no id exists.
      int id;
    };
  }
}
#endif
