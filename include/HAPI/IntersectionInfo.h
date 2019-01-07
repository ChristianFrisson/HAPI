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
/// \file IntersectionInfo.h
/// \brief Header file for IntersectionInfo
///
//
//////////////////////////////////////////////////////////////////////////////
#ifndef __INTERSECTIONINFO_H__
#define __INTERSECTIONINFO_H__
#include <HAPI/HAPITypes.h>

namespace HAPI {
  /// Contains simple collision classes.
  namespace Collision {

    class GeometryPrimitive;

     typedef enum {
        BACK,
        FRONT,
        FRONT_AND_BACK
      } FaceType;

    /// \struct IntersectionInfo
    /// \brief The IntersectionInfo struct contains information about an 
    /// intersection point in collision detection routines.
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
      intersection( false ),
      t( 0 ),
      id( _id ),
      face( intersected_face ),
      primitive( NULL ) {}

      /// The intersection point.
      Vec3 point;

      /// The normal at the intersection point.
      Vec3 normal;

      /// The texture coordinate at the intersection point.
      Vec3 tex_coord;

      /// The face that was intersected. BACK or FRONT.
      FaceType face;

      /// True if IntersectionInfo has been used when an intersection
      /// was detected. This can be used as an indication that some/all of
      /// the properties of the IntersectionInfo has been set to something
      /// useful.
      bool intersection;

      /// Point along a line where intersection occurs. Used by collision
      /// functions that detects collision along a line
      /// ( lineIntersect, movingSphereIntersect ).
      HAPIFloat t;

      /// The primitive that was intersected.
      GeometryPrimitive * primitive;
      
      /// \brief The id of the primitive that was intersected if applicable,
      /// e.g. triangle index. -1 if no id exists.
      int id;
    };
  }
}
#endif
