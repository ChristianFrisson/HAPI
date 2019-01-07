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
/// \file HLCustomObject.h
/// \brief Header file for HLCustomObject
///
//
//////////////////////////////////////////////////////////////////////////////

#ifndef __HLCUSTOMOBJECT_H__
#define __HLCUSTOMOBJECT_H__

#include <HAPI/Config.h>
#include <HAPI/OpenHapticsRenderer.h>

#ifdef HAVE_OPENHAPTICS

namespace HAPI {

  /// \ingroup OpenHapticsRenderer
  /// \class HLCustomObject
  /// \brief Base class for objects that use HLAPI custom shape callbacks to
  /// render itself.
  ///
  class OPENHAPTICSRENDERER_API HLCustomObject: 
    public HAPIHapticShape, 
    public OpenHapticsRenderer::HLShape {
  public:

    /// Intersect the line segment from start_point to end_point with
    /// the object.  
    /// \param start_point The start point of the line segment.
    /// \param end_point The end point of the line segment.
    /// \param intersection_point Return parameter that should be set to the 
    /// point of intersection between the line segment and object.
    /// \param intersection_normal Return parameter that should be set to the 
    /// normal of the surface of the objet at the intersection point.
    /// \param face Return parameter that should be set to HL_FRONT if the front
    /// of the surface is touched and to HL_BACK if the backside of the surface
    /// is touched. 
    /// \returns true if there is an intersection, false otherwise.
    ///
    virtual bool intersectSurface( const Vec3 &start_point, 
                                   const Vec3 &end_point,
                                   Vec3 &intersection_point, 
                                   Vec3 &intersection_normal,
                                   HLenum &face ) = 0;

    /// Find the closest point to query_point on the surface of the
    /// object. 
    /// \param query_point The point to find the closest point to.
    /// \param target_point The haptic device position.
    /// \param geom Return parameter to return a hlLocalFeature
    /// \param closest_point Return parameter that should be set to the closest
    /// point to query_point.
    /// \returns true if closest point was found.
    virtual bool closestFeature( const Vec3 &query_point, 
                                 const Vec3 &target_point,
                                 HLgeom *geom,
                                 Vec3 &closest_point ) = 0;

    /// This function performs all the HLAPI calls that are needed to render
    /// the object. It sets the object up as a HL_SHAPE_CALLBACK shape and uses
    /// the instersectSurface and closestPointOnSurface function in the callback
    /// functions.
    /// \param hd The haptics device for which the surface should be rendered.
    /// \param _shape_id The HL-id for the shape.
    virtual void hlRender( HAPIHapticsDevice *hd,
                           HLuint _shape_id );

  protected:
    // Callback function for finding the intersection between a line segment
    // and the object. Used in hlRender. 
    //
    static HLboolean HLCALLBACK intersectCallback( 
                                      const HLdouble *start_point, 
                                      const HLdouble *end_point,
                                      HLdouble *intersection_point, 
                                      HLdouble *intersection_normal,
                                      HLenum* face,
                                      void *user_data ) {
      HLCustomObject* object = static_cast<HLCustomObject*>( user_data );
      Vec3 ip, in;
      HLboolean b = object->intersectSurface( Vec3( start_point[0], 
                                                    start_point[1], 
                                                    start_point[2] ), 
                                              Vec3( end_point[0],
                                                    end_point[1],
                                                    end_point[2] ),
                                              ip, 
                                              in, 
                                              *face );
      intersection_point[0] = ip.x;
      intersection_point[1] = ip.y;
      intersection_point[2] = ip.z;

      intersection_normal[0] = in.x;
      intersection_normal[1] = in.y;
      intersection_normal[2] = in.z;

      return b;
    }

    // Callback function for finding the closest point on the object. Used
    // in hlRender.
    //
    static HLboolean HLCALLBACK closestFeaturesCallback( 
                                       const HLdouble *query_point, 
                                       const HLdouble *target_point, 
                                       HLgeom *geom,
                                       HLdouble *closest_point,
                                       void* user_data ) {
      HLCustomObject* object = static_cast<HLCustomObject*>( user_data );

      Vec3 cp, cn;
      HLboolean b = object->closestFeature( Vec3( query_point[0],
                                                  query_point[1],
                                                  query_point[2] ),
                                            Vec3( target_point[0],
                                                  target_point[1],
                                                  target_point[2]),
                                            geom,
                                            cp );
      closest_point[0] = cp.x;
      closest_point[1] = cp.y;
      closest_point[2] = cp.z;

      return b;
    }

  };
}

#endif

#endif
