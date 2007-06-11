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
//
//
/// \file HapticTriangle.h
/// \brief Header file for HapticTriangle
///
//
//////////////////////////////////////////////////////////////////////////////
#ifndef __HAPTICTRIANGLE_H__
#define __HAPTICTRIANGLE_H__

#include <HAPIHapticShape.h>

namespace HAPI {

  /// \brief Base class for haptic shapes, i.e. geometrical objects that are
  /// rendered by letting their surfaces constrain the proxy. A HapticShape
  /// has a Surface object associated to it that defines the properties of 
  /// the surface, e.g. stiffness and friction properties.
  ///
  class HAPI_API HapticTriangle: public HAPIHapticShape {
  public:
    /// Constructor.
    HapticTriangle( const Bounds::Triangle &_triangle,
                    void *_userdata,
                    HAPISurfaceObject *_surface,
                    const Matrix4 & _transform,
                    int _shape_id = -1,
                    Bounds::FaceType _touchable_face = 
                    Bounds::FRONT_AND_BACK ):
      HAPIHapticShape( _userdata, _surface, _transform, 
                       _shape_id, _touchable_face  ),
      triangle( _triangle ) {}
    
    /// Get the closest point and normal on the object to the given point p.
    /// \param p The point to find the closest point to.
    /// \param closest_point Return parameter for closest point
    /// \param normal Return parameter for normal at closest point.
    virtual void closestPoint( const Vec3 &p,
                               Vec3 &closest_point,
                               Vec3 &normal,
                               Vec3 &tex_coord ) {
       return triangle.closestPoint( p, closest_point, normal, tex_coord );
    }

    /// Detect collision between a line segment and the object.
    /// \param from The start of the line segment.
    /// \param to The end of the line segment.
    /// \param result Contains info about the closest intersection, if 
    /// line intersects object
    /// \returns true if intersected, false otherwise.
    inline virtual bool lineIntersect( const Vec3 &from, 
                                       const Vec3 &to,
                                       Bounds::IntersectionInfo &result,
                                       Bounds::FaceType face = 
                                       Bounds::FRONT_AND_BACK ) {
      return triangle.lineIntersect( from, to, result, face );
    }

    inline virtual void getConstraints( const Vec3 &point,
                                        std::vector< PlaneConstraint > &constraints,
                                        Bounds::FaceType face = 
                                        Bounds::FRONT_AND_BACK ) {
      Vec3 p = transform.inverse() * point;
      unsigned int size = constraints.size();
      triangle.getConstraints( p, constraints, face );
      for( unsigned int i = size; i < constraints.size(); i ++ ) {
        PlaneConstraint &pc = constraints[i];
        pc.point = transform * pc.point;
        pc.normal = transform.getRotationPart() * pc.normal;
      }
    }

    inline virtual void glRender() {
      glMatrixMode( GL_MODELVIEW );
      glPushMatrix();
      const Matrix4 &m = transform;
      GLdouble vt[] = { m[0][0], m[1][0], m[2][0], 0,
                        m[0][1], m[1][1], m[2][1], 0,
                        m[0][2], m[1][2], m[2][2], 0,
                        m[0][3], m[1][3], m[2][3], 1 };
      glMultMatrixd( vt );
      glBegin( GL_TRIANGLES );
      glVertex3d( triangle.a.x, triangle.a.y, triangle.a.z );
      glVertex3d( triangle.b.x, triangle.b.y, triangle.b.z );
      glVertex3d( triangle.c.x, triangle.c.y, triangle.c.z );
      glEnd();
      glPopMatrix();
    }

    inline virtual int nrTriangles() {
      return 1;
    }
  protected:
    Bounds::Triangle triangle; 
  };
}

#endif
