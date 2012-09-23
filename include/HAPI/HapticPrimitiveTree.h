//////////////////////////////////////////////////////////////////////////////
//    Copyright 2004-2007, SenseGraphics AB
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
/// \file HapticPrimitiveTree.h
/// \brief Header file for HapticPrimitiveTree
///
//
//////////////////////////////////////////////////////////////////////////////
#ifndef __HAPTICPRIMITIVETREE_H__
#define __HAPTICPRIMITIVETREE_H__

#include <HAPI/HAPIHapticShape.h>
#include <H3DUtil/AutoRefVector.h>

namespace HAPI {

  /// \ingroup Shapes
  /// \class HapticPrimitiveTree
  /// \brief A shape defined by a set of the class GeometryPrimitive
  /// arranged in a tree.
  class HAPI_API HapticPrimitiveTree: public HAPIHapticShape {
  public:
    /// Constructor.
    HapticPrimitiveTree( 
              const vector< Collision::GeometryPrimitive * > &_primitives,
              HAPISurfaceObject *_surface, 
              Collision::FaceType _touchable_face = 
              Collision::FRONT_AND_BACK,
              void *_userdata = NULL,
              int _shape_id = -1, 
              void (*_clean_up_func)( void * ) = 0
             ):
      HAPIHapticShape( _surface, _touchable_face, _userdata,
                       _shape_id, _clean_up_func ) {
      tree = new Collision::AABBPrimitiveTree( _primitives  );
    }

    HapticPrimitiveTree(
              const Matrix4 &_transform,
              const vector< Collision::GeometryPrimitive * > &_primitives,
              HAPISurfaceObject *_surface, 
              Collision::FaceType _touchable_face = 
              Collision::FRONT_AND_BACK,
              void *_userdata = NULL,
              int _shape_id = -1, 
              void (*_clean_up_func)( void * ) = 0
             ):
      HAPIHapticShape( _transform, _surface, _touchable_face, _userdata,
                       _shape_id, _clean_up_func ) {
      tree = new Collision::AABBPrimitiveTree( _primitives  );
    }

    template< class Iterator >
    HapticPrimitiveTree( Iterator begin,
                               Iterator end,
              HAPISurfaceObject *_surface, 
              Collision::FaceType _touchable_face = 
              Collision::FRONT_AND_BACK,
              void *_userdata = NULL,
              int _shape_id = -1, 
              void (*_clean_up_func)( void * ) = 0 ):
      HAPIHapticShape( _surface, _touchable_face, _userdata,
                       _shape_id, _clean_up_func ) {
        tree = new Collision::AABBPrimitiveTree( begin, end );
      }

    ~HapticPrimitiveTree() {
      delete tree;
    }

#ifdef HAVE_OPENGL
    /// An upper bound on how many triangles are renderered.
    virtual int nrTriangles() {
      return countPrimitives< Collision::Triangle >( tree );
    }

    /// An upper bound on how many points are renderered.
    virtual int nrPoints() {
      return countPrimitives< Collision::Point >( tree );
    }

    /// An upper bound on how many lines are renderered.
    virtual int nrLines() {
      return countPrimitives< Collision::LineSegment >( tree );
    }
#endif

  protected:

    typedef H3DUtil::AutoRefVector< Collision::GeometryPrimitive >
      PrimitiveVector;

#ifdef HAVE_OPENGL
    template< class ClassToCount >
    inline int countPrimitives( Collision::BBPrimitiveTree *_tree ) {
      if( _tree == 0 )
        return 0;
      
      if( _tree->isLeaf() ) {
        int counter = 0;
        for( PrimitiveVector::const_iterator i = _tree->primitives.begin();
             i != _tree->primitives.end(); i++ ) {
          if( dynamic_cast< ClassToCount * >( *i ) )
            counter++;
          else if( dynamic_cast< Collision::Plane * >( *i ) ||
            dynamic_cast< Collision::Sphere * >( *i ) ) {
            counter = -1;
            break;
          }
        }
        return counter;
      }

      int left_counter = countPrimitives< ClassToCount >( _tree->left.get() );
      if( left_counter == -1 )
        return -1;
      
      int right_counter = countPrimitives< ClassToCount >( _tree->right.get());
      if( right_counter == -1 )
        return -1;
      
      return left_counter + right_counter;
    }
#endif

    /// Detect collision between a line segment and the object.
    /// \param from The start of the line segment(in local coords).
    /// \param to The end of the line segment(in local coords).
    /// \param result Contains info about closest intersection, if 
    /// line intersects object(in local coords).
    /// \param face The sides of the object that can be intersected. E.g.
    /// if FRONT, intersections will be reported only if they occur from
    /// the front side, i.e. the side in which the normal points. 
    /// \returns true if intersected, false otherwise.
    virtual bool lineIntersectShape( const Vec3 &from, 
                                     const Vec3 &to,
                                     Collision::IntersectionInfo &result,
                                     Collision::FaceType face = 
                                     Collision::FRONT_AND_BACK); 

    /// Get the closest point and normal on the object to the given point p.
    /// \param p The point to find the closest point to (in local coords).
    /// \param cp Return parameter for closest point (in local coords).
    /// \param n Return parameter for normal at closest point
    /// (in local coords).
    /// \param tc Return paramater for texture coordinate at closest point.
    virtual void closestPointOnShape( const Vec3 &p, Vec3 &cp, 
                                      Vec3 &n, Vec3 &tc );

    /// Detect collision between a moving sphere and the object.
    /// \param radius The radius of the sphere
    /// \param from The start position of the sphere
    /// \param to The end position of the sphere.
    /// \returns true if intersected, false otherwise.
    virtual bool movingSphereIntersectShape( HAPIFloat radius,
                                             const Vec3 &from, 
                                             const Vec3 &to );

    /// Get constraint planes of the shape. A proxy of a haptics renderer
    /// will always stay above any constraints added.
    ///
    /// \param point Point to constrain(in local coords).
    /// \param constraints Where to add the constraints.
    /// \param face Determines which faces of the shape will be seen as
    /// constraining.
    /// \param radius Only add constraints within this radius. If set to -1
    /// all constraints will be added.
    virtual void getConstraintsOfShape( const Vec3 &point,
                                        Constraints &constraints,
                                        Collision::FaceType face = 
                                        Collision::FRONT_AND_BACK,
                                        HAPIFloat radius = -1 );

    /// Calculates a matrix transforming a vector from local space of the shape
    /// to texture space of the shape.
    /// \param point The point at which to find the tangent vectors
    /// in local coordinates
    /// \param result_mtx Stores the calculated matrix
    virtual void getTangentSpaceMatrixShape( const Vec3 &point,
                                             Matrix4 &result_mtx );

#ifdef HAVE_OPENGL
    /// Render a graphical representation of the shape using OpenGL. This
    /// is used by the OpenHapticsRenderer when using feedback or depth
    /// buffer shapes. The rendering should be done in local space of the 
    /// shape, i.e. ignoring the transform matrix in the shape.
    virtual void glRenderShape();
#endif

    Collision::BBPrimitiveTree *tree;      
  };
}

#endif
