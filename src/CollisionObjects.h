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
#ifndef __COLLISIONSTRUCTURES_H__
#define __COLLISIONSTRUCTURES_H__

#include <HAPITypes.h>
#include <GL/glew.h>
#include <vector>
#include <assert.h>
#include <RefCountedClass.h>
#include <AutoRef.h>
#include <AutoRefVector.h>

namespace HAPI {

  class HAPIHapticShape;   

  namespace Bounds {
    /// Intersect segment S(t)=sa+t(sb-sa), 0<=t<=1 against cylinder 
    /// specified by p, q and r. The segment finds the first along the segment.
    bool intersectSegmentCylinder( Vec3 sa, Vec3 sb, 
                                   Vec3 p, Vec3 q, 
                                   HAPIFloat r, 
                                   HAPIFloat & t );

    class GeometryPrimitive;
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
      HAPIHapticShape * haptic_shape;
      GeometryPrimitive * primitive;
    };
    
    /// \brief The CollisionObject class is the base class for objects that 
    /// can be used  in collision detection.
    class HAPI_API CollisionObject : public H3DUtil::RefCountedClass {
    public:

      CollisionObject(){};

      CollisionObject( bool _use_lock ) :
        H3DUtil::RefCountedClass( _use_lock ){};
         
      virtual void getConstraints( const Vec3 &point,
                                   std::vector< PlaneConstraint > &constraints,
                                   FaceType face = Bounds::FRONT_AND_BACK ) {}
      
      virtual void getConstraints( const Vec3 &point,
                                   const Matrix4 &matrix,
                                   std::vector< PlaneConstraint > &constraints,
                                   FaceType face = Bounds::FRONT_AND_BACK ) {}
      
      /// Get the closest point and normal on the object to the given point p.
      /// \param p The point to find the closest point to.
      /// \param closest_point Return parameter for closest point
      /// \param normal Return parameter for normal at closest point.
      /// \param tex_coord Return paramater for texture coordinate at closest 
      /// point
      virtual void closestPoint( const Vec3 &p,
                                 Vec3 &closest_point,
                                 Vec3 &normal,
                                 Vec3 &tex_coord ) {}// = 0;

      /// Detect collision between a line segment and the object.
      /// \param from The start of the line segment.
      /// \param to The end of the line segment.
      /// \param result Contains info about closest intersection, if 
      /// line intersects object
      /// \param face The sides of the object that can be intersected. E.g.
      /// if FRONT, intersections will be reported only if they occur from
      /// the front side, i.e. the side in which the normal points. 
      /// \returns true if intersected, false otherwise.
      virtual bool lineIntersect( const Vec3 &from, 
                                  const Vec3 &to,
                                  IntersectionInfo &result,
                                  FaceType face = Bounds::FRONT_AND_BACK ){ return false; }// = 0;

      /// Detect collision between a moving sphere and the object.
      /// \param radius The radius of the sphere
      /// \param from The start position of the sphere
      /// \param to The end position of the sphere.
      /// \returns true if intersected, false otherwise.
      virtual bool movingSphereIntersect( HAPIFloat radius,
                                          const Vec3 &from, 
                                          const Vec3 &to ){ return false; }// = 0;
    
    /// Render the outlines of the object for debugging purposes.
    virtual void render() {}
  };

    /// \brief The BoundObject is the base class for bounding objects.
    class HAPI_API BoundObject: public CollisionObject {
    public:
      /// Returns true if the given point is inside the bound, and
      /// false otherwise.
      virtual bool insideBound( const Vec3 &p ) = 0;

      /// The boundIntersect returns true if the line segment intersects the
      /// bound or if the line segment is totally inside the bound.
      virtual bool boundIntersect( const Vec3 &from, 
                                   const Vec3 &to ) {
        IntersectionInfo info;
        return( insideBound( from ) || 
                insideBound( to ) || 
                lineIntersect( from, to, info ) );
                
      }

      /// Detect collision between a moving sphere and the object.
      /// \param radius The radius of the sphere
      /// \param from The start position of the sphere
      /// \param to The end position of the sphere.
      /// \returns true if intersected, false otherwise.
      virtual bool boundMovingSphereIntersect( HAPIFloat radius,
                                               const Vec3 &from, 
                                               const Vec3 &to ) {
        return( insideBound( from ) || 
                insideBound( to ) || 
                movingSphereIntersect( radius, from, to ) );
      }

      /// The closest point on the bound to the point. To know the closest
      /// point to the primitives in the bound, use closestPoint
      virtual Vec3 boundClosestPoint( const Vec3 &p ) = 0;

    };

    /// \brief The BoundPrimitive class is the base class for simple bound 
    /// primitives such as bounding spheres and bounding boxes.
    class HAPI_API BoundPrimitive: public BoundObject {
    public:
      /// Constructor.
      BoundPrimitive() : collided( false ) {}

      /// Update the bound primitive to contain all the given points.
      virtual void fitAroundPoints( const vector< Vec3 >&points ) = 0;

      /// Returns a Vec3 specifying the longest axis of the bound primitive.
      /// 1 0 0 means the x-axis, 0 1 0 the y-axis and 0 0 1 the z-axis.
      virtual Vec3 longestAxis() const = 0;
      
      bool collided;
    };
    
    /// \brief The GeometryPrimitive is the base class for all geometry 
    /// primitives used in collision detection.
    class HAPI_API GeometryPrimitive: public CollisionObject {
    public:
      virtual void getConstraints( const Vec3 &point,
                                   std::vector< PlaneConstraint > &constraints,
                                   FaceType face = Bounds::FRONT_AND_BACK );

      /// Returns a point representing the primitive. Is used for example when
      /// building BinaryTreeBound.
      virtual Vec3 pointRepresentation() const { return Vec3(); }
      bool collided;
    };
    
    
    class HAPI_API Plane: public GeometryPrimitive {
    public:
      /// Constructor.
      Plane( const Vec3 &p, const Vec3 &n ):
        point( p ), normal( n ) {}
      
      /// Returns a point representing the primitive. In this case it is the 
      /// center of the triangle.
      inline virtual Vec3 pointRepresentation() const {
        return point;
      }
      
      /// Get the closest point and normal on the object to the given point p.
      /// \param p The point to find the closest point to.
      /// \param closest_point Return parameter for closest point
      /// \param normal Return parameter for normal at closest point.
      /// \param tex_coord Return paramater for texture coordinate at closest 
      /// point
      virtual void closestPoint( const Vec3 &p,
                                 Vec3 &closest_point,
                                 Vec3 &n,
                                 Vec3 &tex_coord  ) {
        HAPIFloat t = normal * ( p - point );
        closest_point = p - t * normal;
        n = normal;
        tex_coord = Vec3();
      }

      /// Detect collision between a line segment and the object.
      /// \param from The start of the line segment.
      /// \param to The end of the line segment.
      /// \param result Contains info about the closest intersection, if 
      /// line intersects object
      /// \param face The sides of the object that can be intersected. E.g.
      /// if FRONT, intersections will be reported only if they occur from
      /// the front side, i.e. the side in which the normal points. 
      /// \returns true if intersected, false otherwise.
      virtual bool lineIntersect( const Vec3 &from, 
                                  const Vec3 &to,
                                  IntersectionInfo &result,
                                  FaceType face = Bounds::FRONT_AND_BACK );

      /// Detect collision between a moving sphere and the object.
      /// \param radius The radius of the sphere
      /// \param from The start position of the sphere
      /// \param to The end position of the sphere.
      /// \returns true if intersected, false otherwise.
      virtual bool movingSphereIntersect( HAPIFloat radius,
                                          const Vec3 &from, 
                                          const Vec3 &to );


      Vec3 point, normal;
    };


    class HAPI_API Sphere: public GeometryPrimitive {
    public:
      /// Constructor.
      Sphere( const Vec3 &_center, HAPIFloat _radius ):
        center( _center ), radius( _radius ) {}
      
      /// Returns a point representing the primitive. In this case it is the 
      /// center of the triangle.
      inline virtual Vec3 pointRepresentation() const {
        return center;
      }
      
      /// Get the closest point and normal on the object to the given point p.
      /// \param p The point to find the closest point to.
      /// \param closest_point Return parameter for closest point
      /// \param normal Return parameter for normal at closest point.
      /// \param tex_coord Return paramater for texture coordinate at closest 
      /// point
      virtual void closestPoint( const Vec3 &p,
                                 Vec3 &closest_point,
                                 Vec3 &normal,
                                 Vec3 &tex_coord ) {
        Vec3 dir = p - center;
        if( dir * dir < Constants::epsilon ) {
          closest_point = center + Vec3( radius, 0, 0 ); 
          normal = Vec3( 1, 0, 0 );
        }
        else {
          dir.normalize();
          closest_point = center + dir * radius;
          normal = dir;
        }
        // todo: defualt tex coord
        tex_coord = Vec3();
      }

      /// Detect collision between a line segment and the object.
      /// \param from The start of the line segment.
      /// \param to The end of the line segment.
      /// \param result Contains info about the closest intersection, if 
      /// line intersects object
      /// \param face The sides of the object that can be intersected. E.g.
      /// if FRONT, intersections will be reported only if they occur from
      /// the front side, i.e. the side in which the normal points. 
      /// \returns true if intersected, false otherwise.
      virtual bool lineIntersect( const Vec3 &from, 
                                  const Vec3 &to,
                                  IntersectionInfo &result,
                                  FaceType face = Bounds::FRONT_AND_BACK );

      /// Detect collision between a moving sphere and the object.
      /// \param radius The radius of the sphere
      /// \param from The start position of the sphere
      /// \param to The end position of the sphere.
      /// \returns true if intersected, false otherwise.
      virtual bool movingSphereIntersect( HAPIFloat radius,
                                          const Vec3 &from, 
                                          const Vec3 &to );


      Vec3 center;
      HAPIFloat radius;
    };


    /// \brief The Triangle class represents a triangle primitive.
    class HAPI_API Triangle: public GeometryPrimitive  {
    public:
      /// Default constructor.
      Triangle() {}

      /// Constructor.
      /// a, b and are the coordinates for each vertex
      /// ta, tb, and tc are the texture coordinates for each vertex.
      Triangle( const Vec3& a_, const Vec3& b_, const Vec3& c_,
                const Vec3& ta_ = Vec3(), 
                const Vec3& tb_ = Vec3(), 
                const Vec3& tc_ = Vec3()) : 
        a(a_),b(b_),c(c_),
        ta( ta_ ), tb( tb_ ), tc( tc_ ) {}

      /// Get the closest point and normal on the object to the given point p.
      /// \param p The point to find the closest point to.
      /// \param closest_point Return parameter for closest point
      /// \param normal Return parameter for normal at closest point.
      /// \param tex_coord Return paramater for texture coordinate at closest 
      /// point
      virtual void closestPoint( const Vec3 &p,
                                 Vec3 &closest_point,
                                 Vec3 &normal,
                                 Vec3 &tex_coord );

      /// Detect collision between a line segment and the object.
      /// \param from The start of the line segment.
      /// \param to The end of the line segment.
      /// \param result Contains info about the closest intersection, if 
      /// line intersects object
      /// \param face The sides of the object that can be intersected. E.g.
      /// if FRONT, intersections will be reported only if they occur from
      /// the front side, i.e. the side in which the normal points. 
      /// \returns true if intersected, false otherwise.
      virtual bool lineIntersect( const Vec3 &from, 
                                  const Vec3 &to,
                                  IntersectionInfo &result,
                                  FaceType face = Bounds::FRONT_AND_BACK );

      /// Detect collision between a moving sphere and the object.
      /// \param radius The radius of the sphere
      /// \param from The start position of the sphere
      /// \param to The end position of the sphere.
      /// \returns true if intersected, false otherwise.
      virtual bool movingSphereIntersect( HAPIFloat radius,
                                          const Vec3 &from, 
                                          const Vec3 &to );

      /// Detect collision between a moving sphere and the object.
      /// \param radius The radius of the sphere
      /// \param from The start position of the sphere
      /// \param to The end position of the sphere.
      /// object
      /// \returns true if intersected, false otherwise.
      bool movingSphereIntersectRobust( HAPIFloat radius,
                                          const Vec3 &from, 
                                          const Vec3 &to );


      /// Returns a point representing the primitive. In this case it is the 
      /// center of the triangle.
      inline virtual Vec3 pointRepresentation() const {
        return ( a + b + c ) / 3;
      }

      /// Render the outlines of the object for debugging purposes.
      virtual void render();
      
      
    public:
      /// WTF?? Why is this needed. VC complains if not present
      virtual void getConstraints( const Vec3 &point,
                                   std::vector< PlaneConstraint > &constraints,
                                   FaceType face = Bounds::FRONT_AND_BACK ) {
        GeometryPrimitive::getConstraints( point, constraints, face );
      }

      virtual void getConstraints( const Vec3 &point,
                                   const Matrix4 &matrix,
                                   std::vector< PlaneConstraint > &constraints,
                                   FaceType face = Bounds::FRONT_AND_BACK );

      /// The corners of the triangle.
      Vec3 a,b,c;

      /// The texture coordinates at the corners of the triangle.
      Vec3 ta, tb, tc;
    };

    /// \brief The LineSegment class represents a line segment primitive.
    class HAPI_API LineSegment: public GeometryPrimitive  {
    public:
      /// Default constructor.
      LineSegment() {}

      /// Constructor.
      LineSegment( const Vec3& _start, const Vec3& _end) : 
        start( _start ), end( _end ) {}

      /// WTF?? Why is this needed. VC complains if not present
      virtual void getConstraints( const Vec3 &point,
                                   std::vector< PlaneConstraint > &constraints,
                                   FaceType face = Bounds::FRONT_AND_BACK ) {
        GeometryPrimitive::getConstraints( point, constraints, face );
      }

      virtual void getConstraints( const Vec3 &point,
                                   const Matrix4 &matrix,
                                   std::vector< PlaneConstraint > &constraints,
                                   FaceType face = Bounds::FRONT_AND_BACK );

      /// Get the closest point and normal on the object to the given point p.
      /// \param p The point to find the closest point to.
      /// \param closest_point Return parameter for closest point
      /// \param normal Return parameter for normal at closest point.
      /// \param tex_coord Return paramater for texture coordinate at closest 
      /// point
      virtual void closestPoint( const Vec3 &p,
                                 Vec3 &closest_point,
                                 Vec3 &normal,
                                 Vec3 &tex_coord );

      /// Returns the closest point on the object to the line segment
      /// from -> to
      HAPIFloat closestPointOnLine( const Vec3 &from, const Vec3 &to,
                                    HAPIFloat &s, HAPIFloat &t,
                                    Vec3 &c0, Vec3 &c1 );

      /// Detect collision between a line segment and the object.
      /// \param from The start of the line segment.
      /// \param to The end of the line segment.
      /// \param result Contains info about the closest intersection, if 
      /// line intersects object
      /// \param face The sides of the object that can be intersected. E.g.
      /// if FRONT, intersections will be reported only if they occur from
      /// the front side, i.e. the side in which the normal points. 
      /// \returns true if intersected, false otherwise.
      virtual bool lineIntersect( const Vec3 &from, 
                                  const Vec3 &to,
                                  IntersectionInfo &result,
                                  FaceType face = Bounds::FRONT_AND_BACK );

      /// Detect collision between a moving sphere and the object.
      /// \param radius The radius of the sphere
      /// \param from The start position of the sphere
      /// \param to The end position of the sphere.
      /// \returns true if intersected, false otherwise.
      virtual bool movingSphereIntersect( HAPIFloat radius,
                                          const Vec3 &from, 
                                          const Vec3 &to );

      /// Returns a point representing the primitive. In this case it is the 
      /// center of the line segment.
      inline virtual Vec3 pointRepresentation() const {
        return ( start + end ) / 2;
      }

      /// Render the outlines of the object for debugging purposes.
      virtual void render();
      
      /// The start of the line segment.
      Vec3 start;
      /// The end of the line segment.
      Vec3 end;

    protected:
      inline HAPIFloat clamp( HAPIFloat n, HAPIFloat min, HAPIFloat max) {
        if (n < min) return min;
        if (n > max) return max;
        return n;
      }
    };

    /// \brief The Point class represents a point primitive.
    class HAPI_API Point: public GeometryPrimitive  {
    public:
      /// Default constructor.
      Point() {}

      /// Constructor.
      Point( const Vec3& _p ) : 
        position( _p ) {}

      /// WTF?? Why is this needed. VC complains if not present
      virtual void getConstraints( const Vec3 &point,
                                   std::vector< PlaneConstraint > &constraints,
                                   FaceType face = Bounds::FRONT_AND_BACK ) {
        GeometryPrimitive::getConstraints( point, constraints, face );
      }

      virtual void getConstraints( const Vec3 &point,
                                   const Matrix4 &matrix,
                                   std::vector< PlaneConstraint > &constraints,
                                   FaceType face = Bounds::FRONT_AND_BACK );

      /// Get the closest point and normal on the object to the given point p.
      /// \param p The point to find the closest point to.
      /// \param closest_point Return parameter for closest point
      /// \param normal Return parameter for normal at closest point.
      /// \param tex_coord Return paramater for texture coordinate at closest 
      /// point
      virtual void closestPoint( const Vec3 &p,
                                 Vec3 &closest_point,
                                 Vec3 &normal,
                                 Vec3 &tex_coord ) {
        closest_point = position;
        normal = p - closest_point;
        normal.normalizeSafe();
        tex_coord = Vec3(); 
      }

      /// Detect collision between a line segment and the object.
      /// \param from The start of the line segment.
      /// \param to The end of the line segment.
      /// \param result Contains info about the closest intersection, if 
      /// line intersects object
      /// \param face The sides of the object that can be intersected. E.g.
      /// if FRONT, intersections will be reported only if they occur from
      /// the front side, i.e. the side in which the normal points. 
      /// \returns true if intersected, false otherwise.
      virtual bool lineIntersect( const Vec3 &from, 
                                  const Vec3 &to,
                                  IntersectionInfo &result,
                                  FaceType face = Bounds::FRONT_AND_BACK );

      /// Detect collision between a moving sphere and the object.
      /// \param radius The radius of the sphere
      /// \param from The start position of the sphere
      /// \param to The end position of the sphere.
      /// \returns true if intersected, false otherwise.
      virtual bool movingSphereIntersect( HAPIFloat radius,
                                          const Vec3 &from, 
                                          const Vec3 &to );

      /// Returns a point representing the primitive. In this case it is the 
      /// center of the line segment.
      inline virtual Vec3 pointRepresentation() const {
        return position;
      }

      /// Render the outlines of the object for debugging purposes.
      virtual void render();
      
      /// The position of the point
      Vec3 position;
    };
    
    /// \brief The AABoxBound class represents an axis-aligned bounding box.
    class HAPI_API AABoxBound: public BoundPrimitive {
    public:
      /// Render the outlines of the object for debugging purposes.
      virtual void render();

      /// Returns true if the given point is inside the bound, and
      /// false otherwise.
      virtual bool insideBound( const Vec3 &p );

      /// Returns a Vec3 specifying the longest axis of the bound primitive.
      /// 1 0 0 means the x-axis, 0 1 0 the y-axis and 0 0 1 the z-axis.
      inline virtual Vec3 longestAxis() const {
        Vec3 dims = max - min;
        
        if (dims.x >= dims.y && dims.x >= dims.z) return Vec3(1,0,0);
        if (dims.y >= dims.x && dims.y >= dims.z) return Vec3(0,1,0);
        else													return Vec3(0,0,1);
      }

      /// Update the bound primitive to contain all the given points.
      virtual void fitAroundPoints( const vector< Vec3 > &points );

      /// Detect collision between a line segment and the object.
      /// \param from The start of the line segment.
      /// \param to The end of the line segment.
      /// \param result Contains info about the closest intersection, if 
      /// line intersects object
      /// \param face The sides of the object that can be intersected. E.g.
      /// if FRONT, intersections will be reported only if they occur from
      /// the front side, i.e. the side in which the normal points. 
      /// \returns true if intersected, false otherwise.
      virtual bool lineIntersect( const Vec3 &from, 
                                  const Vec3 &to,
                                  IntersectionInfo &result,
                                  FaceType face = Bounds::FRONT_AND_BACK ) {
        return boundIntersect( from, to );
      }

      /// Detect collision between a moving sphere and the object.
      /// \param radius The radius of the sphere
      /// \param from The start position of the sphere
      /// \param to The end position of the sphere.
      /// \returns true if intersected, false otherwise.
      virtual bool movingSphereIntersect( HAPIFloat radius,
                                          const Vec3 &from, 
                                          const Vec3 &to ) {
        Vec3 old_min = min;
        Vec3 old_max = max;

        Vec3 r = Vec3( radius, radius, radius ); 
        min -= r;
        max += r;

        bool res = boundIntersect( from, to );
        
        min = old_min;
        max = old_max;
        return res;
      }

      /// The boundIntersect returns true if the line segment intersects the
      /// bound or if the line segment is totally inside the bound.
      virtual bool boundIntersect( const Vec3 &from, 
                                   const Vec3 &to );
      
      /// Get the closest point and normal on the object to the given point p.
      /// \param p The point to find the closest point to.
      /// \param closest_point Return parameter for closest point
      /// \param normal Return parameter for normal at closest point.
      /// \param tex_coord Return paramater for texture coordinate at closest 
      /// point
      virtual void closestPoint( const Vec3 &p,
                                 Vec3 &closest_point,
                                 Vec3 &normal,
                                 Vec3 &tex_coord );

      /// The closest point on the bound to the point. 
      virtual Vec3 boundClosestPoint( const Vec3 &p ) {
        Vec3 result;
        // for each coordinate axis, if the point coordinate value
        // is outside box, clamp it to the box, e;se keep it as it is
        for( int i = 0; i < 3; i++ ) {
          HAPIFloat v = p[i];
          if( v < min[i] ) v = min[i];
          if( v > max[i] ) v = max[i];
          result[i] = v;
        }
        return result;
      }

      /// The max and min corners of the bounding box,
      Vec3 min;
      Vec3 max;
     
    };

    /// \brief The OrientedBoxBound class represents an oriented bounding box.
    class HAPI_API OrientedBoxBound: public AABoxBound {
    public:
      /// Render the outlines of the object for debugging purposes.
      virtual void render();

      /// Returns true if the given point is inside the bound, and
      /// false otherwise.
      virtual bool insideBound( const Vec3 &p );

      /// Returns a Vec3 specifying the longest axis of the bound primitive.
      /// 1 0 0 means the x-axis, 0 1 0 the y-axis and 0 0 1 the z-axis.
      inline virtual Vec3 longestAxis() const;

      /// Update the bound primitive to contain all the given points.
      virtual void fitAroundPoints( const vector< Vec3 > &points );

      /// Detect collision between a line segment and the object.
      /// \param from The start of the line segment.
      /// \param to The end of the line segment.
      /// \param result Contains info about the closest intersection, if 
      /// line intersects object
      /// \param face The sides of the object that can be intersected. E.g.
      /// if FRONT, intersections will be reported only if they occur from
      /// the front side, i.e. the side in which the normal points. 
      /// \returns true if intersected, false otherwise.
      virtual bool lineIntersect( const Vec3 &from, 
                                  const Vec3 &to,
                                  IntersectionInfo &result,
                                  FaceType face = Bounds::FRONT_AND_BACK ) {
        return boundIntersect( from, to );
      }

      /// The boundIntersect returns true if the line segment intersects the
      /// bound or if the line segment is totally inside the bound.
      virtual bool boundIntersect( const Vec3 &from, 
                                   const Vec3 &to );
      
      /// Get the closest point and normal on the object to the given point p.
      /// \param p The point to find the closest point to.
      /// \param closest_point Return parameter for closest point
      /// \param normal Return parameter for normal at closest point.
      /// \param tex_coord Return paramater for texture coordinate at closest 
      /// point
      virtual void closestPoint( const Vec3 &p,
                                 Vec3 &closest_point,
                                 Vec3 &normal,
                                 Vec3 &tex_coord  );

      Vec3 center;
      Rotation orientation;
      Vec3 halfsize;
      
    };


    /// \brief The SphereBound class represents a bounding sphere.
    class HAPI_API SphereBound: public BoundPrimitive {
    public:
      /// Default constructor.
      SphereBound(): gl_quadric( NULL ) {}

      /// Constructor.
      SphereBound(const Vec3& c, HAPIFloat r): 
        center (c), radius(r), 
        gl_quadric( NULL ) {}
      
      /// Render the outlines of the object for debugging purposes.
      virtual void render();

      /// Returns true if the given point is inside the bound, and
      /// false otherwise.
      virtual bool insideBound( const Vec3 &p );

      /// Get the closest point and normal on the object to the given point p.
      /// \param p The point to find the closest point to.
      /// \param closest_point Return parameter for closest point
      /// \param normal Return parameter for normal at closest point.
      /// \param tex_coord Return paramater for texture coordinate at closest 
      /// point
      virtual void closestPoint( const Vec3 &p,
                                 Vec3 &closest_point,
                                 Vec3 &normal,
                                 Vec3 &tex_coord );

      /// The closest point on the bound to the point. 
      virtual Vec3 boundClosestPoint( const Vec3 &p ) {
        Vec3 cp, tmp;
        closestPoint( p, cp, tmp, tmp );
        return cp;
      }

      /// Detect collision between a line segment and the object.
      /// \param from The start of the line segment.
      /// \param to The end of the line segment.
      /// \param result Contains info about the closest intersection, if
      /// line intersects object
      /// \param face The sides of the object that can be intersected. E.g.
      /// if FRONT, intersections will be reported only if they occur from
      /// the front side, i.e. the side in which the normal points. 
      /// \returns true if intersected, false otherwise.
      virtual bool lineIntersect( const Vec3 &from, 
                                  const Vec3 &to,
                                  IntersectionInfo &result,
                                  FaceType face = Bounds::FRONT_AND_BACK );

      /// The boundIntersect returns true if the line segment intersects the
      /// bound or if the line segment is totally inside the bound.
      virtual bool boundIntersect( const Vec3 &from, const Vec3 &to );

      /// Detect collision between a moving sphere and the object.
      /// \param radius The radius of the sphere
      /// \param from The start position of the sphere
      /// \param to The end position of the sphere.
      /// \returns true if intersected, false otherwise.
      virtual bool movingSphereIntersect( HAPIFloat radius,
                                          const Vec3 &from, 
                                          const Vec3 &to );

      /// Update the bound primitive to contain all the given points.
      virtual void fitAroundPoints( const vector< Vec3 > &points );

      /// Returns a Vec3 specifying the longest axis of the bound primitive.
      /// 1 0 0 means the x-axis, 0 1 0 the y-axis and 0 0 1 the z-axis.
      virtual Vec3 longestAxis() const { return Vec3( 1, 0, 0 ); }

      /// The center of the sphere
      Vec3 center;

      /// The radius of the sphere.
      HAPIFloat radius;

      GLUquadricObj* gl_quadric;
    };

    /// \brief The BinaryBoundTree class is the base class for bound objects 
    /// structured as a binary tree. Each node in the tree has a BoundPrimitive
    /// specifying a bound for itself and each subtree has the same.
    class HAPI_API BinaryBoundTree: public BoundObject {
    public: 
      /// A function type for creating a bound primitive for a node in the tree.
      typedef BoundPrimitive *( *BoundNewFunc)(); 

      /// Convenience template for creating BoundNewFunc functions.
      template< class N >
      static BoundPrimitive *newInstance() { return new N; };

      /// Default constructor.
      BinaryBoundTree(  ): left( NULL ), right( NULL ), new_func( NULL ) {}

      /// Constructor.
      /// Builds a binary tree from a vector of triangles. The func argument specifies
      /// a function for creating  bound of the wanted type in each tree node.
      /// max_nr_triangles_in_leaf specifies the maximum number of triangles 
      /// that are allowed to be in a bound of a leaf in the tree. 
      BinaryBoundTree( BoundNewFunc func, 
                       const vector< Triangle > &triangles,
                       unsigned int max_nr_triangles_in_leaf = 1 );

      BinaryBoundTree( BoundNewFunc func, 
                       const vector< Triangle > &triangle_vector,
                       const vector< LineSegment > &linesegment_vector,
                       const vector< Point > &point_vector,
                       unsigned int max_nr_triangles_in_leaf = 1 );

      /// \brief Returns true if the tree is a leaf, i.e. has no sub-tress
      /// and hence just contains triangles. false otherwise.
      inline bool isLeaf() { 
        return left.get() == NULL && right.get() == NULL; 
      }

      virtual void getConstraints( const Vec3 &point,
                                   std::vector< PlaneConstraint > &constraints,
                                   FaceType face = Bounds::FRONT_AND_BACK );

      virtual void getConstraints( const Vec3 &point,
                                   const Matrix4 &matrix,
                                   std::vector< PlaneConstraint > &constraints,
                                   FaceType face = Bounds::FRONT_AND_BACK );

      virtual void getTrianglesWithinRadius( const Vec3 &p,
                                             HAPIFloat radius,
                                             vector< Triangle > &triangles);

      virtual void getPrimitivesWithinRadius( const Vec3 &p,
                                             HAPIFloat radius,
                                             vector< Triangle > &triangles,
                                             vector< LineSegment > &lines,
                                             vector< Point > &points );


      virtual void getTrianglesIntersectedByMovingSphere( HAPIFloat radius,
                                                          Vec3 from,
                                                          Vec3 to,
                                                          vector< Triangle > &triangles);

      virtual void getPrimitivesIntersectedByMovingSphere( HAPIFloat radius,
                                                          Vec3 from,
                                                          Vec3 to,
                                                          vector< Triangle > &triangles,
                                                          vector< LineSegment > &lines,
                                                          vector< Point > &points );

      /// Render the outlines of the object for debugging purposes.
      virtual void render();

      /// Render the outlines of the tree at a given tree depth for debugging 
      /// purposes.
      virtual void render( int depth );

      /// Returns true if the given point is inside the bound, and
      /// false otherwise.
      virtual bool insideBound( const Vec3 &p ) {
        if( bound.get() )
          return bound->insideBound( p );
        else
          return false;
      }

      /// The boundIntersect returns true if the line segment intersects the
      /// bound or if the line segment is totally inside the bound.
      virtual bool boundIntersect( const Vec3 &from, const Vec3 &to ) {
        if( bound.get() )
          return bound->boundIntersect( from, to );
        else
          return false;
      }

      /// Detect collision between a line segment and the object. Will check for 
      /// collision between the triangles contained in the leaves of the tree and
      /// the line segment.
      /// \param from The start of the line segment.
      /// \param to The end of the line segment.
      /// \param result Contains info about the closest intersection, if line
      /// intersects object
      /// \param face The sides of the object that can be intersected. E.g.
      /// if FRONT, intersections will be reported only if they occur from
      /// the front side, i.e. the side in which the normal points. 
      /// \returns true if intersected, false otherwise.
      virtual bool lineIntersect( const Vec3 &from, 
                                  const Vec3 &to,
                                  IntersectionInfo &result,
                                  FaceType face = Bounds::FRONT_AND_BACK );

      /// Detect collision between a moving sphere and the object.
      /// \param radius The radius of the sphere
      /// \param from The start position of the sphere
      /// \param to The end position of the sphere.
      /// \returns true if intersected, false otherwise.
      virtual bool movingSphereIntersect( HAPIFloat radius,
                                          const Vec3 &from, 
                                          const Vec3 &to );

      /// Detect collision between a moving sphere and the object.
      /// \param radius The radius of the sphere
      /// \param from The start position of the sphere
      /// \param to The end position of the sphere.
      /// \returns true if intersected, false otherwise.
      virtual bool boundMovingSphereIntersect( HAPIFloat radius,
                                               const Vec3 &from, 
                                               const Vec3 &to ) {
        if( bound.get() )
          return bound->boundMovingSphereIntersect( radius, from, to );
        else 
          return false;
      }

      /// Get the closest point and normal on the object to the given point p.
      /// \param p The point to find the closest point to.
      /// \param closest_point Return parameter for closest point
      /// \param normal Return parameter for normal at closest point.
      /// \param tex_coord Return paramater for texture coordinate at closest 
      /// point
      virtual void closestPoint( const Vec3 &p,
                                 Vec3 &closest_point,
                                 Vec3 &normal,
                                 Vec3 &tex_coord );

      /// The closest point on the bound to the point. If tree is a leaf,
      /// the closest point to the triangles in the leaf is returned.
      /// To know the closest point to the primitives in the bound, use closestPoint
      virtual Vec3 boundClosestPoint( const Vec3 &p ) {
        if( !isLeaf() && bound.get() )
          return bound->boundClosestPoint( p );
        else {
          Vec3 cp, tmp;
          closestPoint( p, cp, tmp, tmp );
          return cp;
        }
      }

      /// Add all triangles in the tree to the given vector.
      virtual void getAllTriangles( vector< Triangle > &triangles );

      /// Add all triangles, lines and points in the tree to the given vector.
      virtual void getAllPrimitives( vector< Triangle > &triangles,
                                    vector< LineSegment > &lines,
                                    vector< Point > &points );

      void clearCollidedFlag();

      /*union {
        struct { AutoRef< BoundPrimitive > bound; };
        struct { vector< Triangle > triangles; };
        };*/

      H3DUtil::AutoRef< BoundPrimitive > bound;
      vector< Triangle > triangles;
      vector< LineSegment > linesegments;
      vector< Point > points;

      /// The left subtree.
      H3DUtil::AutoRef< BinaryBoundTree > left;
      /// The right subtree.
      H3DUtil::AutoRef< BinaryBoundTree > right;
      /// The function to create new bounding primitives in for a tree node.
      BoundNewFunc new_func;
    };


    /// \brief The AABBTree is a BinaryBoundTree where the bounding primitive 
    /// for each node is a AABoxBound (axis-aligned bounding box).
    class HAPI_API AABBTree: public BinaryBoundTree {
    public:
      /// Default constructor.
      AABBTree(): BinaryBoundTree( ) {}

      /// Constructor.
      /// Builds a binary tree from a vector of triangles. 
      /// max_nr_triangles_in_leaf specifies the maximum number of triangles 
      /// that are allowed to be in a bound of a leaf in the tree. 
      AABBTree( const vector< Triangle > &triangles,
                unsigned int max_nr_triangles_in_leaf = 1 ):
        BinaryBoundTree( &(newInstance< AABoxBound >), triangles, max_nr_triangles_in_leaf ) {}

      AABBTree( const vector< Triangle > &triangles,
                const vector< LineSegment > &linesegment_vector,
                const vector< Point > &point_vector,
                unsigned int max_nr_triangles_in_leaf = 1 ):
        BinaryBoundTree( &(newInstance< AABoxBound >), triangles,
                         linesegment_vector, point_vector,
                         max_nr_triangles_in_leaf ) {}
    };


    /// \brief The OBBTree is a BinaryBoundTree where the bounding primitive 
    /// for each node is a OrientedBoxBound.

    class HAPI_API OBBTree: public BinaryBoundTree {
    public:
      /// Default constructor.
      OBBTree(): BinaryBoundTree( ) {}

      /// Constructor.
      /// Builds a binary tree from a vector of triangles. 
      /// max_nr_triangles_in_leaf specifies the maximum number of triangles 
      /// that are allowed to be in a bound of a leaf in the tree. 
      OBBTree( const vector< Triangle > &triangles,
               unsigned int max_nr_triangles_in_leaf = 1 ):
        BinaryBoundTree( &(newInstance< OrientedBoxBound >), triangles, 
                         max_nr_triangles_in_leaf ) {}
      
      OBBTree( const vector< Triangle > &triangles,
                const vector< LineSegment > &linesegment_vector,
                const vector< Point > &point_vector,
                unsigned int max_nr_triangles_in_leaf = 1 ):
        BinaryBoundTree( &(newInstance< OrientedBoxBound >), triangles,
                         linesegment_vector, point_vector,
                         max_nr_triangles_in_leaf ) {}
    };

    /// \brief The SphereBoundTree is a BinaryBoundTree where the bounding 
    /// primitive for each node is a SphereBound object.

    class HAPI_API SphereBoundTree: public BinaryBoundTree {
    public:
      /// Default constructor.
      SphereBoundTree(): BinaryBoundTree() {}

      /// Constructor.
      /// Builds a binary tree from a vector of triangles. 
      /// max_nr_triangles_in_leaf specifies the maximum number of triangles 
      /// that are allowed to be in a bound of a leaf in the tree. 
      SphereBoundTree( const vector< Triangle > &triangles,
                       unsigned int max_nr_triangles_in_leaf = 1): 
        BinaryBoundTree( &(newInstance< SphereBound > ), triangles, max_nr_triangles_in_leaf ) {}

      SphereBoundTree( const vector< Triangle > &triangles,
                const vector< LineSegment > &linesegment_vector,
                const vector< Point > &point_vector,
                unsigned int max_nr_triangles_in_leaf = 1 ):
        BinaryBoundTree( &(newInstance< SphereBound >), triangles,
                         linesegment_vector, point_vector,
                         max_nr_triangles_in_leaf ) {}
    };

    // Primitive special kind
    /// \brief The BBTreePrimitive class is the base class for bound objects 
    /// structured as a binary tree. Each node in the tree has a BoundPrimitive
    /// specifying a bound for itself and each subtree has the same.
    class HAPI_API BBTreePrimitive: public BoundObject {
    public: 
      /// A function type for creating a bound primitive for a node in the tree.
      typedef BoundPrimitive *( *BoundNewFunc)(); 

      /// Convenience template for creating BoundNewFunc functions.
      template< class N >
      static BoundPrimitive *newInstance() { return new N; };

      /// Default constructor.
      BBTreePrimitive(  ): left( NULL ), right( NULL ), new_func( NULL ) {}

      /// Constructor.
      /// Builds a binary tree from a vector of GeometryPrimitive pointers.
      /// The func argument specifies a function for creating  bound of the
      /// wanted type in each tree node. max_nr_primitives_in_leaf specifies
      /// the maximum number of primitives that are allowed to be in a bound
      /// of a leaf in the tree. 
      BBTreePrimitive( BoundNewFunc func, 
                       const vector< GeometryPrimitive * > &_primitives,
                       unsigned int max_nr_primitives_in_leaf = 1 );

      /// \brief Returns true if the tree is a leaf, i.e. has no sub-tress
      /// and hence just contains triangles. false otherwise.
      inline bool isLeaf() { 
        return left.get() == NULL && right.get() == NULL; 
      }

      virtual void getConstraints( const Vec3 &point,
                                   std::vector< PlaneConstraint > &constraints,
                                   FaceType face = Bounds::FRONT_AND_BACK );

      virtual void getConstraints( const Vec3 &point,
                                   const Matrix4 &matrix,
                                   std::vector< PlaneConstraint > &constraints,
                                   FaceType face = Bounds::FRONT_AND_BACK );

      virtual void getPrimitivesWithinRadius( const Vec3 &p,
                                             HAPIFloat radius,
                                             vector< GeometryPrimitive * > &primitives);

      virtual void getPrimitivesIntersectedByMovingSphere( HAPIFloat radius,
                                                          Vec3 from,
                                                          Vec3 to,
                                                          vector< GeometryPrimitive * > &primitives);

      /// Render the outlines of the object for debugging purposes.
      virtual void render();

      /// Render the outlines of the tree at a given tree depth for debugging 
      /// purposes.
      virtual void render( int depth );

      /// Returns true if the given point is inside the bound, and
      /// false otherwise.
      virtual bool insideBound( const Vec3 &p ) {
        if( bound.get() )
          return bound->insideBound( p );
        else
          return false;
      }

      /// The boundIntersect returns true if the line segment intersects the
      /// bound or if the line segment is totally inside the bound.
      virtual bool boundIntersect( const Vec3 &from, const Vec3 &to ) {
        if( bound.get() )
          return bound->boundIntersect( from, to );
        else
          return false;
      }

      /// Detect collision between a line segment and the object. Will check for 
      /// collision between the triangles contained in the leaves of the tree and
      /// the line segment.
      /// \param from The start of the line segment.
      /// \param to The end of the line segment.
      /// \param result Contains info about the closest intersection, if line
      /// intersects object
      /// \param face The sides of the object that can be intersected. E.g.
      /// if FRONT, intersections will be reported only if they occur from
      /// the front side, i.e. the side in which the normal points. 
      /// \returns true if intersected, false otherwise.
      virtual bool lineIntersect( const Vec3 &from, 
                                  const Vec3 &to,
                                  IntersectionInfo &result,
                                  FaceType face = Bounds::FRONT_AND_BACK );

      /// Detect collision between a moving sphere and the object.
      /// \param radius The radius of the sphere
      /// \param from The start position of the sphere
      /// \param to The end position of the sphere.
      /// \returns true if intersected, false otherwise.
      virtual bool movingSphereIntersect( HAPIFloat radius,
                                          const Vec3 &from, 
                                          const Vec3 &to );

      /// Detect collision between a moving sphere and the object.
      /// \param radius The radius of the sphere
      /// \param from The start position of the sphere
      /// \param to The end position of the sphere.
      /// \returns true if intersected, false otherwise.
      virtual bool boundMovingSphereIntersect( HAPIFloat radius,
                                               const Vec3 &from, 
                                               const Vec3 &to ) {
        if( bound.get() )
          return bound->boundMovingSphereIntersect( radius, from, to );
        else 
          return false;
      }

      /// Get the closest point and normal on the object to the given point p.
      /// \param p The point to find the closest point to.
      /// \param closest_point Return parameter for closest point
      /// \param normal Return parameter for normal at closest point.
      /// \param tex_coord Return paramater for texture coordinate at closest 
      /// point
      virtual void closestPoint( const Vec3 &p,
                                 Vec3 &closest_point,
                                 Vec3 &normal,
                                 Vec3 &tex_coord );

      /// The closest point on the bound to the point. If tree is a leaf,
      /// the closest point to the triangles in the leaf is returned.
      /// To know the closest point to the primitives in the bound, use closestPoint
      virtual Vec3 boundClosestPoint( const Vec3 &p ) {
        if( !isLeaf() && bound.get() )
          return bound->boundClosestPoint( p );
        else {
          Vec3 cp, tmp;
          closestPoint( p, cp, tmp, tmp );
          return cp;
        }
      }

      /// Add all triangles in the tree to the given vector.
      virtual void getAllPrimitives( vector< GeometryPrimitive * > &primitives );

      void clearCollidedFlag();

      H3DUtil::AutoRef< BoundPrimitive > bound;
      H3DUtil::AutoRefVector< GeometryPrimitive > primitives;

      /// The left subtree.
      H3DUtil::AutoRef< BBTreePrimitive > left;
      /// The right subtree.
      H3DUtil::AutoRef< BBTreePrimitive > right;
      /// The function to create new bounding primitives in for a tree node.
      BoundNewFunc new_func;
    };


    /// \brief The AABBTree is a BinaryBoundTree where the bounding primitive 
    /// for each node is a AABoxBound (axis-aligned bounding box).
    class HAPI_API AABBTreePrimitive: public BBTreePrimitive {
    public:
      /// Default constructor.
      AABBTreePrimitive(): BBTreePrimitive( ) {}

      /// Constructor.
      /// Builds a binary tree from a vector of primitives. 
      /// max_nr_primitives_in_leaf specifies the maximum number of primitives 
      /// that are allowed to be in a bound of a leaf in the tree. 
      AABBTreePrimitive( const vector< GeometryPrimitive * > &_primitives,
                unsigned int max_nr_primitives_in_leaf = 1 ):
        BBTreePrimitive( &(newInstance< AABoxBound >), _primitives, max_nr_primitives_in_leaf ) {}
    };


    /// \brief The OBBTree is a BinaryBoundTree where the bounding primitive 
    /// for each node is a OrientedBoxBound.

    class HAPI_API OBBTreePrimitive: public BBTreePrimitive {
    public:
      /// Default constructor.
      OBBTreePrimitive(): BBTreePrimitive( ) {}

      /// Constructor.
      /// Builds a binary tree from a vector of primitives. 
      /// max_nr_primitives_in_leaf specifies the maximum number of primitives 
      /// that are allowed to be in a bound of a leaf in the tree. 
      OBBTreePrimitive( const vector< GeometryPrimitive * > &_primitives,
               unsigned int max_nr_primitives_in_leaf = 1 ):
        BBTreePrimitive( &(newInstance< OrientedBoxBound >), _primitives, 
                         max_nr_primitives_in_leaf ) {}
    };

    /// \brief The SphereBoundTree is a BinaryBoundTree where the bounding 
    /// primitive for each node is a SphereBound object.

    class HAPI_API SBTreePrimitive: public BBTreePrimitive {
    public:
      /// Default constructor.
      SBTreePrimitive(): BBTreePrimitive() {}

      /// Constructor.
      /// Builds a binary tree from a vector of primitives. 
      /// max_nr_primitives_in_leaf specifies the maximum number of primitives 
      /// that are allowed to be in a bound of a leaf in the tree. 
      SBTreePrimitive( const vector< GeometryPrimitive * > &_primitives,
                       unsigned int max_nr_primitives_in_leaf = 1): 
        BBTreePrimitive( &(newInstance< SphereBound > ), _primitives, max_nr_primitives_in_leaf ) {}
    };

}
  using Bounds::PlaneConstraint;
}

#endif
