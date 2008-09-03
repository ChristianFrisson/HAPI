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
/// \file CollisionObjects.h
/// \brief Header file for CollisionObjects. The collision algorithm codes
/// are written with help of the book "Real-Time Collision Detection" by
/// Christer Ericson (Morgan Kaufmann, 2005). Home page of book
/// http://realtimecollisiondetection.net/
///
//
//////////////////////////////////////////////////////////////////////////////
#ifndef __COLLISIONSTRUCTURES_H__
#define __COLLISIONSTRUCTURES_H__

#include <HAPI/HAPITypes.h>
#include <HAPI/IntersectionInfo.h>

#include <vector>
#include <assert.h>
#include <H3DUtil/RefCountedClass.h>
#include <H3DUtil/AutoRef.h>
#include <H3DUtil/AutoRefVector.h>

namespace HAPI {
class Constraints;
  namespace Collision {
    /// Intersect segment S(t)=sa+t(sb-sa), 0<=t<=1 against cylinder 
    /// specified by p, q and r. The segment finds the first along the segment.
    /// If the segment intersects the cylinder the point of intersection can
    /// be calculated from t.
    bool HAPI_API intersectSegmentCylinder( Vec3 sa, Vec3 sb, 
                                            Vec3 p, Vec3 q, 
                                            HAPIFloat r, 
                                            HAPIFloat & t );

    class PlaneConstraint; 

    
    /// \ingroup AbstractClasses
    /// \class CollisionObject
    /// \brief The base class for objects that can be used in collision
    /// detection.
    class HAPI_API CollisionObject : public H3DUtil::RefCountedClass {
    public:
      /// Constructor.
      CollisionObject(){};
      
      CollisionObject( bool _use_lock ) :
        H3DUtil::RefCountedClass( _use_lock ){};
         
      typedef HAPI::Constraints Constraints;
      //typedef vector< Collision::PlaneConstraint > Constraints;

      /// Get constraint planes of the object. A proxy of a haptics renderer
      /// will always stay above any constraints added.
      /// \param point The point to constrain.
      /// \param constraints Where to add the constraints.
      /// \param face Determines which faces of the shape will be seen as
      /// constraining.
      /// \param radius Only add constraints within this radius. If set to -1
      /// all constraints will be added.
      virtual void getConstraints( const Vec3 &point,
                                   Constraints &constraints,
                                   FaceType face = Collision::FRONT_AND_BACK,
                                   HAPIFloat radius = -1 ) {}
      
      /// Get the closest point and normal on the object to the given point p.
      /// \param p The point to find the closest point to.
      /// \param closest_point Return parameter for closest point
      /// \param normal Return parameter for normal at closest point.
      /// \param tex_coord Return paramater for texture coordinate at closest 
      /// point
      virtual void closestPoint( const Vec3 &p,
                                 Vec3 &closest_point,
                                 Vec3 &normal,
                                 Vec3 &tex_coord ) = 0;

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
                               FaceType face = Collision::FRONT_AND_BACK ) = 0;

      /// Detect collision between a moving sphere and the object.
      /// \param radius The radius of the sphere
      /// \param from The start position of the sphere
      /// \param to The end position of the sphere.
      /// \returns true if intersected, false otherwise.
      virtual bool movingSphereIntersect( HAPIFloat radius,
                                          const Vec3 &from, 
                                          const Vec3 &to ){ return false; }

      /// Detect collision between a moving sphere and the object and returns
      /// information about the closest intersection. Is of course slower
      /// than the version of movingSphereIntersect which does not return
      /// information about the intersection.
      /// \param radius The radius of the sphere
      /// \param from The start position of the sphere
      /// \param to The end position of the sphere.
      /// \param result Contains information about the closest intersection
      /// if an intersection is detected.
      /// \returns true if intersected, false otherwise.
      virtual bool movingSphereIntersect( HAPIFloat radius,
                                          const Vec3 &from, 
                                          const Vec3 &to,
                                          IntersectionInfo &result){ return false; }
    
    /// Render the object. The caller of the function need to set up OpenGL
    /// state in case the rendering should be done differently
    /// (wireframe for example).
    virtual void render() {}
  };

    /// \ingroup AbstractClasses
    /// \class BoundObject
    /// \brief The base class for bounding objects.
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

    /// \ingroup AbstractClasses
    /// \class BoundPrimitive
    /// \brief The base class for simple bound primitives such as bounding
    /// spheres and bounding boxes.
    class HAPI_API BoundPrimitive: public BoundObject {
    public:
      /// Constructor.
      BoundPrimitive() {}

      /// Update the bound primitive to contain all the given points.
      virtual void fitAroundPoints( const vector< Vec3 >&points ) = 0;

      /// Returns a Vec3 specifying the longest axis of the bound primitive.
      /// 1 0 0 means the x-axis, 0 1 0 the y-axis and 0 0 1 the z-axis.
      virtual Vec3 longestAxis() const = 0;
    };
    
    /// \ingroup AbstractClasses
    /// \class GeometryPrimitive
    /// \brief The base class for all geometry primitives used in collision
    /// detection.
    class HAPI_API GeometryPrimitive: public CollisionObject {
    public:
      /// Get constraint planes of the shape. A proxy of a haptics renderer
      /// will always stay above any constraints added.
      /// \param point The point to constrain.
      /// \param constraints Where to add the constraints.
      /// \param face Determines which faces of the shape will be seen as
      /// constraining.
      /// \param radius Only add constraints within this radius. If set to -1
      /// all constraints will be added.
      virtual void getConstraints( const Vec3 &point,
                                   Constraints &constraints,
                                   FaceType face = Collision::FRONT_AND_BACK,
                                   HAPIFloat radius = -1 );

      /// Returns a point representing the primitive. Is used for example when
      /// building BinaryTreeBound.
      virtual Vec3 pointRepresentation() const { return Vec3(); }

      /// Calculates a matrix transforming from local space to texture space.
      /// \param point The point at which to find the tangent vectors
      /// \param result_mtx Where the result is stored.
      virtual void getTangentSpaceMatrix( const Vec3 &point,
                                          Matrix4 &result_mtx );

    };
    
    /// \ingroup CollisionStructures
    /// \class Plane
    /// \brief A collision primitive for a plane.
    ///
    /// The plane is defined by a point and a normal.
    class HAPI_API Plane: public GeometryPrimitive {
    public:
      /// Constructor.
      /// \param p A point in the plane.
      /// \param n The normal of the plane.
      Plane( const Vec3 &p, const Vec3 &n ):
        point( p ), normal( n ) {}
      
      /// Returns a point representing the primitive. In this case it is the 
      /// point defining the plane.
      inline virtual Vec3 pointRepresentation() const {
        return point;
      }
      
      /// Get the closest point and normal on the object to the given point p.
      /// \param p The point to find the closest point to.
      /// \param closest_point Return parameter for closest point
      /// \param n Return parameter for normal at closest point.
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
                                  FaceType face = Collision::FRONT_AND_BACK );

      /// Detect collision between a moving sphere and the object.
      /// \param radius The radius of the sphere
      /// \param from The start position of the sphere
      /// \param to The end position of the sphere.
      /// \returns true if intersected, false otherwise.
      virtual bool movingSphereIntersect( HAPIFloat radius,
                                          const Vec3 &from, 
                                          const Vec3 &to );

      /// Detect collision between a moving sphere and the object and returns
      /// information about the closest intersection. Is of course slower
      /// than the version of movingSphereIntersect which does not return
      /// information about the intersection.
      /// \param radius The radius of the sphere
      /// \param from The start position of the sphere
      /// \param to The end position of the sphere.
      /// \param result Contains information about the closest intersection
      /// if an intersection is detected.
      /// \returns true if intersected, false otherwise.
      virtual bool movingSphereIntersect( HAPIFloat radius,
                                          const Vec3 &from, 
                                          const Vec3 &to,
                                          IntersectionInfo &result);

      /// Render the object. The caller of the function need to set up OpenGL
      /// state in case the rendering should be done differently
      /// (wireframe for example).
      virtual void render();

      Vec3 point, normal;
    };


    /// \ingroup CollisionStructures
    /// \class Sphere
    /// \brief A collision primitive for a sphere.
    ///
    /// The Sphere is defined by a center and a radius.
    class HAPI_API Sphere: public GeometryPrimitive {
    public:
      /// Constructor.
      /// \param _center The center of the sphere.
      /// \param _radius The radius of the sphere.
      Sphere( const Vec3 &_center, HAPIFloat _radius );

      /// Destructor
      ~Sphere();
      
      /// Returns a point representing the primitive. In this case it is the 
      /// center of the sphere.
      inline virtual Vec3 pointRepresentation() const {
        return center;
      }

      /// Calculates a matrix transforming from local space to texture space.
      /// \param point The point at which to find the tangent vectors
      /// \param result_mtx Where the result is stored.
      virtual void getTangentSpaceMatrix( const Vec3 &point,
                                          Matrix4 &result_mtx );
      
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
                                  FaceType face = Collision::FRONT_AND_BACK );

      /// Detect collision between a moving sphere and the object.
      /// \param radius The radius of the sphere
      /// \param from The start position of the sphere
      /// \param to The end position of the sphere.
      /// \returns true if intersected, false otherwise.
      virtual bool movingSphereIntersect( HAPIFloat radius,
                                          const Vec3 &from, 
                                          const Vec3 &to );

      /// Detect collision between a moving sphere and the object and returns
      /// information about the closest intersection. Is of course slower
      /// than the version of movingSphereIntersect which does not return
      /// information about the intersection.
      /// \param radius The radius of the sphere
      /// \param from The start position of the sphere
      /// \param to The end position of the sphere.
      /// \param result Contains information about the closest intersection
      /// if an intersection is detected.
      /// \returns true if intersected, false otherwise.
      virtual bool movingSphereIntersect( HAPIFloat radius,
                                          const Vec3 &from, 
                                          const Vec3 &to,
                                          IntersectionInfo &result);

      /// Render the object. The caller of the function need to set up OpenGL
      /// state in case the rendering should be done differently
      /// (wireframe for example).
      virtual void render();

      /// The coordinate of the center of the sphere.
      Vec3 center;

      /// The radius of the sphere.
      HAPIFloat radius;
    };


    /// \ingroup CollisionStructures
    /// \class Triangle
    /// \brief Represents a triangle primitive.
    class HAPI_API Triangle: public GeometryPrimitive  {
    public:
      /// Default constructor.
      Triangle() {}

      /// Constructor.
      /// a, b and c are the coordinates for each vertex
      /// ta, tb, and tc are the texture coordinates for each vertex.
      Triangle( const Vec3& a_, const Vec3& b_, const Vec3& c_,
                const Vec3& ta_ = Vec3(), 
                const Vec3& tb_ = Vec3(), 
                const Vec3& tc_ = Vec3()) : 
        a(a_),b(b_),c(c_),
        ta( ta_ ), tb( tb_ ), tc( tc_ ) {
        ab = b - a;
        ac = c - a;
        normal = ab % ac;
        normal.normalizeSafe();
      }

      bool getConstraint(  const Vec3 &point,
                           Collision::PlaneConstraint *constraint,
                           FaceType face = Collision::FRONT_AND_BACK );

      /// Calculates a matrix transforming from local space to texture space.
      /// \param point The point at which to find the tangent vectors
      /// \param result_mtx Where the result is stored.
      virtual void getTangentSpaceMatrix( const Vec3 &point,
                                          Matrix4 &result_mtx );

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
                                  FaceType face = Collision::FRONT_AND_BACK );

      /// Detect collision between a moving sphere and the object.
      /// \param radius The radius of the sphere
      /// \param from The start position of the sphere
      /// \param to The end position of the sphere.
      /// \returns true if intersected, false otherwise.
      virtual bool movingSphereIntersect( HAPIFloat radius,
                                          const Vec3 &from, 
                                          const Vec3 &to );

      /// Detect collision between a moving sphere and the object and returns
      /// information about the closest intersection. Is of course slower
      /// than the version of movingSphereIntersect which does not return
      /// information about the intersection.
      /// \param radius The radius of the sphere
      /// \param from The start position of the sphere
      /// \param to The end position of the sphere.
      /// \param result Contains information about the closest intersection
      /// if an intersection is detected.
      /// \returns true if intersected, false otherwise.
      virtual bool movingSphereIntersect( HAPIFloat radius,
                                          const Vec3 &from, 
                                          const Vec3 &to,
                                          IntersectionInfo &result);

      /// Detect collision between a moving sphere and the object.
      /// \param radius The radius of the sphere
      /// \param from The start position of the sphere
      /// \param to The end position of the sphere.
      /// object
      /// \returns true if intersected, false otherwise.
      /// NOTE: This function is deprecated, kept for backwards
      /// compability, all it does is call the corresponding
      /// movingSphereIntersect version.
      bool movingSphereIntersectRobust( HAPIFloat radius,
                                          const Vec3 &from, 
                                          const Vec3 &to );


      /// Returns a point representing the primitive. In this case it is the 
      /// center of the triangle.
      inline virtual Vec3 pointRepresentation() const {
        return ( a + b + c ) / 3;
      }

      /// Render the object. The caller of the function need to set up OpenGL
      /// state in case the rendering should be done differently
      /// (wireframe for example).
      virtual void render();
      
      
    public:
      /// The corners of the triangle.
      Vec3 a,b,c;

      /// The texture coordinates at the corners of the triangle.
      Vec3 ta, tb, tc;

      Vec3 normal;

      Vec3 ac, ab;
    };

    /// \ingroup CollisionStructures
    /// \class LineSegment
    /// \brief Represents a line segment primitive.
    ///
    /// A line segment is a line from point a to point b.
    class HAPI_API LineSegment: public GeometryPrimitive  {
    public:
      /// Default constructor.
      LineSegment() {}

      /// Constructor.
      /// \param _start is the point where the linesegment starts.
      /// \param _end is the point where the linesegement ends.
      LineSegment( const Vec3& _start, const Vec3& _end) : 
        start( _start ), end( _end ) {}

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
                                  FaceType face = Collision::FRONT_AND_BACK );

      /// Detect collision between a moving sphere and the object.
      /// \param radius The radius of the sphere
      /// \param from The start position of the sphere
      /// \param to The end position of the sphere.
      /// \returns true if intersected, false otherwise.
      virtual bool movingSphereIntersect( HAPIFloat radius,
                                          const Vec3 &from, 
                                          const Vec3 &to );

      /// Detect collision between a moving sphere and the object and returns
      /// information about the closest intersection. Is of course slower
      /// than the version of movingSphereIntersect which does not return
      /// information about the intersection.
      /// \param radius The radius of the sphere
      /// \param from The start position of the sphere
      /// \param to The end position of the sphere.
      /// \param result Contains information about the closest intersection
      /// if an intersection is detected.
      /// \returns true if intersected, false otherwise.
      virtual bool movingSphereIntersect( HAPIFloat radius,
                                          const Vec3 &from, 
                                          const Vec3 &to,
                                          IntersectionInfo &result);

      /// Returns a point representing the primitive. In this case it is the 
      /// center of the line segment.
      inline virtual Vec3 pointRepresentation() const {
        return ( start + end ) / 2;
      }

      /// Render the object. The caller of the function need to set up OpenGL
      /// state in case the rendering should be done differently
      /// (wireframe for example).
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

    /// \ingroup CollisionStructures
    /// \class Point
    /// \brief Represents a point primitive.
    class HAPI_API Point: public GeometryPrimitive  {
    public:
      /// Default constructor.
      Point() {}

      /// Constructor.
      Point( const Vec3& _p ) : 
        position( _p ) {}

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
                                  FaceType face = Collision::FRONT_AND_BACK );

      /// Detect collision between a moving sphere and the object.
      /// \param radius The radius of the sphere
      /// \param from The start position of the sphere
      /// \param to The end position of the sphere.
      /// \returns true if intersected, false otherwise.
      virtual bool movingSphereIntersect( HAPIFloat radius,
                                          const Vec3 &from, 
                                          const Vec3 &to );

      /// Detect collision between a moving sphere and the object and returns
      /// information about the closest intersection. Is of course slower
      /// than the version of movingSphereIntersect which does not return
      /// information about the intersection.
      /// \param radius The radius of the sphere
      /// \param from The start position of the sphere
      /// \param to The end position of the sphere.
      /// \param result Contains information about the closest intersection
      /// if an intersection is detected.
      /// \returns true if intersected, false otherwise.
      virtual bool movingSphereIntersect( HAPIFloat radius,
                                          const Vec3 &from, 
                                          const Vec3 &to,
                                          IntersectionInfo &result);

      /// Returns a point representing the primitive. This primitive is a point
      /// so the pointRepresentation is easy to calculate.
      inline virtual Vec3 pointRepresentation() const {
        return position;
      }

      /// Render the object. The caller of the function need to set up OpenGL
      /// state in case the rendering should be done differently
      /// (wireframe for example).
      virtual void render();
      
      /// The position of the point
      Vec3 position;
    };
    
    /// \ingroup CollisionStructures
    /// \class AABoxBound
    /// \brief Represents an axis-aligned bounding box.
    class HAPI_API AABoxBound: public BoundPrimitive {
    public:
      /// Render the object. The caller of the function need to set up OpenGL
      /// state in case the rendering should be done differently
      /// (wireframe for example).
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
        else                          return Vec3(0,0,1);
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
                                  FaceType face = Collision::FRONT_AND_BACK ) {
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
        // is outside box, clamp it to the box, else keep it as it is
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

    /// \ingroup CollisionStructures
    /// \class OrientedBoxBound
    /// \brief Represents an oriented bounding box.
    class HAPI_API OrientedBoxBound: public AABoxBound {
    public:
      /// Render the object. The caller of the function need to set up OpenGL
      /// state in case the rendering should be done differently
      /// (wireframe for example).
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
                                  FaceType face = Collision::FRONT_AND_BACK ) {
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


    /// \ingroup CollisionStructures
    /// \class SphereBound
    /// \brief Represents a bounding sphere.
    class HAPI_API SphereBound: public BoundPrimitive {
    public:
      /// Default constructor.
      SphereBound();

      /// Constructor.
      SphereBound(const Vec3& c, HAPIFloat r);

      /// Destructor
      ~SphereBound();
      
      /// Render the object. The caller of the function need to set up OpenGL
      /// state in case the rendering should be done differently
      /// (wireframe for example).
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
                                  FaceType face = Collision::FRONT_AND_BACK );

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
    };

    /// \ingroup AbstractClasses
    /// \class BinaryBoundTree
    /// \brief The base class for bound objects structured as a binary tree.
    /// 
    /// Each node in the tree has a BoundPrimitive
    /// specifying a bound for itself and each subtree has the same. Each leaf
    /// in the tree contains triangle, line and point primitives contained
    /// within its parent bound.
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
      /// Builds a binary tree from a vector of triangles. The func argument
      /// specifies a function for creating  bound of the wanted type in each
      /// tree node.
      /// max_nr_triangles_in_leaf specifies the maximum number of triangles 
      /// that are allowed to be in a bound of a leaf in the tree. 
      BinaryBoundTree( BoundNewFunc func, 
                       const vector< Triangle > &triangles,
                       unsigned int max_nr_triangles_in_leaf = 1 );

      /// Builds a binary tree from a vector of triangles, lines and points.
      /// The func argument specifies
      /// a function for creating  bound of the wanted type in each tree node.
      /// max_nr_triangles_in_leaf specifies the maximum number of primitives 
      /// that are allowed to be in a bound of a leaf in the tree. 
      BinaryBoundTree( BoundNewFunc func, 
                       const vector< Triangle > &triangle_vector,
                       const vector< LineSegment > &linesegment_vector,
                       const vector< Point > &point_vector,
                       unsigned int max_nr_triangles_in_leaf = 1 );

      /// Returns true if the tree is a leaf, i.e. has no sub-tress
      /// and hence just contains triangles. false otherwise.
      inline bool isLeaf() { 
        return left.get() == NULL && right.get() == NULL; 
      }

      /// Get constraint planes of the shape. A proxy of a haptics renderer
      /// will always stay above any constraints added.
      /// \param point The point to constrain.
      /// \param constraints Where to add the constraints.
      /// \param face Determines which faces of the shape will be seen as
      /// constraining.
      /// \param radius Only add constraints within this radius. If set to -1
      /// all constraints will be added.
      virtual void getConstraints( const Vec3 &point,
                                   Constraints &constraints,
                                   FaceType face = Collision::FRONT_AND_BACK,
                                   HAPIFloat radius = -1 );

      /// Adds the triangles found in the tree, that are within the distance 
      /// radius from p, to the triangles vector.
      virtual void getTrianglesWithinRadius( const Vec3 &p,
                                             HAPIFloat radius,
                                             vector< Triangle > &triangles);

      /// Adds the triangles, lines and points found in the tree, that are 
      /// within the distance radius from p, to their respective vector.
      virtual void getPrimitivesWithinRadius( const Vec3 &p,
                                             HAPIFloat radius,
                                             vector< Triangle > &triangles,
                                             vector< LineSegment > &lines,
                                             vector< Point > &points );

      /// Adds the triangles that are intersected by the volume swept by a
      /// sphere when moving from "from" to "to".
      virtual void getTrianglesIntersectedByMovingSphere(
                    HAPIFloat radius,
                    Vec3 from,
                    Vec3 to,
                    vector< Triangle > &triangles);

      /// Adds the triangles, lines and points that are intersected by 
      /// the volume swept by a sphere when moving from "from" to "to".
      virtual void getPrimitivesIntersectedByMovingSphere(
                    HAPIFloat radius,
                    Vec3 from,
                    Vec3 to,
                    vector< Triangle > &triangles,
                    vector< LineSegment > &lines,
                    vector< Point > &points );

      /// Render the objects in the leafs. The caller of the function need to
      /// set up OpenGL state in case the rendering should be done differently
      /// (wireframe for example).
      virtual void render();

      /// Render the outlines of the tree at a given tree depth for debugging 
      /// purposes.
      virtual void renderBounds( int depth );

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

      /// Detect collision between a line segment and the object. Will check
      /// for collision between the triangles, lines or points contained in the
      /// leaves of the tree and the line segment.
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
                                  FaceType face = Collision::FRONT_AND_BACK );

      /// Detect collision between a moving sphere and the object.
      /// \param radius The radius of the sphere
      /// \param from The start position of the sphere
      /// \param to The end position of the sphere.
      /// \returns true if intersected, false otherwise.
      virtual bool movingSphereIntersect( HAPIFloat radius,
                                          const Vec3 &from, 
                                          const Vec3 &to );

      /// Detect collision between a moving sphere and the object and returns
      /// information about the closest intersection. Is of course slower
      /// than the version of movingSphereIntersect which does not return
      /// information about the intersection.
      /// \param radius The radius of the sphere
      /// \param from The start position of the sphere
      /// \param to The end position of the sphere.
      /// \param result Contains information about the closest intersection
      /// if an intersection is detected.
      /// \returns true if intersected, false otherwise.
      virtual bool movingSphereIntersect( HAPIFloat radius,
                                          const Vec3 &from, 
                                          const Vec3 &to,
                                          IntersectionInfo &result);

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
      /// the closest point to the triangles, lines or points in the leaf is
      /// returned. To know the closest point to the primitives in the bound,
      /// use closestPoint.
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


    /// \ingroup CollisionStructures
    /// \class AABBTree
    /// \brief A BinaryBoundTree where the bounding primitive
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
        BinaryBoundTree( &(newInstance< AABoxBound >),
                         triangles,
                         max_nr_triangles_in_leaf ) {}

      /// Builds a binary tree from a vector of triangles, lines and points.
      /// max_nr_triangles_in_leaf specifies the maximum number of primitives 
      /// that are allowed to be in a bound of a leaf in the tree. 
      AABBTree( const vector< Triangle > &triangles,
                const vector< LineSegment > &linesegment_vector,
                const vector< Point > &point_vector,
                unsigned int max_nr_triangles_in_leaf = 1 ):
        BinaryBoundTree( &(newInstance< AABoxBound >), triangles,
                         linesegment_vector, point_vector,
                         max_nr_triangles_in_leaf ) {}
    };


    /// \ingroup CollisionStructures
    /// \class OBBTree
    /// \brief a BinaryBoundTree where the bounding primitive
    /// for each node is an OrientedBoxBound.
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
      
      /// Builds a binary tree from a vector of triangles, lines and points.
      /// max_nr_triangles_in_leaf specifies the maximum number of primitives 
      /// that are allowed to be in a bound of a leaf in the tree. 
      OBBTree( const vector< Triangle > &triangles,
                const vector< LineSegment > &linesegment_vector,
                const vector< Point > &point_vector,
                unsigned int max_nr_triangles_in_leaf = 1 ):
        BinaryBoundTree( &(newInstance< OrientedBoxBound >), triangles,
                         linesegment_vector, point_vector,
                         max_nr_triangles_in_leaf ) {}
    };

    /// \ingroup CollisionStructures
    /// \class SphereBoundTree
    /// \brief A BinaryBoundTree where the bounding
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
        BinaryBoundTree( &(newInstance< SphereBound > ),
                         triangles,
                         max_nr_triangles_in_leaf ) {}

      /// Builds a binary tree from a vector of triangles, lines and points.
      /// max_nr_triangles_in_leaf specifies the maximum number of primitives 
      /// that are allowed to be in a bound of a leaf in the tree. 
      SphereBoundTree( const vector< Triangle > &triangles,
                const vector< LineSegment > &linesegment_vector,
                const vector< Point > &point_vector,
                unsigned int max_nr_triangles_in_leaf = 1 ):
        BinaryBoundTree( &(newInstance< SphereBound >), triangles,
                         linesegment_vector, point_vector,
                         max_nr_triangles_in_leaf ) {}
    };

    /// \ingroup AbstractClasses
    /// \class BBPrimitiveTree
    /// \brief The base class for bound objects using structured as a binary
    /// tree which contains pointers to the GeometryPrimitive class.
    ///
    /// Each node in the tree has a BoundPrimitive
    /// specifying a bound for itself and each subtree has the same. The leaf
    /// nodes can contain any GeometryPrimitive objects.
    class HAPI_API BBPrimitiveTree: public BoundObject {
    public: 
      /// A function type for creating a bound primitive for a node in the tree.
      typedef BoundPrimitive *( *BoundNewFunc)(); 

      /// Convenience template for creating BoundNewFunc functions.
      template< class N >
      static BoundPrimitive *newInstance() { return new N; };

      /// Default constructor.
      BBPrimitiveTree(  ): left( NULL ), right( NULL ), new_func( NULL ) {}

      /// Constructor.
      /// Builds a binary tree from a vector of GeometryPrimitive pointers.
      /// The func argument specifies a function for creating  bound of the
      /// wanted type in each tree node. max_nr_primitives_in_leaf specifies
      /// the maximum number of primitives that are allowed to be in a bound
      /// of a leaf in the tree. 
      BBPrimitiveTree( BoundNewFunc func, 
                       const vector< GeometryPrimitive * > &_primitives,
                       unsigned int max_nr_primitives_in_leaf = 1 );

      /// Returns true if the tree is a leaf, i.e. has no sub-trees
      /// and hence just contains primitives. false otherwise.
      inline bool isLeaf() { 
        return left.get() == NULL && right.get() == NULL; 
      }

      /// Get constraint planes of the object. A proxy of a haptics renderer
      /// will always stay above any constraints added.
      /// \param point The point to constrain.
      /// \param constraints Where to add the constraints.
      /// \param face Determines which faces of the shape will be seen as
      /// constraining.
      /// \param radius Only add constraints within this radius. If set to -1
      /// all constraints will be added.
      virtual void getConstraints( const Vec3 &point,
                                   Constraints &constraints,
                                   FaceType face = Collision::FRONT_AND_BACK,
                                   HAPIFloat radius = -1 );

      /// Adds the primitives found in the tree, that are 
      /// within the distance radius from p, to the vector of primitives.
      virtual void getPrimitivesWithinRadius( const Vec3 &p,
                                             HAPIFloat radius,
                                    vector< GeometryPrimitive * > &primitives);

      /// Adds the primitives that are intersected by the volume swept by a
      /// sphere when moving from "from" to "to".
      virtual void getPrimitivesIntersectedByMovingSphere(
                      HAPIFloat radius,
                      Vec3 from,
                      Vec3 to,
                      vector< GeometryPrimitive * > &primitives);

      /// Render the objects in the leafs. The caller of the function need to
      /// set up OpenGL state in case the rendering should be done differently
      /// (wireframe for example).
      virtual void render();

      /// Render the outlines of the tree at a given tree depth for debugging 
      /// purposes.
      virtual void renderBounds( int depth );

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

      /// Detect collision between a line segment and the object. Will check
      /// for collision between the primitives contained in the leaves of the
      /// tree and the line segment.
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
                                  FaceType face = Collision::FRONT_AND_BACK );

      /// Detect collision between a moving sphere and the object.
      /// \param radius The radius of the sphere
      /// \param from The start position of the sphere
      /// \param to The end position of the sphere.
      /// \returns true if intersected, false otherwise.
      virtual bool movingSphereIntersect( HAPIFloat radius,
                                          const Vec3 &from, 
                                          const Vec3 &to );

      /// Detect collision between a moving sphere and the object and returns
      /// information about the closest intersection. Is of course slower
      /// than the version of movingSphereIntersect which does not return
      /// information about the intersection.
      /// \param radius The radius of the sphere
      /// \param from The start position of the sphere
      /// \param to The end position of the sphere.
      /// \param result Contains information about the closest intersection
      /// if an intersection is detected.
      /// \returns true if intersected, false otherwise.
      virtual bool movingSphereIntersect( HAPIFloat radius,
                                          const Vec3 &from, 
                                          const Vec3 &to,
                                          IntersectionInfo &result);

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
      /// the closest point to the primitives in the leaf is returned.
      /// To know the closest point to the primitives in the bound,
      /// use closestPoint
      virtual Vec3 boundClosestPoint( const Vec3 &p ) {
        if( !isLeaf() && bound.get() )
          return bound->boundClosestPoint( p );
        else {
          Vec3 cp, tmp;
          closestPoint( p, cp, tmp, tmp );
          return cp;
        }
      }

      /// Add all primitives in the tree to the given vector.
      virtual void getAllPrimitives(
        vector< GeometryPrimitive * > &primitives );

      H3DUtil::AutoRef< BoundPrimitive > bound;
      H3DUtil::AutoRefVector< GeometryPrimitive > primitives;

      /// The left subtree.
      H3DUtil::AutoRef< BBPrimitiveTree > left;
      /// The right subtree.
      H3DUtil::AutoRef< BBPrimitiveTree > right;
      /// The function to create new bounding primitives in for a tree node.
      BoundNewFunc new_func;
    };


    /// \ingroup CollisionStructures
    /// \class AABBPrimitiveTree
    /// \brief A BBPrimitiveTree where the bounding
    /// primitive for each node is a AABoxBound (axis-aligned bounding box).
    class HAPI_API AABBPrimitiveTree: public BBPrimitiveTree {
    public:
      /// Default constructor.
      AABBPrimitiveTree(): BBPrimitiveTree( ) {}

      /// Constructor.
      /// Builds a binary tree from a vector of primitives. 
      /// max_nr_primitives_in_leaf specifies the maximum number of primitives 
      /// that are allowed to be in a bound of a leaf in the tree. 
      AABBPrimitiveTree( const vector< GeometryPrimitive * > &_primitives,
                unsigned int max_nr_primitives_in_leaf = 1 ):
        BBPrimitiveTree( &(newInstance< AABoxBound >),
                         _primitives,
                         max_nr_primitives_in_leaf ) {}
    };


    /// \ingroup CollisionStructures
    /// \class OBBPrimitiveTree
    /// \brief A BBPrimitiveTree where the bounding
    /// primitive for each node is a OrientedBoxBound.
    class HAPI_API OBBPrimitiveTree: public BBPrimitiveTree {
    public:
      /// Default constructor.
      OBBPrimitiveTree(): BBPrimitiveTree( ) {}

      /// Constructor.
      /// Builds a binary tree from a vector of primitives. 
      /// max_nr_primitives_in_leaf specifies the maximum number of primitives 
      /// that are allowed to be in a bound of a leaf in the tree. 
      OBBPrimitiveTree( const vector< GeometryPrimitive * > &_primitives,
               unsigned int max_nr_primitives_in_leaf = 1 ):
        BBPrimitiveTree( &(newInstance< OrientedBoxBound >), _primitives, 
                         max_nr_primitives_in_leaf ) {}
    };

    /// \ingroup CollisionStructures
    /// \class SBPrimitiveTree
    /// \brief A BBPrimitiveTree where the bounding 
    /// primitive for each node is a SphereBound object.
    class HAPI_API SBPrimitiveTree: public BBPrimitiveTree {
    public:
      /// Default constructor.
      SBPrimitiveTree(): BBPrimitiveTree() {}

      /// Constructor.
      /// Builds a binary tree from a vector of primitives. 
      /// max_nr_primitives_in_leaf specifies the maximum number of primitives 
      /// that are allowed to be in a bound of a leaf in the tree. 
      SBPrimitiveTree( const vector< GeometryPrimitive * > &_primitives,
                       unsigned int max_nr_primitives_in_leaf = 1): 
        BBPrimitiveTree( &(newInstance< SphereBound > ),
                         _primitives,
                         max_nr_primitives_in_leaf ) {}
    };

}
  using Collision::PlaneConstraint;
}

#endif
