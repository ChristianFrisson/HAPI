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
/// \file CollisionObjects.cpp
/// \brief Cpp file for CollisionObjects
///
//
//////////////////////////////////////////////////////////////////////////////

#include <HAPI/CollisionObjects.h>
#include <HAPI/PlaneConstraint.h>

#ifdef HAVE_OPENGL
#ifdef MACOSX
#include <OpenGL/glu.h>
#else
#include <GL/glu.h>
#endif
#endif

#ifdef MACOSX
#ifdef __clang__
#pragma clang diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#endif
#endif


#include <stack>
#include <map>

#ifdef H3D_WINDOWS
#undef max
#undef min
#endif
#include <limits>

using namespace HAPI;
using namespace Collision;
using namespace std;

namespace CollisionInternals {
  // Intersects segment r = p * t * d, |d| = tmax,
  // 0 <= t <= tmax with sphere
  // defined by point s_c and r and, if intersecting, returns t value of
  // interseection and intersection point q.
  // Used by intersectSegmentCapsule
  bool intersectSegmentSphere( Vec3 p, Vec3 d, Vec3 s_c, HAPIFloat r,
                               HAPIFloat &t ) {
    HAPIFloat tmax = d * d;
    if( tmax > 0.0f ) {
      tmax = H3DUtil::H3DSqrt( tmax );
      d = d / tmax;
    } else {
      Vec3 m = p - s_c;
      HAPIFloat c = m * m - r * r;
      if( c > 0.0f )
        return false;
      else {
        t = 0;
        //q = p;
        return true;
      }
    }
    Vec3 m = p - s_c;
    HAPIFloat c = m * m - r * r;
    HAPIFloat b = m * d;
    //Exit if r's origin outside s (c > 0) and r pointing away from s (b > 0)
    if( c > 0.0f && b > 0.0f ) return false;
    HAPIFloat discr = b * b - c;
    // A negative discriminant corresponds to ray missing sphere.
    if( discr < 0.0f ) return false;
    // Ray now found to intersect sphere, compute smallest t value of
    // intersection.
    t = - b - H3DUtil::H3DSqrt( discr );
    // If t > tmax then sphere is missed.
    if( t > tmax )
      return false;
    // If t is negative then segment started inside sphere, so clamp t to
    // zero.
    if( t < 0.0f ) t = 0.0f;
    // Calculate intersection point.
    //q = p + d * t;
    // Set t to the interval 0 <= t <= tmax.
    t = t / tmax;
    return true;
  }

  /// Function for intersecting a segment with a capsule. Return time t
  /// of intersection. Used by AABox::movingSphereIntersect.
  bool intersectSegmentCapsule( Vec3 sa, Vec3 sb, Vec3 p, Vec3 q,
                                HAPIFloat r, HAPIFloat &t ) {
    Vec3 d = q - p, m = sa - p, n = sb - sa;
    HAPIFloat md = m * d;
    HAPIFloat nd = n * d;
    HAPIFloat dd = d * d;
    // Test if segment fully outside either endcap of capsule.
    if( md < 0.0f && md + nd < 0.0f ) {
      // Segment outside 'p'.
      return intersectSegmentSphere( sa, n, p, r, t );
    }
    if( md > dd && md + nd > dd ) {
      // Segment outside 'q'
      return intersectSegmentSphere( sa, n, q, r, t );
    }
    HAPIFloat nn = n * n;
    HAPIFloat mn = m * n;
    HAPIFloat a = dd * nn - nd * nd;
    HAPIFloat k = m * m - r * r;
    HAPIFloat c = dd * k - md * md;
    if( H3DUtil::H3DAbs(a) < Constants::epsilon ) {
      // Segment runs parallel to cylinder axis.
      if( c > 0.0f ) return 0; // 'a' and thus the segment lies outside cyl.
      // Now known that segment intersects cylinder. Figure out how.
      if( md < 0.0f )
        // Intersect against 'p' endcap.
        intersectSegmentSphere( sa, n, p, r, t );
      else if( md > dd )
        // Intersect against 'q' encap.
        intersectSegmentSphere( sa, n, q, r, t );
      else t = 0.0f; // 'a' lies inside cylinder.
      return true;
    }
    HAPIFloat b = dd * mn - nd * md;
    HAPIFloat discr = b * b - a * c;
    if( discr < 0.0f ) return false; // No real roots; no intersection.

    t = (-b - H3DUtil::H3DSqrt( discr )) / a;
    HAPIFloat t0 = t;
    if( md + t * nd < 0.0f ) {
      // Intersection outside cylinder on 'p' side;
      return intersectSegmentSphere( sa, n, p, r, t );
    } else if( md + t * nd > dd ) {
      // Intsection outside cylinder on 'q' side.
      return intersectSegmentSphere( sa, n, q, r, t );
    }
    t = t0;
    // Intersection if segment intersects cylinder between end-caps.
    return t >= 0.0f && t <= 1.0f;
  }

  // Help function to AABox::movingSphereIntersect
  bool findTMin( HAPIFloat &tmin,
                 Vec3 &normal,
                 const Vec3 &d,
                 const Vec3 &_min,
                 const Vec3 &_max,
                 const Vec3 &from,
                 const Vec3 &to ) {
    HAPIFloat tmax = 1.0f;
    // For all three slabs
    for( int i = 0; i < 3; ++i ) {
      if( H3DUtil::H3DAbs(d[i]) < Constants::epsilon ) {
        // Ray is parallel to slab. No hit if origin not within slab.
        if( from[i] < _min[i] || from[i] > _max[i] ) return false;
      } else {
        // Compute intersection t value of ray with near and far plane of slab.
        HAPIFloat ood = 1.0f / d[i];
        HAPIFloat t1 = (_min[i] - from[i]) * ood;
        HAPIFloat t2 = (_max[i] - from[i]) * ood;
        // Make t1 be intersection with near plane, t2 with far plane.
        bool swapped = false;
        if( t1 > t2 ) {
          HAPIFloat tmp = t1;
          t1 = t2;
          t2 = tmp;
          swapped = true;
        }
        // Compute intersection of slab intersection intervals.
        if( t1 > tmin ) {
          tmin = t1;
          normal = Vec3();
          if( swapped ) normal[i] = 1;
          else normal[i] = -1;
        }
        tmin = H3DUtil::H3DMax( tmin, t1 );
        tmax = H3DUtil::H3DMin( tmax, t2 );
        // Exit with no collision as soon as slab intersection becomes empty.
        if( tmin > tmax ) return false;
      }
    }
    return true;
  }

  // Used by Sphere::getTangentSpaceMatrix
  HAPIFloat pi_inv = 1 / H3DUtil::Constants::pi;
  HAPIFloat two_pi_inv = pi_inv / 2;

#ifdef HAVE_OPENGL
  // Used by Sphere and SphereBound for rendering of a sphere.
  H3DUtil::MutexLock nr_of_spheres_lock;
  unsigned int nr_of_spheres = 0;
  GLuint sphere_display_list_id;
  bool sphere_display_list_created = false;
  
  void createSphereDisplayList() {
    if( !sphere_display_list_created ) {
      sphere_display_list_id = glGenLists(1);
      glNewList( sphere_display_list_id, GL_COMPILE );
      GLUquadricObj* pObj = gluNewQuadric(); 
      gluSphere(pObj, 1, 10, 10); 
      gluDeleteQuadric(pObj); 
      glEndList();
      sphere_display_list_created = true;
    }
  }

  void increaseSphereCounter() {
    nr_of_spheres_lock.lock();
    ++nr_of_spheres;
    nr_of_spheres_lock.unlock();
  }

  void decreseSphereCounter() {
    nr_of_spheres_lock.lock();
    --nr_of_spheres;
    if( nr_of_spheres == 0 && sphere_display_list_created ) {
      glDeleteLists( sphere_display_list_id, 1 );
      sphere_display_list_created = false;
    }
    nr_of_spheres_lock.unlock();
  }

  H3DUtil::MutexLock nr_of_cylinders_lock;
  unsigned int nr_of_cylinders = 0;
  GLuint cylinder_display_list_id;
  bool cylinder_display_list_created = false;
  
  void createCylinderDisplayList() {
    if( !cylinder_display_list_created ) {
      cylinder_display_list_id = glGenLists(1);
      glNewList( cylinder_display_list_id, GL_COMPILE );
      GLUquadricObj* pObj = gluNewQuadric(); 
      gluCylinder(pObj, 1, 1, 1, 10, 10); 
      gluDeleteQuadric(pObj); 
      glEndList();
      cylinder_display_list_created = true;
    }
  }

  void increaseCylinderCounter() {
    nr_of_cylinders_lock.lock();
    ++nr_of_cylinders;
    nr_of_cylinders_lock.unlock();
  }

  void decreseCylinderCounter() {
    nr_of_cylinders_lock.lock();
    --nr_of_cylinders;
    if( nr_of_cylinders == 0 && cylinder_display_list_created ) {
      glDeleteLists( cylinder_display_list_id, 1 );
      cylinder_display_list_created = false;
    }
    nr_of_cylinders_lock.unlock();
  }
#endif
}

bool Collision::intersectSegmentCylinder( Vec3 sa, Vec3 sb,
                                          Vec3 p, Vec3 q,
                                          HAPIFloat r, 
                                          HAPIFloat & t ) {
  Vec3 d = q - p, m = sa - p, n = sb - sa;
  HAPIFloat md = m * d;
  HAPIFloat nd = n * d;
  HAPIFloat dd = d * d;
  //Test if segment fully outside either endcap of cylinder
  if( md < 0.0f && md + nd < 0.0f )
    return false; // Segment outside 'p' side of cylinder
  if( md > dd && md + nd > dd )
    return false; // Segment outside 'q' side of cylinder
  HAPIFloat nn = n * n;
  HAPIFloat mn = m * n;
  HAPIFloat a = dd * nn - nd * nd;
  HAPIFloat k = m * m - r * r;
  HAPIFloat c = dd * k - md * md;
  if( H3DUtil::H3DAbs(a) < Constants::epsilon ) {
    // Segment runs parallel to cylinder axis
    if( c > 0.0f )
      return false; // 'a' and thus the segment lie outside cylinder.
    // Now known that segment intersects cylinder; figure out how it intersects
    if( md < 0.0f ) return false; // Intersect segment against 'p' endcap
    else if( md > dd ) return false; // Intersect segment against 'q' endcap
    else t = 0.0f; // 'a' lies inside cylinder
    return true;
  }
  HAPIFloat b = dd * mn - nd * md;
  HAPIFloat discr = b * b - a * c;
  if( discr < 0.0f )
    return false; // no real roots; no intersection

  t = (-b - H3DUtil::H3DSqrt( discr ) ) / a;
  // Intersection if segment intersects cylinder between the end-caps.
  // We don't care about end-caps intersection for use anywhere in these
  // algorithms.
  return t >= 0.0f && t <= 1.0f;
}

 /// Constructor.
AABoxBound::AABoxBound( const Vec3 &_min,
                        const Vec3 &_max ):
  min( _min ), 
  max( _max ) {
}

void AABoxBound::render() {
#ifdef HAVE_OPENGL
  glDisable( GL_LIGHTING );

  glColor3d( 1, 1, 0 );
  glBegin( GL_LINE_STRIP );
  glVertex3d( min.x, min.y, min.z );
  glVertex3d( min.x, max.y, min.z );
  glVertex3d( max.x, max.y, min.z );
  glVertex3d( max.x, min.y, min.z );
  glVertex3d( min.x, min.y, min.z );
  glEnd();

  glBegin( GL_LINE_STRIP );
  glVertex3d( min.x, min.y, max.z );
  glVertex3d( min.x, max.y, max.z );
  glVertex3d( max.x, max.y, max.z );
  glVertex3d( max.x, min.y, max.z );
  glVertex3d( min.x, min.y, max.z );
  glEnd();

  glBegin( GL_LINES );
  glVertex3d( min.x, min.y, max.z );
  glVertex3d( min.x, min.y, min.z );
  glVertex3d( max.x, min.y, max.z );
  glVertex3d( max.x, min.y, min.z );
  glVertex3d( min.x, max.y, max.z );
  glVertex3d( min.x, max.y, min.z );
  glVertex3d( max.x, max.y, max.z );
  glVertex3d( max.x, max.y, min.z );
  glEnd();
  glEnable( GL_LIGHTING );
#endif
}

void AABoxBound::fitAroundPoints( const vector< Vec3 > &points ) {
  if( points.size() == 0 ) {
    max = Vec3( 0, 0, 0 );
    min = Vec3( 0, 0, 0 );
    return;
  }
  min = points[0];
  max = points[0];
  
  for(unsigned int i=1 ; i<points.size() ; ++i) {
    if (points[i].x < min.x) min.x = points[i].x;
    if (points[i].y < min.y) min.y = points[i].y;
    if (points[i].z < min.z) min.z = points[i].z;
    
    if (points[i].x > max.x) max.x = points[i].x;
    if (points[i].y > max.y) max.y = points[i].y;
    if (points[i].z > max.z) max.z = points[i].z;
  }
}


bool AABoxBound::boundIntersect( const Vec3 &from, 
                                 const Vec3 &to ) {
  Vec3 c = 0.5f*(min+max);
  Vec3 e = max-c;
  Vec3 m = 0.5f*(from+to);
  Vec3 d = to-m;

  m = m-c;

  HAPIFloat adx = H3DUtil::H3DAbs(d.x);
  if (H3DUtil::H3DAbs(m.x) > e.x + adx) return false;
  HAPIFloat ady = H3DUtil::H3DAbs(d.y);
  if (H3DUtil::H3DAbs(m.y) > e.y + ady) return false;
  HAPIFloat adz = H3DUtil::H3DAbs(d.z);
  if (H3DUtil::H3DAbs(m.z) > e.z + adz) return false;

  // bias with an epsilon to increase robustness when segment is parallel
  // to a box plane.
  HAPIFloat epsilon = (e*e)/100000;
  adx += epsilon;
  ady += epsilon;
  adz += epsilon;

  if (H3DUtil::H3DAbs(m.y*d.z - m.z*d.y) > e.y*adz + e.z*ady) return false;
  if (H3DUtil::H3DAbs(m.z*d.x - m.x*d.z) > e.x*adz + e.z*adx) return false;
  if (H3DUtil::H3DAbs(m.x*d.y - m.y*d.x) > e.x*ady + e.y*adx) return false;

  return true;
}



void AABoxBound::closestPoint( const Vec3 &p,
                               Vec3 &closest_point,
                               Vec3 &normal,
                               Vec3 &tex_coord ) {
  assert( false );
}

bool AABoxBound::insideBound( const Vec3 &p ) {
  return ( p.x <= max.x &&
           p.x >= min.x &&
           p.y <= max.y &&
           p.y >= min.y &&
           p.z <= max.z &&
           p.z >= min.z );
}

SphereBound::SphereBound() {
#ifdef HAVE_OPENGL
  CollisionInternals::increaseSphereCounter();
#endif
}

SphereBound::SphereBound(const Vec3& c, HAPIFloat r): center (c), radius(r) {
#ifdef HAVE_OPENGL
  CollisionInternals::increaseSphereCounter();
#endif
}

SphereBound::~SphereBound() {
#ifdef HAVE_OPENGL
  CollisionInternals::decreseSphereCounter();
#endif
}

void SphereBound::closestPoint( const Vec3 &p,
                                Vec3 &closest_point,
                                Vec3 &normal,
                                Vec3 &tex_coord ) {
  Vec3 dir = p - center;
  if( dir * dir < Constants::epsilon ) {
    closest_point = center + Vec3( radius, 0, 0 ); 
    normal = Vec3( 1, 0, 0 );
  } else {
    dir.normalize();
    closest_point = center + dir * radius;
    normal = dir;
  }

  tex_coord = Vec3();
}

bool SphereBound::insideBound( const Vec3 &p ) {
  Vec3 v = p - center;
  HAPIFloat d2 = v * v;
  HAPIFloat r2 = radius * radius;
  return d2 <= r2;
}

bool SphereBound::boundIntersect( const Vec3 &from, 
                                  const Vec3 &to ) {
  Vec3 ab = to - from;
  Vec3 ap = center-from;
  Vec3 bp = center-to;
  HAPIFloat r2 = radius * radius;

  HAPIFloat e = ap * ab;
  if (e<0)  return ap*ap <= r2;
  
  HAPIFloat f = ab * ab;
  if (e>=f) return bp*bp <= r2;
  
  return (ap*ap - e*e/f) <= r2;
}

bool SphereBound::movingSphereIntersect( HAPIFloat _radius,
                                         const Vec3 &from, 
                                         const Vec3 &to ) {
  Sphere sphere( center, radius );
  return sphere.movingSphereIntersect( _radius, from, to );
}

bool SphereBound::lineIntersect( const Vec3 &from, 
                                 const Vec3 &to,
                                 IntersectionInfo &result,
                                 FaceType face ) {
  Sphere sphere( center, radius );
  return sphere.lineIntersect( from, to, result, face );
}

void SphereBound::render() {
#ifdef HAVE_OPENGL
  CollisionInternals::createSphereDisplayList();

  glDisable( GL_LIGHTING );
  glMatrixMode( GL_MODELVIEW );
  glPushMatrix();
  glTranslated( center.x, center.y, center.z );
  glScaled( radius, radius, radius );
  glColor3d( 1, 1, 0 );
  glPushAttrib( GL_POLYGON_BIT );
  glPolygonMode( GL_FRONT_AND_BACK, GL_LINE );
  glCallList( CollisionInternals::sphere_display_list_id );
  glPopAttrib();
  glPopMatrix();
  glEnable( GL_LIGHTING );
#endif
}

void SphereBound::fitAroundPoints( const vector< Vec3 > &points ) {
  if( points.size() == 0 ) {
    center = Vec3( 0, 0, 0 );
    radius = 0;
    return;
  }

  AABoxBound box;
  box.fitAroundPoints( points );
  Vec3 c = 0.5 * (box.min + box.max);
  HAPIFloat r = 0;
  
  // find largest square distance
  for(unsigned int i=0 ; i<points.size() ; ++i) {
    Vec3 from_center = points[i]-c;
    HAPIFloat d = from_center * from_center;
    if (d > r) r = d;
  }
  
  center = c;
  radius = H3DUtil::H3DSqrt( r );
}

struct StackElement {
  BinaryBoundTree *tree;
  vector< int > triangles;
  vector< int > linesegments;
  vector< int > points;
}; 

BinaryBoundTree::BinaryBoundTree( BoundNewFunc func, 
                                  const vector< Triangle > &triangle_vector,
                                  unsigned int max_nr_triangles_in_leaf ):
  left( NULL ), right( NULL ), new_func( func ) {

  std::stack< StackElement > stack;
  
  if( triangle_vector.size() == 0 ) return;
 
  // add the starting subtree to stack...
  StackElement start;
  start.tree = this;

  vector< Vec3 > point_reps;
  point_reps.reserve( triangle_vector.size() );

  start.triangles.reserve( triangle_vector.size() );
  for( unsigned int i = 0; i < triangle_vector.size(); ++i ) {
    start.triangles.push_back( i );
    point_reps.push_back( triangle_vector[i].pointRepresentation() );
  }



  stack.push( start );
  //cerr << start.triangles.size() << endl;
  // BUILD TREE
  while( !stack.empty() ) {
    const std::vector< int > &stack_triangles = stack.top().triangles;
    BinaryBoundTree* stack_tree = stack.top().tree;

    std::vector<Vec3 > fitting_points;
    points.reserve( stack_triangles.size() * 3 );
    for (unsigned int i=0 ; i<stack_triangles.size() ; ++i ) {
      const Triangle &tri = triangle_vector[ stack_triangles[i ] ];
      fitting_points.push_back( tri.a );
      fitting_points.push_back( tri.b );
      fitting_points.push_back( tri.c);
    }
    
    // build bounding shape
    stack_tree->bound.reset( new_func() );
    stack_tree->bound->fitAroundPoints( fitting_points );
    
    if ( stack_triangles.size() <= max_nr_triangles_in_leaf ) {
      //  build a leaf
      stack_tree->triangles.reserve( stack_triangles.size() );
      for( unsigned int i = 0; i < stack_triangles.size(); ++i ) {
        stack_tree->triangles.push_back( triangle_vector[stack_triangles[i]] );
      }
      stack.pop();
      continue;
    }


    // DIVIDE SUBSET AND RECURSE
    // longest axis
    Vec3 axis = stack_tree->bound->longestAxis();

    // middle point of current set
    Vec3 mid(0,0,0);

    for(unsigned int i = 0 ; i < stack_triangles.size() ; ++i) {
      mid = mid + point_reps[stack_triangles[i]];
    }

    mid = (1.0f/stack_triangles.size()) * mid;  //divide by N
    
    //  build subsets based on axis/middle point
    std::vector<int> left_points;
    std::vector<int> right_points;
    
    left_points.reserve( stack_triangles.size() / 2 );
    right_points.reserve( stack_triangles.size() / 2 );

    
    for( unsigned int i = 0 ; i<stack_triangles.size() ; ++i ) {
      Vec3 mid_to_point = point_reps[ stack_triangles[i] ] - mid;
      if ( mid_to_point * axis < 0 )
        left_points.push_back(stack_triangles[i]);
      else
        right_points.push_back(stack_triangles[i]);
    }
    
    // anity check... sometimes current subset cannot be divided by longest
    // axis: change axis or just half
    if ( (left_points.size() == 0) || (right_points.size() == 0) ) {
      left_points.clear();
      right_points.clear();
      for (size_t i = 0 ; i<stack_triangles.size()/2 ; ++i) 
        left_points.push_back(stack_triangles[i]);
      for (size_t i = stack_triangles.size()/2 ;
           i<stack_triangles.size() ; ++i)
        right_points.push_back(stack_triangles[i]);
    }
    
    stack.pop();

    // do recurse
    if (left_points.size() != 0) {
      stack_tree->left.reset( new BinaryBoundTree );
      
      StackElement element;
      element.tree = stack_tree->left.get();
      element.triangles.swap(left_points);
      stack.push( element );
    }
    
    if (right_points.size() != 0) {
      stack_tree->right.reset( new BinaryBoundTree );
      
      StackElement element;
      element.tree = stack_tree->right.get();
      element.triangles.swap( right_points );
      stack.push( element );
    }
  }
} 

BinaryBoundTree::BinaryBoundTree( BoundNewFunc func, 
                       const vector< Triangle > &triangle_vector,
                       const vector< LineSegment > &linesegment_vector,
                       const vector< Point > &point_vector,
                       unsigned int max_nr_triangles_in_leaf ):
  left( NULL ), right( NULL ), new_func( func ) {

  std::stack< StackElement > stack;
  
  if( triangle_vector.size() == 0 &&
      linesegment_vector.size() == 0 &&
      point_vector.size() == 0 ) return;
 
  // add the starting subtree to stack...
  StackElement start;
  start.tree = this;

  vector< Vec3 > point_reps_triangles;
  point_reps_triangles.reserve( triangle_vector.size() );

  vector< Vec3 > point_reps_linesegments;
  point_reps_linesegments.reserve( linesegment_vector.size() );

  vector< Vec3 > point_reps_points;
  point_reps_points.reserve( point_vector.size() );

  start.triangles.reserve( triangle_vector.size() );
  for( unsigned int i = 0; i < triangle_vector.size(); ++i ) {
    start.triangles.push_back( i );
    point_reps_triangles.push_back( triangle_vector[i].pointRepresentation() );
  }

  start.linesegments.reserve( linesegment_vector.size() );
  for( unsigned int i = 0; i < linesegment_vector.size(); ++i ) {
    start.linesegments.push_back( i );
    point_reps_linesegments.push_back(
      linesegment_vector[i].pointRepresentation() );
  }

  start.points.reserve( point_vector.size() );
  for( unsigned int i = 0; i < point_vector.size(); ++i ) {
    start.points.push_back( i );
    point_reps_points.push_back( point_vector[i].pointRepresentation() );
  }

  stack.push( start );
  //cerr << start.triangles.size() << endl;
  // BUILD TREE
  while( !stack.empty() ) {
    const std::vector< int > &stack_triangles = stack.top().triangles;
    const std::vector< int > &stack_linesegments = stack.top().linesegments;
    const std::vector< int > &stack_points = stack.top().points;
    BinaryBoundTree* stack_tree = stack.top().tree;

    std::vector<Vec3 > fitting_points;
    fitting_points.reserve( stack_triangles.size() * 3 +
      stack_linesegments.size() * 2 + stack_points.size() );
    for (unsigned int i=0 ; i<stack_triangles.size() ; ++i ) {
      const Triangle &tri = triangle_vector[ stack_triangles[i ] ];
      fitting_points.push_back( tri.a );
      fitting_points.push_back( tri.b );
      fitting_points.push_back( tri.c);
    }

    for (unsigned int i=0 ; i<stack_linesegments.size() ; ++i ) {
      const LineSegment &ls = linesegment_vector[ stack_linesegments[i ] ];
      fitting_points.push_back( ls.start );
      fitting_points.push_back( ls.end );
    }

    for (unsigned int i=0 ; i<stack_points.size() ; ++i ) {
      const Point &p = point_vector[ stack_points[i ] ];
      fitting_points.push_back( p.position );
    }
    
    // build bounding shape
    stack_tree->bound.reset( new_func() );
    stack_tree->bound->fitAroundPoints( fitting_points );
    
   if ( ( stack_triangles.size() <= max_nr_triangles_in_leaf &&
          stack_linesegments.size() <= max_nr_triangles_in_leaf &&
          stack_points.size() <= max_nr_triangles_in_leaf ) ) {
      // build a leaf
      stack_tree->triangles.reserve( stack_triangles.size() );
      for( unsigned int i = 0; i < stack_triangles.size(); ++i ) {
        stack_tree->triangles.push_back( triangle_vector[stack_triangles[i]] );
      }

      stack_tree->linesegments.reserve( stack_linesegments.size() );
      for( unsigned int i = 0; i < stack_linesegments.size(); ++i ) {
        stack_tree->linesegments.push_back(
          linesegment_vector[stack_linesegments[i]] );
      }

      stack_tree->points.reserve( stack_points.size() );
      for( unsigned int i = 0; i < stack_points.size(); ++i ) {
        stack_tree->points.push_back( point_vector[stack_points[i]] );
      }
      stack.pop();
      continue;
    }

    // DIVIDE SUBSET AND RECURSE
    // longest axis
    Vec3 axis = stack_tree->bound->longestAxis();

    // middle point of current set
    Vec3 mid(0,0,0);

    for(unsigned int i = 0 ; i < stack_triangles.size() ; ++i) {
      mid = mid + point_reps_triangles[stack_triangles[i]];
    }

    for(unsigned int i = 0 ; i < stack_linesegments.size() ; ++i) {
      mid = mid + point_reps_linesegments[stack_linesegments[i]];
    }

    for(unsigned int i = 0 ; i < stack_points.size() ; ++i) {
      mid = mid + point_reps_points[stack_points[i]];
    }

    mid = (1.0f/(stack_triangles.size() + stack_linesegments.size() +
      stack_points.size()) ) * mid;  //divide by N
    
    // build subsets based on axis/middle point
    std::vector<int> left_triangles;
    std::vector<int> right_triangles;
    
    left_triangles.reserve( stack_triangles.size() / 2 );
    right_triangles.reserve( stack_triangles.size() / 2 );
    
    for( unsigned int i = 0 ; i<stack_triangles.size() ; ++i ) {
      Vec3 mid_to_point = 
        point_reps_triangles[ stack_triangles[i] ] - mid;
      if ( mid_to_point * axis < 0 )
        left_triangles.push_back(stack_triangles[i]);
      else
        right_triangles.push_back(stack_triangles[i]);
    }
    
    // anity check... sometimes current subset cannot be divided by longest 
    // axis: change axis or just half
    if ( (left_triangles.size() == 0) || (right_triangles.size() == 0) ) {
      left_triangles.clear();
      right_triangles.clear();
      for (size_t i = 0 ; i<stack_triangles.size()/2 ; ++i) 
        left_triangles.push_back(stack_triangles[i]);
      for (size_t i = stack_triangles.size()/2;
        i<stack_triangles.size() ; ++i) 
        right_triangles.push_back(stack_triangles[i]);
    }

    // build subsets based on axis/middle point
    std::vector<int> left_linesegments;
    std::vector<int> right_linesegments;
    
    left_linesegments.reserve( stack_linesegments.size() / 2 );
    right_linesegments.reserve( stack_linesegments.size() / 2 );
    
    for( unsigned int i = 0 ; i<stack_linesegments.size() ; ++i ) {
      Vec3 mid_to_point = 
        point_reps_linesegments[ stack_linesegments[i] ] - mid;
      if ( mid_to_point * axis < 0 )
        left_linesegments.push_back(stack_linesegments[i]);
      else
        right_linesegments.push_back(stack_linesegments[i]);
    }
    
    // anity check... sometimes current subset cannot be divided by longest 
    // axis: change axis or just half
    if ( (left_linesegments.size() == 0) || (right_linesegments.size() == 0) ) {
      left_linesegments.clear();
      right_linesegments.clear();
      for (size_t i = 0 ; i<stack_linesegments.size()/2 ; ++i) 
        left_linesegments.push_back(stack_linesegments[i]);
      for (size_t i = stack_linesegments.size()/2 ;
        i<stack_linesegments.size() ; ++i) 
        right_linesegments.push_back(stack_linesegments[i]);
    }

    // build subsets based on axis/middle point
    std::vector<int> left_points;
    std::vector<int> right_points;
    
    left_points.reserve( stack_points.size() / 2 );
    right_points.reserve( stack_points.size() / 2 );
    
    for( unsigned int i = 0 ; i<stack_points.size() ; ++i ) {
      Vec3 mid_to_point = 
        point_reps_points[ stack_points[i] ] - mid;
      if ( mid_to_point * axis < 0 )
        left_points.push_back(stack_points[i]);
      else
        right_points.push_back(stack_points[i]);
    }
    
    // anity check... sometimes current subset cannot be divided by longest 
    // axis: change axis or just half
    if ( (left_points.size() == 0) || (right_points.size() == 0) ) {
      left_points.clear();
      right_points.clear();
      for (size_t i = 0 ; i<stack_points.size()/2 ; ++i) 
        left_points.push_back(stack_points[i]);
      for (size_t i = stack_points.size()/2;
        i<stack_points.size() ; ++i) 
        right_points.push_back(stack_points[i]);
    }
    
    stack.pop();

    // do recurse
    if ( left_triangles.size() != 0 ||
         left_linesegments.size() != 0 ||
         left_points.size() != 0 ) {
      stack_tree->left.reset( new BinaryBoundTree );
      
      StackElement element;
      element.tree = stack_tree->left.get();
      element.triangles.swap( left_triangles );
      element.linesegments.swap( left_linesegments );
      element.points.swap( left_points );
      stack.push( element );
    }
    
    if ( right_triangles.size() != 0 ||
         right_linesegments.size() != 0 ||
         right_points.size() != 0 ) {
      stack_tree->right.reset( new BinaryBoundTree );
      
      StackElement element;
      element.tree = stack_tree->right.get();
      element.triangles.swap( right_triangles );
      element.linesegments.swap( right_linesegments );
      element.points.swap( right_points );
      stack.push( element );
    }
  }
}

void BinaryBoundTree::render() {
  if( !isLeaf() ) {
    if( left.get() ) left->render();
    if( right.get() ) right->render();
  } else {
    for( unsigned int i = 0; i < triangles.size(); ++i ) {
      triangles[i].render();
    }
    for( unsigned int i = 0; i < linesegments.size(); ++i ) {
      linesegments[i].render();
    }
    for( unsigned int i = 0; i < points.size(); ++i ) {
      points[i].render();
    }
  }
}

void Triangle::render() {
#ifdef HAVE_OPENGL
  glBegin( GL_TRIANGLES );
  glNormal3d( normal.x, normal.y, normal.z );
  glVertex3d( a.x, a.y, a.z );
  glVertex3d( b.x, b.y, b.z );
  glVertex3d( c.x, c.y, c.z );
  glEnd();
#endif
}

void BinaryBoundTree::renderBounds( int depth ) {
  if( !isLeaf() ) {
    if( depth == 0 ) {
      bound->render();
    } else {
      if( left.get() ) left->renderBounds( depth - 1 );
      if( right.get() ) right->renderBounds( depth - 1 );
    }
  } else {
#ifdef HAVE_OPENGL
    glPushAttrib( GL_POLYGON_BIT );
    glPolygonMode( GL_FRONT_AND_BACK, GL_LINE );
#endif
    for( unsigned int i = 0; i < triangles.size(); ++i ) {
      triangles[i].render();
    }
    for( unsigned int i = 0; i < linesegments.size(); ++i ) {
      linesegments[i].render();
    }
    for( unsigned int i = 0; i < points.size(); ++i ) {
      points[i].render();
    }
#ifdef HAVE_OPENGL
    glPopAttrib();
#endif
  }
}

 void Triangle::closestPoint( const Vec3 &p,
                              Vec3 &closest_point,
                              Vec3 &return_normal,
                              Vec3 &tex_coord ) {

  Vec3 _ab = b-a;
  Vec3 _ac = c-a;
  Vec3 ap = p-a ;

  Vec3  nn = _ab % _ac;
  nn.normalizeSafe();
  return_normal = nn;

  HAPIFloat d1 = _ab * ap;
  HAPIFloat d2 = _ac * ap;
  if( d1 <= Constants::epsilon && d2 <= Constants::epsilon ) {
    //cerr << "1" << endl;
    tex_coord = ta;
    closest_point = a;
    return;
  }

  Vec3 bp = p - b;
  HAPIFloat d3 = _ab * bp;
  HAPIFloat d4 = _ac * bp;

  if( d3 >= -Constants::epsilon && d4 <= d3 ) {
    //    cerr << "2" << endl;
    tex_coord = tb;
    closest_point = b;
    return;
  }
  
  HAPIFloat vc = d1*d4 - d3*d2;
  if( vc <= Constants::epsilon && 
      d1 >= -Constants::epsilon && 
      d3 <= Constants::epsilon ) {
    // cerr << "3" << endl;
    HAPIFloat v = d1 / ( d1 - d3 );
    closest_point = a + v * _ab;
    tex_coord = ta + v * (tb - ta);
    return;
  }

  Vec3 cp = p - c;
  HAPIFloat d5 = _ab * cp;
  HAPIFloat d6 = _ac * cp;
  if( d6 >= -Constants::epsilon && d5 <= d6 ) {
    //cerr << "4" << endl;
    tex_coord = tc;
    closest_point = c;
    return;
  }

  HAPIFloat vb = d5*d2 - d1*d6;
  if( vb <= Constants::epsilon && 
      d2 >= -Constants::epsilon &&
      d6 <= Constants::epsilon ) {
    //cerr << "5" << endl;
    HAPIFloat w = d2 / ( d2 - d6 );
    closest_point = a + w * _ac;
    tex_coord = ta + w * (tc - ta );
    return;
  }
  
  HAPIFloat va = d3*d6 - d5*d4;
  if( va <= Constants::epsilon &&
      (d4-d3) >= -Constants::epsilon &&
      (d5-d6) >= -Constants::epsilon ) {
    //cerr << "6" << endl;
    HAPIFloat w = (d4-d3) /((d4 -d3) + (d5 - d6 ) );
    closest_point = b + w * (c-b);
    tex_coord = tb + w * (tc - tb );
    return;
  }

  //cerr << "7" << endl;
  HAPIFloat denom = 1 / ( va + vb + vc );
  HAPIFloat v = vb * denom;
  HAPIFloat w = vc * denom;
  closest_point = a + _ab * v + _ac * w;
  tex_coord = ta + (tb - ta) * v + (tc - ta ) * w;
  /*
  Vec3 normal = ca % cb;
  if( H3DUtil::H3DAbs( normal * normal ) > Constants::epsilon ) {
    normal.normalizeSafe();
    
    Vec3 pp = p - (-ap).dotProduct( -normal ) * normal;
    
    // determine dominant axis
    int axis;
    if (H3DUtil::H3DAbs( normal.x ) > H3DUtil::H3DAbs( normal.y )) {
      if (H3DUtil::H3DAbs( normal.x ) > H3DUtil::H3DAbs( normal.z )) axis = 0; 
      else axis = 2;
    } else {
      if (H3DUtil::H3DAbs( normal.y ) > H3DUtil::H3DAbs( normal.z )) axis = 1; 
      else axis = 2;
    }
    
    int u_axis = (axis + 1 ) % 3;
    int v_axis = (axis + 2 ) % 3;
    
    Vec3 cp = pp - c;
    
    HAPIFloat denom = (ca[u_axis] * cb[v_axis] - ca[v_axis] * cb[u_axis]);
    
    HAPIFloat du = 
      (cp[u_axis] * cb[v_axis] - cp[v_axis] * cb[u_axis]) / denom;
    
    if( du >= 0 && du <= 1 ) {
      HAPIFloat dv = 
        (ca[u_axis] * cp[v_axis] - ca[v_axis] * cp[u_axis]) / denom;
      if( dv >= 0 && dv <= 1 ) {
        HAPIFloat dw = 1 - du - dv;
        if( dw >= 0 && dw <= 1 ) {
          return pp;
        }
      }
    }
  }

  // point is on edge.

  Vec3 bp = p-b ;
  Vec3 cp = p-c ;

  HAPIFloat ca_length2 = ca.lengthSqr();
  HAPIFloat cb_length2 = cb.lengthSqr();
  HAPIFloat ab_length2 = _ab.lengthSqr();

  HAPIFloat ca_t = (cp * ca) / ca_length2;
  HAPIFloat cb_t = (cp * cb) / cb_length2;
  HAPIFloat ab_t = (ap * _ab) / ab_length2;

  if( ca_t < 0 && cb_t < 0 )
    return c;

  if( cb_t > 1 && ab_t > 1 )
    return b;

  if( ca_t > 1 && ab_t < 0 )
    return a;
  
  bool discard_ca = ca_t < 0 || ca_t > 1;
  bool discard_cb = cb_t < 0 || cb_t > 1;
  bool discard_ab = ab_t < 0 || ab_t > 1;

  Vec3 ca_p = c + ca * ca_t;
  Vec3 cb_p = c + cb * cb_t;
  Vec3 ab_p = a + _ab * ab_t;
  
  HAPIFloat dist_cap = (ca_p - p).lengthSqr();  
  HAPIFloat dist_cbp = (cb_p - p).lengthSqr();  
  HAPIFloat dist_abp = (ab_p - p).lengthSqr();  

  Vec3 closest;
  HAPIFloat dist;

  if( !discard_ca ) {
    closest = ca_p;
    dist = dist_cap;
  } else if( !discard_cb ) {
    closest = cb_p;
    dist = dist_cbp;
  } else {
    closest = ab_p;
    dist = dist_abp;
  }

  if( !discard_cb && dist_cbp < dist ) {
    closest = cb_p;
    dist = dist_cbp;
  } 

  if( !discard_ab && dist_abp < dist ) {
    closest = ab_p;
  }

  return closest;*/
}

#if 0
bool Triangle::lineIntersect( const Vec3 &from, 
                              const Vec3 &to,
                              IntersectionInfo &result ) {
  Vec3 ca = a-c;
  Vec3 cb = b-c;
  Vec3 dir = to - from;
  Vec3 normal = ca % cb;
  /*if( H3DUtil::H3DAbs( normal * normal ) < Constants::epsilon ||
      H3DUtil::H3DAbs( dir * dir ) < Constants::epsilon ||
      H3DUtil::H3DAbs( normal * dir ) < Constants::epsilon ) {
    return false;
  }*/
  normal.normalizeSafe();

  HAPIFloat dir_length = dir.length();
  dir.normalizeSafe();

  HAPIFloat t = ( normal * ( c - from ) ) / ( normal * dir );
  // check that the point is within the line segement
  if( t < 0 || t > dir_length ) return false;

  Vec3 p = from + t * dir;

  // determine dominant axis
  int axis;
  if (H3DUtil::H3DAbs( normal.x ) > H3DUtil::H3DAbs( normal.y )) {
    if (H3DUtil::H3DAbs( normal.x ) > H3DUtil::H3DAbs( normal.z )) axis = 0; 
    else axis = 2;
  } else {
    if (H3DUtil::H3DAbs( normal.y ) > H3DUtil::H3DAbs( normal.z )) axis = 1; 
    else axis = 2;
  }

  int u_axis = (axis + 1 ) % 3;
  int v_axis = (axis + 2 ) % 3;

  Vec3 cp = p - c;

  HAPIFloat denom = (ca[u_axis] * cb[v_axis] - ca[v_axis] * cb[u_axis]);

  HAPIFloat du = 
    (cp[u_axis] * cb[v_axis] - cp[v_axis] * cb[u_axis]) / denom;

  if( du < 0 || du > 1 ) return false;

  HAPIFloat dv = 
    (ca[u_axis] * cp[v_axis] - ca[v_axis] * cp[u_axis]) / denom;

  if( dv < 0 || dv > 1 ) return false;

  HAPIFloat dw = 1 - du - dv;

  if( dw < 0 || dw > 1 ) {
    return false;
  }

  result.point = p;

  if( dir * normal > 0 ) 
    result.normal = -normal;
  else 
    result.normal = normal;
  
  return true;
  
}

#if 0
                              // line intersect, not segment 
bool Triangle::lineIntersect( const Vec3 &p, 
                              const Vec3 &q,
                              IntersectionInfo &result ) {
  Vec3 pq = q - p;
  Vec3 pa = a - p;
  Vec3 pb = b - p;
  Vec3 pc = c - p;

  Vec3 m = pq.crossProduct( pc );
  HAPIFloat u = pb * m;
  if( u < 0 ) return false;
  HAPIFloat v = -(pa * m );
  if( v < 0 ) return false;
  HAPIFloat w = (pq % pb) * pa;
  if( w < 0 ) return false;

  // compute barycentric coordinates
  HAPIFloat denom = 1.0 / ( u + v + w );
  u *= denom;
  v *= denom;
  w *= denom;

  result.point = u*a + v*b + w*c;
  result.normal = (a - c) % (b - c);
  result.normal.normalizeSafe();
  if( pq * result.normal > 0 ) 
    result.normal = -result.normal;
  return true;
}

#endif

#else
bool Triangle::lineIntersect( const Vec3 &p, 
                              const Vec3 &q,
                              IntersectionInfo &result,
                              FaceType face  ) {
  
  Vec3 v0 = a;
  Vec3 v1 = b;
  Vec3 v2 = c;

  Vec3 tc0 = ta;
  Vec3 tc1 = tb;
  Vec3 tc2 = tc;
 
  Vec3 t_ab = ab;
  Vec3 t_ac = ac;
  Vec3 qp = p - q;

  // Compute normal.
  Vec3 n = t_ab % t_ac;

  // Commented away the epsilon since it gives wrong answer in
  // some cases (very small triangles compared to distance)
  // Comment in in case there are some problems.
  //HAPIFloat epsilon = 1e-8;

  HAPIFloat d = qp * n;

  FaceType intersection_face = Collision::FRONT;
  if ( d < 0 ) {
    // line segment pointing away from triangle
    if( face == Collision::FRONT ) return false;
 
    // flip normal in calculations since they assume starting point is
    // in the direction of the normal
    d = -d;
    n = -n;
    Vec3 tmp = v1;
    v1 = v2;
    v2 = tmp;
    
    tmp = tc1;
    tc1 = tc2;
    tc2 = tmp;

    tmp = t_ab;
    t_ab = t_ac;
    t_ac = tmp;
    intersection_face = Collision::BACK;
  } else if( d == 0 || face == Collision::BACK ) {
    return false;
  }

  Vec3 ap = p - v0;
  HAPIFloat t = ap * n;

  if( t < 0/*-epsilon*/ || t > d/* + epsilon*/ ) return false;

  Vec3 e = qp % ap;
  HAPIFloat v = t_ac * e;
  if( v < 0/*-epsilon*/ || v > d/* + epsilon*/ ) return false;
  HAPIFloat w = -(t_ab * e );
  if( w < 0/*-epsilon*/ || v + w > d/* + epsilon*/) return false;

  HAPIFloat ood = 1 / d;
  t *= ood; 
  v *= ood;
  w *= ood;
  HAPIFloat u = 1 - v - w;

  result.point = u*v0 + v*v1 + w*v2;
  result.normal = normal;
  result.tex_coord =  u*tc0 + v*tc1 + w*tc2;
  result.face = intersection_face;
  result.intersection = true;
  result.primitive = this;
  result.t = t;

  return true;
}

#endif

bool Triangle::movingSphereIntersect( HAPIFloat radius,
                                      const Vec3 &from, 
                                      const Vec3 &to ) {

  Vec3 closest, tmp;
  closestPoint( from, closest, tmp, tmp );
  Vec3 ttt = from - closest;
  HAPIFloat aba = ttt * ttt;
  if( aba <= radius * radius ) return true; 

  Vec3 n = normal;

  Vec3 t = from - a;
  if( t * n < 0 )
    n = -n;

  n.normalizeSafe();

  Plane plane( a, n );
  IntersectionInfo info;
  Vec3 D = from - radius * n;
  Vec3 v = to - from;


  // if res is false the plane is parallell to the line and
  // then we could have a hit, this can give problems.
  bool res = plane.lineIntersect( D, D + v, info );
  Vec3 P = info.point;

  // p 204 to test triangle
  if( res ) {
    bool inside = true;
    Vec3 a0 = a - P;
    Vec3 b0 = b - P;
    Vec3 c0 = c - P;
    HAPIFloat _ab = a0 * b0;
    HAPIFloat _ac = a0 * c0;
    HAPIFloat bc = b0 * c0;
    // Make sure plane normals for pab and pbc point in the same direction
    if( bc * _ac - (c0 * c0) * _ab < 0.0f ) inside = false;
    else if(_ab * bc - _ac * (b0 * b0) ) inside = false;
    
    if( inside )
      return true;
  }

  HAPIFloat cyl_t;
  if( Collision::intersectSegmentCylinder( from, to, a, b, radius, cyl_t ) ) {
    return true;
  }

  if( Collision::intersectSegmentCylinder( from, to, a, c, radius, cyl_t ) ) {
    return true;
  }

  if( Collision::intersectSegmentCylinder( from, to, b, c, radius, cyl_t ) ) {
    return true;
  }

  Sphere sphere( a, radius );
  if( sphere.lineIntersect( from, to, info ) ) {
    return true;
  }

  sphere.center = b;
  if( sphere.lineIntersect( from, to, info ) ) {
    return true;
  }

  sphere.center = c;
  if( sphere.lineIntersect( from, to, info ) ) {
    return true;
  }

  return false;
}

// at the moment this function does not return a IntersectionInfo
bool Triangle::movingSphereIntersectRobust( HAPIFloat radius,
                                            const Vec3 &from, 
                                            const Vec3 &to ) {
  return movingSphereIntersect( radius, from,to );
}

bool Triangle::movingSphereIntersect( HAPIFloat radius,
                                      const Vec3 &from, 
                                      const Vec3 &to,
                                      IntersectionInfo &result ) {
  // Using robuster version of movingSphereIntersect with triangle.
  Vec3 closest, tmp;
  closestPoint( from, closest, tmp, tmp );
  Vec3 ttt = from - closest;
  HAPIFloat aba = ttt * ttt;
  if( aba <= radius * radius ) {
    result.intersection = true;
    result.point = closest;
    result.normal = ttt;
    result.normal.normalizeSafe();
    result.t = 0;
    result.primitive = this;
    return true; 
  }
                     
  Vec3 n = normal;

  Vec3 t = from - closest;
  if( t * n < 0 )
    n = -n;

  Plane plane( closest, n );
  IntersectionInfo info;
  Vec3 v = to - from;

  Vec3 D = from - radius * n;

  bool res = plane.lineIntersect( D, D + v, info );
  Vec3 P = info.point;

  if( res ) {
    bool inside = true;
    Vec3 a0 = a - P;
    Vec3 b0 = b - P;
    Vec3 c0 = c - P;
    HAPIFloat _ab = a0 * b0;
    HAPIFloat _ac = a0 * c0;
    HAPIFloat bc = b0 * c0;
    // Make sure plane normals for pab and pbc point in the same direction
    if( bc * _ac - (c0 * c0) * _ab < 0.0f ) inside = false;
    else if(_ab * bc - _ac * (b0 * b0) < 0.0f ) inside = false;

    if( inside ) {
      result.point = P;
      result.intersection = true;
      result.primitive = this;
      result.t = info.t;
      result.normal = ( from + v * result.t ) - result.point;
      result.normal.normalize();
      return true;
    }
  }

  HAPIFloat closest_t = 5;
  LineSegment cyl_line( a, c );
  if( Collision::intersectSegmentCylinder( from, to,
                                           cyl_line.start,
                                           cyl_line.end,
                                           radius,
                                           aba ) ) {
    if( aba < closest_t ) closest_t = aba;
  }

  if( Collision::intersectSegmentCylinder( from, to, b, c, radius, aba ) ) {
    if( aba < closest_t ) {
      closest_t = aba;
      cyl_line.start = b;
    }
  }

  if( Collision::intersectSegmentCylinder( from, to, a, b, radius, aba ) ) {
    if( aba < closest_t ) {
      closest_t = aba;
      cyl_line.start = a;
      cyl_line.end = b;
    }
  }

  if( closest_t < 4 ) {
    result.primitive = this;
    Vec3 sphere_cent = from + v * closest_t;
    cyl_line.closestPoint( sphere_cent, result.point, tmp, tmp );
    result.normal = sphere_cent - result.point;
    result.normal.normalize();
    result.intersection = true;
    result.t = closest_t;
    return true;
  }

  Sphere sphere( a, radius );
  sphere.center = a;
  closest_t = 5;
  if( sphere.lineIntersect( from, to, info ) ) {
    if( info.t < closest_t ) {
      closest_t = info.t;
      P = sphere.center;
    }
  }

  sphere.center = b;
  if( sphere.lineIntersect( from, to, info ) ) {
    if( info.t < closest_t ) {
      closest_t = info.t;
      P = sphere.center;
    }
  }

  sphere.center = c;
  if( sphere.lineIntersect( from, to, info ) ) {
    if( info.t < closest_t ) {
      closest_t = info.t;
      P = sphere.center;
    }
  }

  if( closest_t < 4 ) {
    result.point = P;
    result.primitive = this;
    result.normal = ( from + v * closest_t ) - result.point;
    result.normal.normalize();
    result.intersection = true;
    result.t = closest_t;
    return true;
  }
  return false;
}

bool BinaryBoundTree::lineIntersect( const Vec3 &from, 
                                     const Vec3 &to,
                                     IntersectionInfo &result,
                                     FaceType face ) {
  if ( isLeaf() )  {
    IntersectionInfo tempResult;
    for( unsigned int i = 0; i < triangles.size(); ++i ) {
      Triangle &t = triangles[i];
      if( t.lineIntersect( from, to, tempResult, face ) ) {
        if( result.intersection) {
          if( (tempResult.point - from).lengthSqr() < 
              (result.point - from).lengthSqr() )
            result = tempResult;
        }
        else
          result = tempResult;
      }
    }

    for( unsigned int i = 0; i < linesegments.size(); ++i ) {
      LineSegment &ls = linesegments[i];
      if( ls.lineIntersect( from, to, tempResult, face ) ) {
        if( result.intersection) {
          if( (tempResult.point - from).lengthSqr() < 
              (result.point - from).lengthSqr() )
            result = tempResult;
        }
        else
          result = tempResult;
      }
    }

    for( unsigned int i = 0; i < points.size(); ++i ) {
      Point &p = points[i];
      if( p.lineIntersect( from, to, tempResult, face ) ) {
        if( result.intersection) {
          if( (tempResult.point - from).lengthSqr() < 
              (result.point - from).lengthSqr() )
            result = tempResult;
        }
        else
          result = tempResult;
      }
    }
    return result.intersection;
    //return false;
  }  else   {
    if ( bound->boundIntersect( from, to ) )  {
      bool overlap = false;
      
      if (left.get()) overlap |= left->lineIntersect( from, to, result, face );
      if (right.get()) overlap |= right->lineIntersect( from, to, result, face );
      
      return overlap;
    }
    else return false;
  }
}

bool BinaryBoundTree::movingSphereIntersect( HAPIFloat radius,
                                             const Vec3 &from, 
                                             const Vec3 &to ) {
  if( isLeaf() ) {
    for( unsigned int i = 0; i < triangles.size(); ++i ) {
      Triangle &t = triangles[i];
      if( t.movingSphereIntersect( radius, from, to ) ) return true;
    }

    for( unsigned int i = 0; i < linesegments.size(); ++i ) {
      LineSegment &ls = linesegments[i];
      if( ls.movingSphereIntersect( radius, from, to ) ) return true;
    }

    for( unsigned int i = 0; i < points.size(); ++i ) {
      Point &pt = points[i];
      if( pt.movingSphereIntersect( radius, from, to ) ) return true;
    }
  } else {
    if( bound->boundMovingSphereIntersect( radius, from, to ) ) {
      if( left.get() && left->movingSphereIntersect( radius, from, to ) )
        return true;

      if( right.get() && right->movingSphereIntersect( radius, from, to ) )
        return true;
    }
  }
  return false;
}

bool BinaryBoundTree::movingSphereIntersect( HAPIFloat radius,
                                             const Vec3 &from, 
                                             const Vec3 &to,
                                             IntersectionInfo &result ) {
  if ( isLeaf() )  {
    IntersectionInfo temp_result;
    for( unsigned int i = 0; i < triangles.size(); ++i ) {
      Triangle &t = triangles[i];
      if( t.movingSphereIntersect( radius, from, to, temp_result ) ) {
        if( temp_result.intersection ) {
          if( result.intersection ) {
            if( temp_result.t < result.t )
              result = temp_result;
          } else {
            result = temp_result;
          }
        }
      }
    }

    return result.intersection;
  }  else   {
    if ( bound->boundMovingSphereIntersect( radius, from, to ) )  {
      bool overlap = result.intersection;
      
      if (left.get()) {
        overlap = left->movingSphereIntersect( radius, from, to, result );
      }
      if (right.get()) 
        overlap = overlap || right->movingSphereIntersect( radius, from, to, result );
      
      return overlap;
    }
    else return false;
  }
}

Matrix3 covarianceMatrix( const vector< Vec3 >&points ) {

  size_t nr_points = points.size();

  Vec3 c = points[0];
  for(unsigned int i=1 ; i < nr_points; ++i) 
    c+= points[i];

  c = c / points.size();
  
  HAPIFloat e00 = 0, e01 = 0, e02 = 0,
                    e11 = 0, e12 = 0,
                             e22 = 0;
  for (unsigned int i = 0; i < nr_points; ++i)  {
    Vec3 p = points[i] - c;
    
    e00 += p.x * p.x;
    e11 += p.y * p.y;
    e22 += p.z * p.z;
    e01 += p.x * p.y;
    e02 += p.x * p.z;
    e12 += p.y * p.z;
  }
  
  Matrix3 cov;
  cov[0][0] = e00 / nr_points;
  cov[1][1] = e11 / nr_points;
  cov[2][2] = e22 / nr_points;
  cov[0][1] = cov[1][0] = e01 / nr_points;
  cov[0][2] = cov[2][0] = e02 / nr_points;
  cov[1][2] = cov[2][1] = e12 / nr_points;
  
  return cov;
}

void symmetricSchurDecomposition( const Matrix3 &A, 
                                  unsigned int p, 
                                  unsigned int q, 
                                  HAPIFloat &c, 
                                  HAPIFloat &s) {
  if (H3DUtil::H3DAbs(A[p][q]) > 0.0001f) {
    HAPIFloat r = (A[q][q] - A[p][p]) / (2.0f * A[p][q]);
    HAPIFloat t = (r >= 0.0f ? 
                  1.0f / (r + H3DUtil::H3DSqrt(1.0f + r*r)) : 
                  -1.0f / (-r + H3DUtil::H3DSqrt(1.0f + r*r)) );
    
    c = 1.0f / H3DUtil::H3DSqrt(1.0f + t*t);
    s = t * c;
  }  else{
    c = 1.0f;
    s = 0.0f;
  }
}

/// Tries to find eigen values and eigen vectors for the matrix A given as
/// input.
/// \param A The matrix to find eigen values and eigen vectors for.
/// \param eigValues Will contain the calculated eigen values.
/// \param eigVectors Will contain the calculated eigen vectors.
/// \param maxIterations Maximum number of iterations before stop trying
/// to calculate a result.
void eigenValuesAndEigenVectorsJacobi(const Matrix3& A, 
                                      Vec3& eigValues, 
                                      Matrix3& eigVectors, 
                                      const unsigned int maxIterations = 50 ) {
  //!NOTE: on return rows (not cols) of eigVectors contains eigenvalues

  Matrix3 a = A;
  Matrix3 v;
  
  unsigned int i, j, n, p, q;
  HAPIFloat prevoff, c, s;
  Matrix3 J, b, t;

  // Repeat for some maximum number of iterations
  prevoff = 0;
  for (n = 0; n < maxIterations; ++n)  {
    p = 0; q = 1;
    for ( i = 0; i < 3; ++i) for (j = 0; j < 3; ++j ) {
      if (i == j) continue;
      if (H3DUtil::H3DAbs(a[i][j]) > H3DUtil::H3DAbs(a[p][q])) {p = i; q = j;}
    }

    symmetricSchurDecomposition(a, p, q, c, s);
    for (i = 0; i < 3; ++i) {
      J[i][0] = J[i][1] = J[i][2] = 0.0f;
      J[i][i] = 1.0f;
    }
    J[p][p] =  c; J[p][q] = s;
    J[q][p] = -s; J[q][q] = c;

    // cumulate rotations
    v = v * J;

    // diagonalize
    a = (J.transpose() * a) * J;
    
    //  eval norm
    HAPIFloat off = 0.0;
    for (i = 0; i < 3; ++i) for (j = 0; j < 3; ++j)  {
      if (i == j) continue;
      off += a[i][j] * a[i][j];
    }
    
    // stop condition
    if (n > 2 && off >= prevoff) break;
    prevoff = off;
  }
  
  eigValues = Vec3(a[0][0],a[1][1],a[2][2]);
  eigVectors = v.transpose();
}

void OrientedBoxBound::fitAroundPoints( const vector< Vec3 > &points ) {
  Matrix3 covariance = covarianceMatrix( points );
  
  Vec3 eig_values;
  Matrix3 eig_vectors;

  eigenValuesAndEigenVectorsJacobi(covariance, eig_values, eig_vectors);
  
  
  //  guess an oriented reference frame
  unsigned int length_order[3];
  //  initialize
  for(unsigned int i=0 ; i<3 ; ++i) length_order[i] = i;
  
  //  do a bubble sort on indices
  for( unsigned int i=0 ; i<2 ; ++i) {
    for(unsigned int j=i+1 ; j<3 ; ++j)  {
      if ( eig_values[length_order[i]] < eig_values[length_order[j]] ) {
        // swap
        unsigned int temp( length_order[i] );
        length_order[i] = length_order[j];
        length_order[j] = temp;
      }
    }
  }

  Vec3 principal( eig_vectors[length_order[0]][0],
                   eig_vectors[length_order[0]][1],
                   eig_vectors[length_order[0]][2] );

  Vec3 secondary( eig_vectors[length_order[1]][0],
                   eig_vectors[length_order[1]][1],
                   eig_vectors[length_order[1]][2] );

  principal.normalize();
  secondary.normalize();
  Vec3 v = principal % secondary;

  Matrix3 frame( principal.x, principal.y, principal.z,
                  secondary.x, secondary.y, secondary.z, 
                  v.x, v.y, v.z );
    
  // build box in the new frame
  min = frame * points[0];
  max = min;
  for(unsigned int i=1 ; i<points.size() ; ++i)  {
    Vec3 pt = frame * points[i];
    
    if (pt.x < min.x) min.x = pt.x;
    if (pt.y < min.y) min.y = pt.y;
    if (pt.z < min.z) min.z = pt.z;
    
    if (pt.x > max.x) max.x = pt.x;
    if (pt.y > max.y) max.y = pt.y;
    if (pt.z > max.z) max.z = pt.z;
  }
  
  // fill oriented box
  orientation = Rotation( frame );
  
  //cerr << frame * (0.5f*(min+max)) << endl;
  center = frame.transpose() * (0.5f*(min+max)) ;
  halfsize = 0.5f*(max-min);
}

bool OrientedBoxBound::boundIntersect( const Vec3 &from, 
                                       const Vec3 &to ) {
  Vec3 rp = orientation * from;
  Vec3 rq = orientation * to;
  
  return AABoxBound::boundIntersect( rp, rq );
}

void OrientedBoxBound::closestPoint( const Vec3 &p,
                                     Vec3 &closest_point,
                                     Vec3 &normal,
                                     Vec3 &tex_coord ) {
  assert( false );
}

bool OrientedBoxBound::insideBound( const Vec3 &p ) {
  Vec3 rp = orientation * p; 
  return ( rp.x <= max.x &&
           rp.x >= min.x &&
           rp.y <= max.y &&
           rp.y >= min.y &&
           rp.z <= max.z &&
           rp.z >= min.z );
}

void OrientedBoxBound::render() {
  Vec3 _center = -orientation * ((max + min)/2); // Why not use already calculated center?
#ifdef HAVE_OPENGL
  glMatrixMode( GL_MODELVIEW );
  glPushMatrix();
  glDisable( GL_LIGHTING );
  glColor3d( 1, 1, 0 );
  glTranslated(_center.x, _center.y, _center.z );
  glRotated( -orientation.angle *180 / H3DUtil::Constants::pi, 
             orientation.axis.x, orientation.axis.y, orientation.axis.z );
  Vec3 minf( - halfsize );
  Vec3 maxf( halfsize );
  glBegin( GL_LINE_STRIP );
  glVertex3d( minf.x, minf.y, minf.z );
  glVertex3d( minf.x, maxf.y, minf.z );
  glVertex3d( maxf.x, maxf.y, minf.z );
  glVertex3d( maxf.x, minf.y, minf.z );
  glVertex3d( minf.x, minf.y, minf.z );
  glEnd();

  glBegin( GL_LINE_STRIP );
  glVertex3d( minf.x, minf.y, maxf.z );
  glVertex3d( minf.x, maxf.y, maxf.z );
  glVertex3d( maxf.x, maxf.y, maxf.z );
  glVertex3d( maxf.x, minf.y, maxf.z );
  glVertex3d( minf.x, minf.y, maxf.z );
  glEnd();

  glBegin( GL_LINES );
  glVertex3d( minf.x, minf.y, maxf.z );
  glVertex3d( minf.x, minf.y, minf.z );
  glVertex3d( maxf.x, minf.y, maxf.z );
  glVertex3d( maxf.x, minf.y, minf.z );
  glVertex3d( minf.x, maxf.y, maxf.z );
  glVertex3d( minf.x, maxf.y, minf.z );
  glVertex3d( maxf.x, maxf.y, maxf.z );
  glVertex3d( maxf.x, maxf.y, minf.z );
  glEnd();
  glEnable( GL_LIGHTING );
  glPopMatrix();
#endif
}

void GeometryPrimitive::getConstraints( const Vec3 &point,
                                        Constraints &constraints,
                                        FaceType face,
                                        HAPIFloat radius ) {
  Vec3 closest_point, cp_normal, cp_tex_coord;
  closestPoint( point, closest_point, cp_normal, cp_tex_coord );
  //cerr << closest_point << endl;
  Vec3 normal = point - closest_point;

  if( face == Collision::FRONT ) {
    if( normal * cp_normal < 0 ) return;
  } else if( face == Collision::BACK ) {
    if( normal * cp_normal > 0 ) return;
  }
  HAPIFloat l2 = normal.x*normal.x+normal.y*normal.y+normal.z*normal.z;
  // if the length already is 1 we don't have to do anything
  if( H3DUtil::H3DAbs(l2-1) > Constants::epsilon ) {
    HAPIFloat l = H3DUtil::H3DSqrt( l2 );
    if( H3DUtil::H3DAbs(l) >= Constants::epsilon ) {
      normal.x /= l; 
      normal.y /= l;
      normal.z /= l;
    } else {
      // Choose primitive normal instead.
      normal = cp_normal;
    }
  }
  //cerr << closest_point << endl;
   Vec3 v = closest_point - point;
   if( radius < 0 || v * v <= radius * radius )
     constraints.push_back( PlaneConstraint( closest_point, normal, 
                                             cp_tex_coord, NULL, this ) );
}


void BinaryBoundTree::getConstraints( const Vec3 &point,
                                      Constraints &constraints,
                                      FaceType face,
                                      HAPIFloat radius ) {
  if ( isLeaf() )  {
    for( unsigned int i = 0; i < triangles.size(); ++i ) {
      Triangle &t = triangles[i];
      t.getConstraints( point, constraints, face, radius );
    }
    for( unsigned int i = 0; i < linesegments.size(); ++i ) {
      LineSegment &ls = linesegments[i];
      ls.getConstraints( point, constraints, face, radius );
    }
    for( unsigned int i = 0; i < points.size(); ++i ) {
      Point &pt = points[i];
      pt.getConstraints( point, constraints, face, radius );
    }
  }  else   {
    //if ( bound->boundIntersect( from, to ) )  {
    if (left.get()) left->getConstraints( point, constraints, face, radius );
    if (right.get()) right->getConstraints( point, constraints, face, radius );
    
  }
}

void BinaryBoundTree::getTrianglesWithinRadius( const Vec3 &p,
                                                HAPIFloat radius,
                                                vector< Triangle > &result ) {
  HAPIFloat r2 = radius * radius;
  if ( isLeaf() )  {
    for( vector< Triangle >::iterator i = triangles.begin();
         i != triangles.end(); ++i ) {
      Vec3 cp, tmp;
      (*i).closestPoint( p, cp, tmp, tmp );
      if( (cp - p).lengthSqr() <= r2 )
        result.push_back( *i );
    }
    //cerr << "!: " << triangles.size() << endl;
           //result.insert( result.end(), triangles.begin(), triangles.end() );
  }  else   {
    Vec3 cp = bound->boundClosestPoint( p );
    if( (cp - p).lengthSqr() > r2 && !bound->insideBound(p) ) return;

    if (left.get()) left->getTrianglesWithinRadius( p, radius, result );
    if (right.get()) right->getTrianglesWithinRadius( p, radius, result );
  }
}

void BinaryBoundTree::getPrimitivesWithinRadius( const Vec3 &p,
                                                HAPIFloat radius,
                                                vector< Triangle > &tris,
                                                vector< LineSegment > &lines,
                                                vector< Point > &pts ) {
  HAPIFloat r2 = radius * radius;
  if ( isLeaf() )  {
    for( vector< Triangle >::iterator i = triangles.begin();
         i != triangles.end(); ++i ) {
      Vec3 cp, tmp;
      (*i).closestPoint( p, cp, tmp, tmp );
      if( (cp - p).lengthSqr() <= r2 )
        tris.push_back( *i );
    }

    for( vector< LineSegment >::iterator i = linesegments.begin();
         i != linesegments.end(); ++i ) {
      Vec3 cp, tmp;
      (*i).closestPoint( p, cp, tmp, tmp );
      if( (cp - p).lengthSqr() <= r2 )
        lines.push_back( *i );
    }

    for( vector< Point >::iterator i = points.begin();
         i != points.end(); ++i ) {
      Vec3 cp, tmp;
      (*i).closestPoint( p, cp, tmp, tmp );
      if( (cp - p).lengthSqr() <= r2 )
        pts.push_back( *i );
    }
    //cerr << "!: " << triangles.size() << endl;
           //result.insert( result.end(), triangles.begin(), triangles.end() );
  }  else   {
    Vec3 cp = bound->boundClosestPoint( p );
    if( (cp - p).lengthSqr() > r2 && !bound->insideBound(p) ) return;

    if (left.get()) left->getPrimitivesWithinRadius( p, radius, tris, lines, pts );
    if (right.get()) right->getPrimitivesWithinRadius( p, radius, tris, lines, pts );
  }
}

void LineSegment::render() {
#ifdef HAVE_OPENGL
  glDisable( GL_LIGHTING );
  glBegin( GL_LINES );
  glVertex3d( start.x, start.y, start.z );
  glVertex3d( end.x, end.y, end.z );
  glEnd();
  glEnable( GL_LIGHTING );
#endif
}

/// Returns the closest point on the object to the given point p.
void LineSegment::closestPoint( const Vec3 &p,
                                Vec3 &closest_point,
                                Vec3 &normal,
                                Vec3 &tex_coord ) {
  Vec3 ab = end - start;
  HAPIFloat ab2 = ab * ab;
  if( ab2 < Constants::epsilon ) {
    closest_point = start;
  } else {
    HAPIFloat t = 
      (p - start).dotProduct( ab ) / ab2;
    if( t < 0 ) t = 0;
    if( t > 1 ) t = 1;
    closest_point = start + t * ab;
  }
  normal = p - closest_point;
  normal.normalizeSafe();
  tex_coord = Vec3();
}

/// Detect collision between a line segment and the object.
bool LineSegment::lineIntersect( const Vec3 &from, 
                                 const Vec3 &to,
                                 IntersectionInfo &result,
                                 FaceType face ) {
  HAPIFloat s,t;
  Vec3 c0, c1;
  closestPointOnLine( from, to, s, t, c0, c1 );
  if( (c0-c1).lengthSqr() < Constants::epsilon ) {
    result.intersection = true;
    result.normal = ( to - from ).crossProduct( end - start );
    result.normal.normalizeSafe();
    result.point = c0;
    result.primitive = this;
    result.t = t;
    result.tex_coord = Vec3();
    result.face = Collision::FRONT;
    return true;
  }
  return false;
}

// Detect collision between a moving sphere and the object.
bool LineSegment::movingSphereIntersect( HAPIFloat radius,
                                         const Vec3 &from, 
                                         const Vec3 &to ) {
  Vec3 tmp;
  HAPIFloat t;
  HAPIFloat r2 = radius * radius;
  HAPIFloat d2 = 
    LineSegment( from, to ).closestPointOnLine( start, end, 
                                                t, t, 
                                                tmp, tmp );
  return d2 <= r2;
}

HAPIFloat LineSegment::closestPointOnLine( const Vec3 &from, const Vec3 &to,
                                           HAPIFloat &s, HAPIFloat &t,
                                           Vec3 &c1, Vec3 &c2 ) {
  Vec3 p1 = from;
  Vec3 q1 = to;
  Vec3 p2 = start;
  Vec3 q2 = end;

  Vec3 d1 = q1 - p1; // Direction vector of segment S1
  Vec3 d2 = q2 - p2; // Direction vector of segment S2
  Vec3 r = p1 - p2;

  // Squared length of segment S1, always nonnegative
  HAPIFloat a = d1 * d1; 
  // Squared length of segment S2, always nonnegative
  HAPIFloat e = d2 * d2; 
  HAPIFloat f = d2 * r;

  // Check if either or both segments degenerate into points
  if (a <= Constants::epsilon && e <= Constants::epsilon) {
    // Both segments degenerate into points
    s = t = 0.0f;
    c1 = p1;
    c2 = p2;
    Vec3 v = c1 - c2;
    return v * v;
  }
  if (a <= Constants::epsilon) {
    // First segment degenerates into a point
    s = 0.0f;
    t = f / e; // s = 0 => t = (b*s + f) / e = f / e
    t = clamp(t, 0.0f, 1.0f);
  } else {
    HAPIFloat c = d1 *r;
    if (e <= Constants::epsilon) {
      // Second segment degenerates into a point
      t = 0.0f;
      s = clamp(-c / a, 0.0f, 1.0f); // t = 0 => s = (b*t - c) / a = -c / a
    } else {
      // The general nondegenerate case starts here
      HAPIFloat b = d1 * d2;
      HAPIFloat denom = a*e-b*b; // Always nonnegative

      // If segments not parallel, compute closest point on L1 to L2, and
      // clamp to segment S1. Else pick arbitrary s (here 0)
      if (denom != 0.0f) {
        s = clamp((b*f - c*e) / denom, 0.0f, 1.0f);
      } else s = 0.0f;

      // Compute point on L2 closest to S1(s) using
      // t = Dot((P1+D1*s)-P2,D2) / Dot(D2,D2) = (b*s + f) / e
      t = (b*s + f) / e;

      // If t in [0,1] done. Else clamp t, recompute s for the new value
      // of t using s = Dot((P2+D2*t)-P1,D1) / Dot(D1,D1)= (t*b - c) / a
      // and clamp s to [0, 1]
      if (t < 0.0f) {
        t = 0.0f;
        s = clamp(-c / a, 0.0f, 1.0f);
      } else if (t > 1.0f) {
        t = 1.0f;
        s = clamp((b - c) / a, 0.0f, 1.0f);
      }
    }
  }

  c1 = p1 + d1 * s;
  c2 = p2 + d2 * t;
  Vec3 v = c1 - c2;
  return v * v;
}
  
void Collision::Point::render() {
#ifdef HAVE_OPENGL
  glDisable( GL_LIGHTING );
  glBegin( GL_POINTS );
  glVertex3d( position.x, position.y, position.z );
  glEnd();
  glEnable( GL_LIGHTING );
#endif
}

/// Detect collision between a line segment and the object.
bool Collision::Point::lineIntersect( const Vec3 &from, 
                                 const Vec3 &to,
                                 IntersectionInfo &result,
                                  FaceType face ) {
  Vec3 ab = to - from;
  // Project c onto ab, but deferring divide by Dot( ab, ab )
  HAPIFloat t = ( position - from ) * ab;
  if( t <= 0.0f ) {
    // c project outside the [from, to] interval, on from side.
    return false;
  }
  else {
    HAPIFloat denom = ab * ab;
    if( t >= denom ) {
      // c projects outside the [from, to] interval, on to side.
      return false;
    }
    else {
      // c projects inside the [from, to] interval
      t = t / denom;
      Vec3 d = from + t * ab;
      if( (position - d).lengthSqr() ) {
        result.point = position;
        result.primitive = this;
        result.t = t;
        return true;
      }
    }
  }
  return false;
}
      
// Detect collision between a moving sphere and the object.
bool Collision::Point::movingSphereIntersect( HAPIFloat radius,
                                   const Vec3 &from, 
                                   const Vec3 &to ) {
  HAPIFloat r2 = radius * radius;
  Vec3 closest_point, tmp; 
  LineSegment( from, to ).closestPoint( position, closest_point, 
                                        tmp, tmp );
  Vec3 v = position - closest_point;
  return v * v <= r2;
}


bool Plane::lineIntersect( const Vec3 &from, 
                           const Vec3 &to,
                           IntersectionInfo &result,
                           FaceType face ) {
  result.intersection = false;
  Vec3 from_to = to - from;

  HAPIFloat denom = normal * from_to;

  if( denom > Constants::epsilon ) { 
    // line pointing away from normal
    if( face == Collision::FRONT ) return false;
    result.face = Collision::BACK;
  } else {
    if( face == Collision::BACK ) return false;
    result.face = Collision::FRONT;
  }
   
  if( denom * denom < Constants::epsilon ) {
    return false;
  }
  
  HAPIFloat u = ( normal * ( point - from ) ) / denom; 
  if( u <= 1 + Constants::epsilon && u >= 0 - Constants::epsilon ) {
    
    if( u < 0 ) u = 0;
    if( u > 1 ) u = 1;
    result.point = from + u * from_to;
    result.normal = normal;
    result.primitive = this;
    result.t = u;
    result.intersection = true;
    return true;
  }
  return false;
}

bool Plane::movingSphereIntersect( HAPIFloat radius,
                                   const Vec3 &from, 
                                   const Vec3 &to ) {
  HAPIFloat d = normal * point;

  // Code from p222 in Realtime Collision detection book

  // Get the distance for both a and b from plane p
  HAPIFloat adist = from * normal - d;
  HAPIFloat bdist = to * normal - d;
  // Intersects if on different sides of plane (distances have different signs)
  if ( adist * bdist < 0.0f ) return true;
  // Intersects if start or end position within radius from plane
  if ( H3DUtil::H3DAbs(adist) <= radius || 
       H3DUtil::H3DAbs( bdist ) <= radius ) return true;
  // No intersection
  return false;
}

void Plane::render() {
  // This rendering will of course not work in all cases. How do you actually
  // render a plane? It needs to be infitely big to be accurate.
  // max_length * 2 is approximately the length between triangle edges.
  if( normal.lengthSqr() > Constants::epsilon ) {
    Vec3 rand_point( rand(), rand(), rand() );
    Vec3 point_in_plane, dummy;
    closestPoint( rand_point, point_in_plane, dummy, dummy );
    while( (point - point_in_plane ).lengthSqr() <= Constants::epsilon ) {
      rand_point = Vec3( rand(), rand(), rand() );
      closestPoint( rand_point, point_in_plane, dummy, dummy );
    }

    Vec3 t1 = (point_in_plane - point );
    t1.normalize();
    Vec3 t2 = normal % t1;
    t2.normalize();

    HAPIFloat max_length = 1e10;
    t1 = t1 * max_length;
    t2 = t2 * max_length;

#ifdef HAVE_OPENGL
    glBegin( GL_TRIANGLES );
    glNormal3d( normal.x, normal.y, normal.z );
    glVertex3d( t2.x, t2.y, t2.z );
    glVertex3d( -t2.x - t1.x, -t2.y - t1.y, -t2.z - t1.z );
    glVertex3d( -t2.x + t1.x, -t2.y + t1.y, -t2.z + t1.z );
    glEnd();
#endif
  }
}

Sphere::Sphere( const Vec3 &_center, HAPIFloat _radius ):
  center( _center ), radius( _radius ) {
#ifdef HAVE_OPENGL
  CollisionInternals::increaseSphereCounter();
#endif
}

Sphere::~Sphere() {
#ifdef HAVE_OPENGL
  CollisionInternals::decreseSphereCounter();
#endif
}

bool Sphere::lineIntersect( const Vec3 &from, 
                            const Vec3 &to,
                            IntersectionInfo &result,
                            FaceType face ) {
  Vec3 start_point = from - center;
  Vec3 end_point = to - center;
 
  // p is the starting point of the ray used for determinining intersection
  // v is the vector between this starting point and the end point.
  // the starting point must be outside the sphere.
  Vec3 p, v;
  HAPIFloat r2 = radius * radius;

  HAPIFloat a0  = start_point * start_point - r2;
  if (a0 <= 0) {
    // start_point is inside sphere
    a0 = end_point * end_point - r2;
    if( a0 <= 0 || face == Collision::FRONT ) {
      // both start_point and end_point are inside sphere so no intersection
      return false;
    } else {
      // start_point is inside and end_point is outside. We will have an 
      // intersection.
      p = end_point;
      v = start_point - end_point;
      result.face = Collision::BACK;
    }
  } else {
    // start_point is outside sphere
    if( face == Collision::BACK ) return false;

    p = start_point;
    v = end_point - start_point;
    
    // check that the line will intersect 
    HAPIFloat a1 = v * p;
    if (a1 >= 0) {
      // v is pointing away from the sphere so no intersection
      return false;
    }
    result.face = Collision::FRONT;
  }
  // use implicit quadratic formula to find the roots
  HAPIFloat a = v.x*v.x + v.y*v.y + v.z*v.z;
  HAPIFloat b = 2 * (p.x*v.x + p.y*v.y + p.z*v.z);
  HAPIFloat c = p.x*p.x + p.y*p.y + p.z*p.z - r2;

  HAPIFloat s = b*b - 4*a*c;

  HAPIFloat u;
  if( s == 0.0 ) {
    // line is a tangent to the sphere
    u = -b/(2*a);
  } else if( s > 0.0 ) {
    // line intersects sphere in two points
    HAPIFloat u0 = (-b + sqrt(s))/(2*a);
    HAPIFloat u1 = (-b - sqrt(s))/(2*a);
    u = u0 < u1 ? u0 : u1;
  }  else {
    // line does not intersect
    return false;
  }

  // check that the intersection point is within the line segment
  if( u > 1.0 || u < 0.0) {
    return false;
  }
  
  // calculate the intersection point and normal
  result.normal =  p + u*v;
  result.point = result.normal + center;
  result.t = u;
  // NOTE: Texturing done as per X3D standard although HAPI has nothing to do
  // with X3D.
  HAPIFloat phi = H3DUtil::H3DAcos( result.normal.y / radius );
  HAPIFloat theta = H3DUtil::H3DAtan2( -result.normal.x, -result.normal.z );
  if( theta < 0 )
    theta = 2 * H3DUtil::Constants::pi + theta;
  result.tex_coord = Vec3( theta / (2 * H3DUtil::Constants::pi ),
                           1 - phi / H3DUtil::Constants::pi,
                           0 );
  result.normal.normalize();
  result.primitive = this;
  return true;
}

void Sphere::closestPoint( const Vec3 &p,
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

  // NOTE: Texturing done as per X3D standard for the Sphere node.
  HAPIFloat phi = H3DUtil::H3DAcos( closest_point.y / radius );
  HAPIFloat theta = H3DUtil::H3DAtan2( -closest_point.x, -closest_point.z );
  if( theta < 0 )
    theta = 2 * H3DUtil::Constants::pi + theta;
  tex_coord = Vec3( theta / (2 * H3DUtil::Constants::pi ),
                    1 - phi / H3DUtil::Constants::pi,
                    0 );
}

bool Sphere::movingSphereIntersect( HAPIFloat r,
                                    const Vec3 &from, 
                                    const Vec3 &to ) {
  // Code from p224 in Realtime Collision detection book

  // Vector between sphere centers
  Vec3 s = from - center;    
  // Motion of sphere
  Vec3 v = to - from;         

  // Sum of sphere radii
  HAPIFloat radius_sum = radius + r;
  
  HAPIFloat c = s * s - radius_sum * radius_sum;
  if (c < 0.0f) {
    // Spheres initially overlapping so exit directly
    return true;
  }

  HAPIFloat a = v * v;
  // Spheres not moving relative each other
  if (a < Constants::epsilon) return false; 
  HAPIFloat b = v * s;
  // Spheres not moving towards each other
  if (b >= 0.0f) return false; 
  HAPIFloat d = b * b - a * c;
  // No real-valued root, spheres do not intersect
  if (d < 0.0f) return false;  

  HAPIFloat t = ( -b - H3DUtil::H3DSqrt( d ) ) / a;
  if( t < 0.0f || t > 1.0f ) return false;

  return true;
}

bool Sphere::movingSphereIntersect( HAPIFloat r,
                                    const Vec3 &from, 
                                    const Vec3 &to,
                                    IntersectionInfo &result ) {
  // Code from p224 in Realtime Collision detection book

  // Vector between sphere centers
  Vec3 s = from - center;    
  // Motion of sphere
  Vec3 v = to - from;         

  // Sum of sphere radii
  HAPIFloat radius_sum = radius + r;
  
  HAPIFloat c = s * s - radius_sum * radius_sum;
  if (c < 0.0f) {
    // Spheres initially overlapping so exit directly
    // This is only correct if we are only interested in not detecting
    // when a sphere moves through another sphere from the inside. Might give
    // problems.
    result.intersection = true;
    result.normal = s;
    result.normal.normalizeSafe();
    result.primitive = this;
    result.point = result.normal * radius;
    result.t = 0.0f;
    return true;
  }

  HAPIFloat a = v * v;
  // Spheres not moving relative each other
  if (a < Constants::epsilon) return false; 
  HAPIFloat b = v * s;
  // Spheres not moving towards each other
  if (b >= 0.0f) return false; 
  HAPIFloat d = b * b - a * c;
  // No real-valued root, spheres do not intersect
  if (d < 0.0f) return false;  

  HAPIFloat t = ( -b - H3DUtil::H3DSqrt( d ) ) / a;
  if( t < 0.0f || t > 1.0f ) return false;

  result.t = t;
  Vec3 moving_center = from + v * t;
  // normal is the direction from the stationary sphere towards where the
  // moving sphere will be at time for intersection.
  result.normal = ( moving_center - center ) / radius_sum;
  result.point = result.normal * radius;
  result.intersection = true;
  result.primitive = this;

  return true;
}

void Sphere::render() {
#ifdef HAVE_OPENGL
  CollisionInternals::createSphereDisplayList();
  glMatrixMode( GL_MODELVIEW );
  glPushMatrix();
  glTranslated( center.x, center.y, center.z );
  glScaled( radius, radius, radius );
  glCallList( CollisionInternals::sphere_display_list_id );
  glPopMatrix();
#endif
}

void BinaryBoundTree::getTrianglesIntersectedByMovingSphere( HAPIFloat radius,
                                                             Vec3 from,
                                                             Vec3 to,
                                                             vector< Triangle > &result) {
  if ( isLeaf() )  {
    for( vector< Triangle >::iterator i = triangles.begin();
         i != triangles.end(); ++i ) {
      if( (*i).movingSphereIntersect( radius, from, to ) )
        result.push_back( *i );
    }
  }  else   {
    if(  !bound->boundMovingSphereIntersect( radius, from, to ) ) return;

    if (left.get()) left->getTrianglesIntersectedByMovingSphere( radius, from, to, result );
    if (right.get()) right->getTrianglesIntersectedByMovingSphere( radius, from, to, result );
  }
}

void BinaryBoundTree::getPrimitivesIntersectedByMovingSphere(
                HAPIFloat radius,
                Vec3 from,
                Vec3 to,
                vector< Triangle > &result_triangles,
                vector< LineSegment > &result_lines,
                vector< Point > &result_points ) {
  if ( isLeaf() )  {
    for( vector< Triangle >::iterator i = triangles.begin();
         i != triangles.end(); ++i ) {
      if( (*i).movingSphereIntersect( radius, from, to ) )
        result_triangles.push_back( *i );
    }

    for( vector< LineSegment >::iterator i = linesegments.begin();
         i != linesegments.end(); ++i ) {
      if( (*i).movingSphereIntersect( radius, from, to ) )
        result_lines.push_back( *i );
    }

    for( vector< Point >::iterator i = points.begin();
         i != points.end(); ++i ) {
      if( (*i).movingSphereIntersect( radius, from, to ) )
        result_points.push_back( *i );
    }
  }  else   {
    if(  !bound->boundMovingSphereIntersect( radius, from, to ) ) return;

    if (left.get()) left->getPrimitivesIntersectedByMovingSphere( radius,
      from, to, result_triangles, result_lines, result_points );
    if (right.get()) right->getPrimitivesIntersectedByMovingSphere( radius,
      from, to, result_triangles, result_lines, result_points );
  }
}

void BinaryBoundTree::closestPoint( const Vec3 &p,
                                    Vec3 &closest_point,
                                    Vec3 &closest_normal,
                                    Vec3 &closest_tex_coord ) {
  if ( isLeaf() )  {
    if( triangles.size() == 0 && linesegments.size() == 0 &&
        points.size() == 0 ) return;
    Vec3 cp, cn, tc;
    HAPIFloat d2 = std::numeric_limits<HAPIFloat>::max();
    for( vector< Triangle >::iterator i = triangles.begin();
         i != triangles.end(); ++i ) {
      Vec3 point, normal, tex_coord;
      (*i).closestPoint( p, point, normal, tex_coord );
      Vec3 v = p - point;
      HAPIFloat new_d2 = v * v;
      if( new_d2 < d2 ) {
        cp = point;
        cn = normal;
        d2 = new_d2;
        tc = tex_coord;
      }
    }

    for( vector< LineSegment >::iterator i = linesegments.begin();
         i != linesegments.end(); ++i ) {
      Vec3 point, normal, tex_coord;
      (*i).closestPoint( p, point, normal, tex_coord );
      Vec3 v = p - point;
      HAPIFloat new_d2 = v * v;
      if( new_d2 < d2 ) {
        cp = point;
        cn = normal;
        d2 = new_d2;
        tc = tex_coord;
      }
    }

    for( vector< Point >::iterator i = points.begin();
         i != points.end(); ++i ) {
      Vec3 point, normal, tex_coord;
      (*i).closestPoint( p, point, normal, tex_coord );
      Vec3 v = p - point;
      HAPIFloat new_d2 = v * v;
      if( new_d2 < d2 ) {
        cp = point;
        cn = normal;
        d2 = new_d2;
        tc = tex_coord;
      }
    }
    closest_point = cp;
    closest_normal = cn;
    closest_tex_coord = tc;
  } else {

    if( left.get() && right.get() ) {
      vector< BinaryBoundTree * > children(2);
      if( left->insideBound( p ) ) {
        children[0] = left.get();
        children[1] = right.get();
      } else {
        children[0] = right.get();
        children[1] = left.get();
      }

      children[0]->closestPoint( p, closest_point, closest_normal, closest_tex_coord );
      HAPI::Vec3 cv = p - closest_point;
      HAPI::Vec3 v;
      if( !children[1]->isLeaf() ) {
        HAPI::Vec3 cp;
        cp = children[1]->boundClosestPoint(p);
        v = (cp-p);
      }

      if( children[1]->isLeaf() || v * v < cv * cv ) {
        Vec3 cp, cn, tc;
        children[1]->closestPoint( p, cp, cn, tc );
        Vec3 v = p - cp;
        if( v * v < cv * cv ) {
          closest_point = cp;
          closest_normal = cn;
          closest_tex_coord = tc;
        }
      }
    } else {
      if( left.get() ) {
        left->closestPoint( p, closest_point, closest_normal, closest_tex_coord  );
      } else {
        right->closestPoint( p, closest_point, closest_normal, closest_tex_coord );
      }
    }
  }
}

void BinaryBoundTree::getAllTriangles( vector< Triangle > &tris ) {
  if ( isLeaf() )  {
    tris.insert( tris.end(), triangles.begin(), triangles.end() );
  }  else   {
    if (left.get()) left->getAllTriangles( tris );
    if (right.get()) right->getAllTriangles( tris );
  }
}

bool operator< (const Vec3 & s1, const Vec3 &s2) {
  return (s1.x < s2.x || 
          (s2.x == s1.x && s1.y < s2.y) ||
          ( s2.x == s1.x && s2.y == s1.y && s1.z < s2.z) );
}

struct lt {
  bool operator()(const pair<Vec3, Vec3>& _Left,
                  const pair<Vec3, Vec3 >& _Right ) const {
    return (_Left.first < _Right.first ||
            (!(_Right.first < _Left.first) && _Left.second < _Right.second));
        }
};

const vector<int> &BinaryBoundTree::getNeighbours() {
  if( !neighbours.empty() ) return neighbours;

  vector< HAPI::Collision::Triangle > tris;
  getAllTriangles(tris);

  neighbours.resize(tris.size()*3, -1 );

  // map from triangle edge(as pair of vertex) to pair of 
  // (triangle index, edge index within triangle) 
  typedef map< pair< Vec3, Vec3 >, pair<int, int>, lt >  EdgeTriangleMap;
  EdgeTriangleMap edges;
 
  for( unsigned int i = 0; i < tris.size(); ++i ) {
    const HAPI::Collision::Triangle &tri = tris[i];

    // ignore invalid tris that are lines or points
    if( tri.a == tri.b || tri.b == tri.c || tri.a == tri.c ) {
      continue;
    }

    // edges of the triangles. We keep a strict ordering when defining an edge so that 
    // the edge (a,b) will be the same as (b,a).
    pair< Vec3, Vec3 > edge0 = tri.a < tri.b ? make_pair(tri.a, tri.b) : make_pair(tri.b, tri.a );
    pair< Vec3, Vec3 > edge1 = tri.b < tri.c ? make_pair(tri.b, tri.c) : make_pair(tri.c, tri.b );
    pair< Vec3, Vec3 > edge2 = tri.c < tri.a ? make_pair(tri.c, tri.a) : make_pair(tri.a, tri.c );

    // Check if the edge exists in previously processed triangle.
    EdgeTriangleMap::iterator edge0_i = edges.find( edge0 );
    EdgeTriangleMap::iterator edge1_i = edges.find( edge1 );
    EdgeTriangleMap::iterator edge2_i = edges.find( edge2 );

    if( edge0_i != edges.end() ) {
      // shared edge found, update the neighbour array.
      int triangle = (*edge0_i).second.first;
      int edge =  (*edge0_i).second.second;
      neighbours[i*3] = triangle;
      if( neighbours[triangle*3+edge] == -1 ) {
         neighbours[triangle*3+edge] = i;
      }
    } else {
      edges[edge0] = make_pair(i, 0);
    }

   if( edge1_i != edges.end() ) {
      // shared edge found, update the neighbour array.
      int triangle = (*edge1_i).second.first;
      int edge =  (*edge1_i).second.second;
      neighbours[i*3+1] = triangle;
      if( neighbours[triangle*3+edge] == -1 ) {
         neighbours[triangle*3+edge] = i;
      }
    } else {
      edges[edge1] = make_pair(i, 1);
    }

   if( edge2_i != edges.end() ) {
     // shared edge found, update the neighbour array.
      int triangle = (*edge2_i).second.first;
      int edge =  (*edge2_i).second.second;
      neighbours[i*3+2] = triangle;
      if( neighbours[triangle*3+edge] == -1 ) {
        neighbours[triangle*3+edge] = i;
      }
    } else {
      edges[edge2] = make_pair(i, 2);
    }
  }
  return neighbours;
}





void BinaryBoundTree::getAllPrimitives( vector< Triangle > &tris,
                                    vector< LineSegment > &lins,
                                    vector< Point > &poins ) {
  if ( isLeaf() )  {
    tris.insert( tris.end(), triangles.begin(), triangles.end() );
    lins.insert( lins.end(), linesegments.begin(), linesegments.end() );
    poins.insert( poins.end(), points.begin(), points.end() );
  }  else   {
    if (left.get()) left->getAllPrimitives( tris, lins, poins );
    if (right.get()) right->getAllPrimitives( tris, lins, poins );
  }
}

/// Constructor.
OrientedBoxBound::OrientedBoxBound( const Vec3 &_min,
                                    const Vec3 &_max,
                                    const Rotation &_orientation ):
  AABoxBound( _min, _max ),
  orientation( _orientation ) {
  center = (max + min) / 2;
  halfsize = (max - min) / 2;
}

/// Constructor.
OrientedBoxBound::OrientedBoxBound( const Rotation &_orientation,
                                    const Vec3 &_center,
                                    const Vec3 &_size ):
  center( _center ),
  orientation( _orientation ) {

  halfsize = _size / 2;
  min = center - halfsize;
  max = center + halfsize;
}

Vec3 OrientedBoxBound::longestAxis() const {
  Vec3 a = AABoxBound::longestAxis();
  return orientation * a;
}


struct StackElementPrimitive {
  BBPrimitiveTree *tree;
  vector< int > primitives;
}; 

BBPrimitiveTree::BBPrimitiveTree(
            BoundNewFunc func, 
            const vector< GeometryPrimitive * > &primitive_vector,
            unsigned int max_nr_primitives_in_leaf ):
  left( NULL ), right( NULL ), new_func( func ) {

  std::stack< StackElementPrimitive > stack;
  
  if( primitive_vector.size() == 0 ) return;
 
  // add the starting subtree to stack...
  StackElementPrimitive start;
  start.tree = this;

  vector< Vec3 > point_reps;
  point_reps.reserve( primitive_vector.size() );

  start.primitives.reserve( primitive_vector.size() );
  for( unsigned int i = 0; i < primitive_vector.size(); ++i ) {
    start.primitives.push_back( i );
    point_reps.push_back( primitive_vector[i]->pointRepresentation() );
  }



  stack.push( start );
  //cerr << start.triangles.size() << endl;
  //  BUILD TREE
  while( !stack.empty() ) {
    const std::vector< int > &stack_primitives = stack.top().primitives;
    BBPrimitiveTree* stack_tree = stack.top().tree;

    if ( stack_primitives.size() <= max_nr_primitives_in_leaf ) {
      //  build a leaf
      stack_tree->primitives.reserve( stack_primitives.size() );
      for( unsigned int i = 0; i < stack_primitives.size(); ++i ) {
        stack_tree->primitives.push_back(
          primitive_vector[stack_primitives[i]] );
      }
      stack.pop();
      continue;
    }

    std::vector<Vec3 > points;
    points.reserve( stack_primitives.size() * 3 );
    for (unsigned int i=0 ; i<stack_primitives.size() ; ++i ) {
      GeometryPrimitive *a_primitive = primitive_vector[ stack_primitives[i ]];
      Triangle *tri = dynamic_cast< Triangle * >(a_primitive);
      if( tri ) {
        points.push_back( tri->a );
        points.push_back( tri->b );
        points.push_back( tri->c );
      } else {
        Sphere * sph = dynamic_cast< Sphere * >(a_primitive);
        if( sph ) {
          Vec3 x_axis = Vec3( 1, 0, 0 );
          Vec3 y_axis = Vec3( 0, 1, 0 );
          Vec3 z_axis = Vec3( 0, 0, 1 );

          points.push_back( sph->center + sph->radius * x_axis +
                            sph->radius * y_axis + sph->radius * z_axis );
          points.push_back( sph->center + sph->radius * x_axis -
                            sph->radius * y_axis + sph->radius * z_axis );
          points.push_back( sph->center - sph->radius * x_axis +
                            sph->radius * y_axis + sph->radius * z_axis );
          points.push_back( sph->center - sph->radius * x_axis -
                            sph->radius * y_axis + sph->radius * z_axis );
          points.push_back( sph->center + sph->radius * x_axis +
                            sph->radius * y_axis - sph->radius * z_axis );
          points.push_back( sph->center + sph->radius * x_axis -
                            sph->radius * y_axis - sph->radius * z_axis );
          points.push_back( sph->center - sph->radius * x_axis +
                            sph->radius * y_axis - sph->radius * z_axis );
          points.push_back( sph->center - sph->radius * x_axis -
                            sph->radius * y_axis - sph->radius * z_axis );
        } else {
          LineSegment * lineseg = dynamic_cast< LineSegment * >(a_primitive);
          if( lineseg ) {
            points.push_back( lineseg->start );
            points.push_back( lineseg->end );
          } else {
            Point * pt = dynamic_cast< Point * >(a_primitive);
            if( pt ) {
              points.push_back( pt->position );
            }
          }
        }
      }
    }
    
    //  build bounding shape
    stack_tree->bound.reset( new_func() );
    stack_tree->bound->fitAroundPoints( points );
    
    //  DIVIDE SUBSET AND RECURSE
    //  longest axis
    Vec3 axis = stack_tree->bound->longestAxis();

    //  middle point of current set
    Vec3 mid(0,0,0);

    for(unsigned int i = 0 ; i < stack_primitives.size() ; ++i) {
      mid = mid + point_reps[stack_primitives[i]];
    }

    mid = (1.0f/stack_primitives.size()) * mid;  //divide by N
    
    //  build subsets based on axis/middle point
    std::vector<int> left_primitives;
    std::vector<int> right_primitives;
    
    left_primitives.reserve( stack_primitives.size() / 2 );
    right_primitives.reserve( stack_primitives.size() / 2 );

    
    for( unsigned int i = 0 ; i<stack_primitives.size() ; ++i ) {
      Vec3 mid_to_point = point_reps[ stack_primitives[i] ] - mid;
      if ( mid_to_point * axis < 0 )
        left_primitives.push_back(stack_primitives[i]);
      else
        right_primitives.push_back(stack_primitives[i]);
    }
    
    // anity check. sometimes current subset cannot be divided by longest 
    // axis: change axis or just half
    if( ( left_primitives.size() == 0 ) || ( right_primitives.size() == 0 ) ) {
      left_primitives.clear();
      right_primitives.clear();
      for (size_t i = 0 ; i<stack_primitives.size()/2 ; ++i) 
          left_primitives.push_back(stack_primitives[i]);
      for (size_t i = stack_primitives.size()/2 ;
            i<stack_primitives.size() ; ++i) 
        right_primitives.push_back(stack_primitives[i]);
    }
    
    stack.pop();

    //  do recurse
    if( left_primitives.size() != 0 ) {
      stack_tree->left.reset( new BBPrimitiveTree );
      
      StackElementPrimitive element;
      element.tree = stack_tree->left.get();
      element.primitives.swap( left_primitives );
      stack.push( element );
    }
    
    if( right_primitives.size() != 0 ) {
      stack_tree->right.reset( new BBPrimitiveTree );
      
      StackElementPrimitive element;
      element.tree = stack_tree->right.get();
      element.primitives.swap( right_primitives );
      stack.push( element );
    }
  }
}

void BBPrimitiveTree::getConstraints(
                                  const Vec3 &point,
                                  Constraints &constraints,
                                  FaceType face,
                                  HAPIFloat radius ) {
  if ( isLeaf() )  {
    for( unsigned int i = 0; i < primitives.size(); ++i ) {
      GeometryPrimitive *gp = primitives[i];
      gp->getConstraints( point, constraints, face, radius );
    }
  }  else   {
    //if ( bound->boundIntersect( from, to ) )  {
    if (left.get()) left->getConstraints( point, constraints, face, radius );
    if (right.get()) right->getConstraints( point, constraints, face, radius );
    
  }
}

void BBPrimitiveTree::getPrimitivesWithinRadius(
                                    const Vec3 &p,
                                    HAPIFloat radius,
                                    vector< GeometryPrimitive * > &result ) {
  HAPIFloat r2 = radius * radius;
  if ( isLeaf() )  {
    for( H3DUtil::AutoRefVector< GeometryPrimitive >::const_iterator
          i = primitives.begin();
         i != primitives.end(); ++i ) {
      Vec3 cp, tmp;
      (*i)->closestPoint( p, cp, tmp, tmp );
      if( (cp - p).lengthSqr() <= r2 )
        result.push_back( *i );
    }
    //cerr << "!: " << triangles.size() << endl;
           //result.insert( result.end(), triangles.begin(), triangles.end() );
  }  else   {
    Vec3 cp = bound->boundClosestPoint( p );
    if( (cp - p).lengthSqr() > r2 && !bound->insideBound(p) ) return;

    if (left.get()) left->getPrimitivesWithinRadius( p, radius, result );
    if (right.get()) right->getPrimitivesWithinRadius( p, radius, result );
  }
}

void BBPrimitiveTree::closestPoint( const Vec3 &p,
                                    Vec3 &closest_point,
                                    Vec3 &closest_normal,
                                    Vec3 &closest_tex_coord ) {
  if ( isLeaf() )  {
    if( primitives.size() == 0 ) return;
    HAPIFloat d2 = std::numeric_limits<HAPIFloat>::max();
    for( H3DUtil::AutoRefVector< GeometryPrimitive >::const_iterator
          i = primitives.begin();
         i != primitives.end(); ++i ) {
      Vec3 point, normal, tmp_tc;
      (*i)->closestPoint( p, point, normal, tmp_tc );
      Vec3 v = p - point;
      HAPIFloat new_d2 = v * v;
      if( new_d2 < d2 ) {
        closest_point = point;
        closest_normal = normal;
        d2 = new_d2;
        closest_tex_coord = tmp_tc;
      }
    }
  }  else   {
   
    if( left.get() && right.get() ) {
      vector< BBPrimitiveTree * > children(2);
      if( left->insideBound( p ) ) {
        children[0] = left.get();
        children[1] = right.get();
      } else {
        children[0] = right.get();
        children[1] = left.get();
      }

      children[0]->closestPoint( p, closest_point, closest_normal, closest_tex_coord );
      HAPI::Vec3 cv = closest_point - p;
      HAPI::Vec3 v;
      if( !children[1]->isLeaf() ) {
        HAPI::Vec3 cp;
        cp = children[1]->boundClosestPoint(p);
        v = (cp-p);
      }

      if( children[1]->isLeaf() || v * v < cv * cv ) {
        Vec3 cp, cn, tc;
        children[1]->closestPoint( p, cp, cn, tc );
        Vec3 v = cp - p;
        if( v * v < cv * cv ) {
          closest_point = cp;
          closest_normal = cn;
          closest_tex_coord = tc;
        }
      }
    } else {
      if( left.get() ) {
        left->closestPoint( p, closest_point, closest_normal, closest_tex_coord );
      } else {
        right->closestPoint( p, closest_point, closest_normal, closest_tex_coord );
      }
    }
  }
}

void BBPrimitiveTree::getAllPrimitives( vector< GeometryPrimitive * > &prim ) {
  if ( isLeaf() )  {
    prim.insert( prim.end(), primitives.begin(), primitives.end() );
  }  else   {
    if (left.get()) left->getAllPrimitives( prim );
    if (right.get()) right->getAllPrimitives( prim );
  }
}

bool BBPrimitiveTree::lineIntersect( const Vec3 &from, 
                                     const Vec3 &to,
                                     IntersectionInfo &result,
                                     FaceType face ) {
  if ( isLeaf() )  {
    IntersectionInfo tempResult;
    for( unsigned int i = 0; i < primitives.size(); ++i ) {
      GeometryPrimitive *gp = primitives[i];
      if( gp->lineIntersect( from, to, tempResult, face ) ) {
        if( result.intersection) {
          if( (tempResult.point - from).lengthSqr() < 
              (result.point - from).lengthSqr() )
            result = tempResult;
        }
        else
          result = tempResult;
        return true;
      }
    }
    return false;
  }  else   {
    if ( bound->boundIntersect( from, to ) )  {
      bool overlap = false;
      
      if (left.get()) overlap |=
        left->lineIntersect( from, to, result, face );
      if (right.get()) overlap |=
        right->lineIntersect( from, to, result, face );
      
      return overlap;
    }
    else return false;
  }
}

bool BBPrimitiveTree::movingSphereIntersect( HAPIFloat radius,
                                             const Vec3 &from, 
                                             const Vec3 &to ) {
  if ( isLeaf() )  {
    for( unsigned int i = 0; i < primitives.size(); ++i ) {
      GeometryPrimitive *gp = primitives[i];
      if( gp->movingSphereIntersect( radius, from, to ) )  return true;
    }
    return false;
  }  else   {
    if ( bound->boundMovingSphereIntersect( radius, from, to ) )  {
      bool overlap = false;
      
      if (left.get()) {
        overlap = left->movingSphereIntersect( radius, from, to );
        if( overlap ) return overlap;
      }
      if (right.get()) 
        overlap = right->movingSphereIntersect( radius,from, to );
      
      return overlap;
    }
    else return false;
  }
}

void BBPrimitiveTree::render() {
  if( !isLeaf() ) {
    if( left.get() ) left->render();
    if( right.get() ) right->render();
  } else {
    for( unsigned int i = 0; i < primitives.size(); ++i ) { 
      primitives[i]->render();
    }
  }
}

void BBPrimitiveTree::renderBounds( int depth ) {
  if( !isLeaf() ) {
    if( depth == 0 ) {
      bound->render();
    } else {
      if( left.get() ) left->renderBounds( depth - 1 );
      if( right.get() ) right->renderBounds( depth - 1 );
    }
  } else {
#ifdef HAVE_OPENGL
    glPushAttrib( GL_POLYGON_BIT );
    glPolygonMode( GL_FRONT_AND_BACK, GL_LINE );
#endif
    for( unsigned int i = 0; i < primitives.size(); ++i ) { 
      primitives[i]->render();
    }
#ifdef HAVE_OPENGL
    glPopAttrib();
#endif
  }
}

void BBPrimitiveTree::getPrimitivesIntersectedByMovingSphere(
                                      HAPIFloat radius,
                                      Vec3 from,
                                      Vec3 to,
                                      vector< GeometryPrimitive * > &result) {
  if ( isLeaf() )  {
    for( H3DUtil::AutoRefVector< GeometryPrimitive >::const_iterator
          i = primitives.begin();
         i != primitives.end(); ++i ) {
      if( (*i)->movingSphereIntersect( radius, from, to ) )
        result.push_back( *i );
    }
  }  else   {
    if(  !bound->boundMovingSphereIntersect( radius, from, to ) ) return;

    if (left.get()) left->getPrimitivesIntersectedByMovingSphere(
      radius, from, to, result );
    if (right.get()) right->getPrimitivesIntersectedByMovingSphere(
      radius, from, to, result );
  }
}


bool Triangle::getConstraint(  const Vec3 &point,
                               Collision::PlaneConstraint *constraint,
                               FaceType face ) {
  Vec3 closest_point, cp_normal, cp_tex_coord;
  closestPoint( point, closest_point, cp_normal, cp_tex_coord );
  Vec3 pc_normal = point - closest_point;
  
  if( face == Collision::FRONT ) {
    if(pc_normal * cp_normal < 0 ) return false;
  } else if( face == Collision::BACK ) {
    if(pc_normal * cp_normal > 0 ) return false;
  }
  HAPIFloat l2 = pc_normal.x*pc_normal.x+ pc_normal.y*pc_normal.y+ pc_normal.z*pc_normal.z;
  // if the length already is 1 we don't have to do anything
  if( H3DUtil::H3DAbs(l2-1) > Constants::epsilon ) {
    HAPIFloat l = H3DUtil::H3DSqrt( l2 );
    if( H3DUtil::H3DAbs(l) >= Constants::epsilon ) {
      pc_normal.x /= l;
      pc_normal.y /= l;
      pc_normal.z /= l;
    } else {
      // Choose primitive normal instead.
      pc_normal = cp_normal;
    }
  }
  
  *constraint = PlaneConstraint( closest_point, pc_normal,
                                 cp_tex_coord, NULL, this );
  return true;
}

void GeometryPrimitive::getTangentSpaceMatrix( const Vec3 &point,
                           Matrix4 &result_mtx ) {}

void Triangle::getTangentSpaceMatrix( const Vec3 &point,
                                      Matrix4 &result_mtx ) {
  //http://www.blacksmith-studios.dk/projects/downloads/tangent_matrix_derivation.php
  //code for calculating matrix.

  //calculate lineary independant basis vectors for texture space
  // The third texture coordinate will not be transformed correctly by this
  // matrix because there is to little information in a triangle.
  Vec3 tbta = tb - ta;
  Vec3 tcta = tc - ta;

  HAPIFloat denom = tbta.x * tcta.y - tcta.x * tbta.y;
  if( H3DUtil::H3DAbs( denom ) > Constants::epsilon ) {
    HAPIFloat eMtex_invert = 1 / denom;
    Vec3 tbase1 = eMtex_invert * ( tcta.y * ab - tbta.y * ac );
    Vec3 tbase2 = eMtex_invert * ( -tcta.x * ab + tbta.x * ac );
    Vec3 tbase3 = tbase1 % tbase2;

    result_mtx = Matrix4( tbase1.x, tbase2.x, tbase3.x, 0,
                          tbase1.y, tbase2.y, tbase3.y, 0,
                          tbase1.z, tbase2.z, tbase3.z, 0,
                                 0,        0,        0, 1 ).inverse();
  }
}

void Sphere::getTangentSpaceMatrix( const Vec3 &point,
                                    Matrix4 &result_mtx ) {
  // Calculated a jacobian with
  // s = arctan( -x/-z) / 2pi,
  // t = 1 - arccos( y / sqrt(x^2 + y^2 + z^2 ) ) / pi,
  // u = 0
  HAPIFloat denom = Vec3( point ).lengthSqr();
  if( H3DUtil::H3DAbs( denom ) > Constants::epsilon ) {

    HAPIFloat length_sqr_inv = 1 / denom ;
    HAPIFloat x2_z2 = point.x * point.x + point.z * point.z;
    HAPIFloat x2_z2_inv = 1 / x2_z2;
    HAPIFloat sqrt_x2_z2_inv = H3DUtil::H3DSqrt( x2_z2_inv );

#if 1
    // Created a third vector from a cross product of the other two only to get
    // the calculated matrix invertable, this might be introducting errors but
    // they should be small enough to be neglectable.
    Vec3 ds_tangent = Vec3( point.z *
                              CollisionInternals::two_pi_inv * x2_z2_inv,
                            0.0f,
                            -point.x * CollisionInternals::two_pi_inv
                            * x2_z2_inv );
    Vec3 dt_tangent = Vec3( -point.x * point.y * CollisionInternals::pi_inv *
                            sqrt_x2_z2_inv * length_sqr_inv,
                            CollisionInternals::pi_inv * x2_z2 * sqrt_x2_z2_inv
                            * length_sqr_inv,
                            -point.y * point.z * CollisionInternals::pi_inv *
                            sqrt_x2_z2_inv * length_sqr_inv );
    Vec3 du_tangent = ds_tangent % dt_tangent;

    result_mtx[0][0] = ds_tangent.x;
    result_mtx[0][1] = ds_tangent.y;
    result_mtx[0][2] = ds_tangent.z;
    result_mtx[0][3] = 0.0f;
    result_mtx[1][0] = dt_tangent.x;
    result_mtx[1][1] = dt_tangent.y;
    result_mtx[1][2] = dt_tangent.z;
    result_mtx[1][3] = 0.0f;
    result_mtx[2][0] = du_tangent.x;
    result_mtx[2][1] = du_tangent.y;
    result_mtx[2][2] = du_tangent.z;
    result_mtx[2][3] = 0.0f;
    result_mtx[3][0] = 0.0f;
    result_mtx[3][1] = 0.0f;
    result_mtx[3][2] = 0.0f;
    result_mtx[3][3] = 1.0f;
#else
    // Saved in case we want to change back to non invertable matrix
    result_mtx[0][0] = point.z * CollisionInternals::two_pi_inv * x2_z2_inv;
    result_mtx[0][1] = 0.0f;
    result_mtx[0][2] = -point.x * CollisionInternals::two_pi_inv * x2_z2_inv;
    result_mtx[0][3] = 0.0f;
    result_mtx[1][0] = -point.x * point.y * CollisionInternals::pi_inv *
                        sqrt_x2_z2_inv * length_sqr_inv;
    result_mtx[1][1] = -CollisionInternals::pi_inv * x2_z2 * sqrt_x2_z2_inv *
                        length_sqr_inv * length_sqr_inv;
    result_mtx[1][2] = -point.y * point.z * CollisionInternals::pi_inv *
                        sqrt_x2_z2_inv * length_sqr_inv;
    result_mtx[1][3] = 0.0f;
    result_mtx[2][0] = 0.0f;
    result_mtx[2][1] = 0.0f;
    result_mtx[2][2] = 0.0f;
    result_mtx[2][3] = 0.0f;
    result_mtx[3][0] = 0.0f;
    result_mtx[3][1] = 0.0f;
    result_mtx[3][2] = 0.0f;
    result_mtx[3][3] = 1.0f;
#endif
  }
}

bool Plane::movingSphereIntersect( HAPIFloat radius,
                                   const Vec3 &from,
                                   const Vec3 &to,
                                   IntersectionInfo &result ) {
  // p 221 in Real Time Collision Detection by Christer Ericsson
  HAPIFloat d = normal * point;
  HAPIFloat dist = normal * from - d;
  if( H3DUtil::H3DAbs( dist ) <= radius ) {
    result.t = 0;
    result.intersection = true;
    if( dist > 0.0 )
      result.normal = normal;
    else
      result.normal = -normal;
    Vec3 tmp;
    closestPoint( from, result.point, tmp, tmp );
    result.primitive = this;
    return true;
  } else {
    Vec3 v = to - from;
    HAPIFloat denom = normal * v;
    if( denom * dist >= 0.0 ) {
      return false;
    } else {
      HAPIFloat r = dist > 0.0 ? radius : -radius;
      result.t = ( r - dist ) / denom;
      result.point = from + result.t * v - radius * normal;
      result.intersection = true;
      if( dist > 0.0 )
        result.normal = normal;
      else
        result.normal = -normal;
      result.primitive = this;
      return true;
    }
  }
  return false;
}

bool LineSegment::movingSphereIntersect( HAPIFloat radius,
                                         const Vec3& from,
                                         const Vec3& to,
                                         IntersectionInfo &result ) {
  HAPIFloat t;
  bool hit = intersectSegmentCylinder( from, to, start, end, radius, t );
  
  if( !hit ) {
    Sphere sphere( start, radius );
    IntersectionInfo temp_result;
    if( sphere.lineIntersect( from, to, temp_result ) ) {
      t = temp_result.t;
      hit = true;
    }

    sphere.center = end;
    if( sphere.lineIntersect( from, to, temp_result ) ) {
      if( hit ) {
        if( temp_result.t < t )
          t = temp_result.t;
      } else {
        t = temp_result.t;
        hit = true;
      }
    }
  }

  if( hit ) {
    result.t = t;
    Vec3 center_point = from + (to - from ) * t;
    Vec3 tmp;
    closestPoint( center_point, result.point, tmp, tmp );
    result.normal = center_point - result.point;
    result.normal.normalizeSafe();
    result.primitive = this;
    result.intersection = true;
    return true;
  }
  return false;
}

bool Point::movingSphereIntersect( HAPIFloat radius,
                                   const Vec3 &from,
                                   const Vec3 &to,
                                   IntersectionInfo &result ) {
  Sphere sphere( position, radius );
  if( sphere.lineIntersect( from, to, result ) ) {
    result.normal = ( from + ( to - from ) * result.t ) - position;
    result.normal.normalize();
    result.point = position;
    return true;
  }
  return false;
}

bool BBPrimitiveTree::movingSphereIntersect( HAPIFloat radius,
                                             const Vec3 &from, 
                                             const Vec3 &to,
                                             IntersectionInfo &result ) {
  if ( isLeaf() )  {
    IntersectionInfo temp_result;
    for( unsigned int i = 0; i < primitives.size(); ++i ) {
      GeometryPrimitive *pr = primitives[i];
      if( pr->movingSphereIntersect( radius, from, to, temp_result ) ) {
        if( temp_result.intersection ) {
          if( result.intersection ) {
            if( temp_result.t < result.t )
              result = temp_result;
          } else {
            result = temp_result;
          }
        }
      }
    }

    return result.intersection;
  }  else   {
    if ( bound->boundMovingSphereIntersect( radius, from, to ) )  {
      bool overlap = result.intersection;
      
      if (left.get()) {
        overlap = left->movingSphereIntersect( radius, from, to, result );
      }
      if (right.get()) 
        overlap = overlap ||
                  right->movingSphereIntersect( radius, from, to, result );
      
      return overlap;
    }
    else return false;
  }
}

void Cylinder::getTangentSpaceMatrix( const Vec3 &point,
                                      Matrix4 &result_mtx ) {
  // Get closest point, if on CYLINDER do one thing otherwise
  // do another.
  Vec3 closest, tmp;
  bool inside;
  CylinderPart part;
  closestPoint( point, closest, tmp, tmp, inside, part );
  if( part == CYLINDER ) {
    // When the point is on the cylinder a Jacobian is created from the
    // functions.
    // s(x,y,z) = pi + arctan( x / z )
    // t(x,y,z) = y / d Where d = (end_point - start_point).length().
    // u(x,y,z) = 0.
    // In order to get an invertable matrix the cross-product is taken
    // between the two first rows of the jacobian. This is not entirely
    // correct since a movement perpendicular to the center segment does
    // not affect the third texture coordinate at all. The z value of a result
    // between this matrix and a point should therefore be ignored.
    HAPIFloat constant = CollisionInternals::two_pi_inv / 
      (closest.x * closest.x + closest.z * closest.z );
    result_mtx[0][0] = constant * closest.z;
    result_mtx[0][1] = 0;
    result_mtx[0][2] = -constant * closest.x;
    result_mtx[0][3] = 0;
    result_mtx[1][0] = 0;
    result_mtx[1][1] = 1 / height;
    result_mtx[1][2] = 0;
    result_mtx[1][3] = 0;
#if 1
    // Invertable matrix.
    result_mtx[2][0] = -result_mtx[0][2] * result_mtx[1][1];
    result_mtx[2][1] = 0;
    result_mtx[2][2] = result_mtx[0][0] * result_mtx[1][1];
    result_mtx[2][3] = 0;
#else
    // Non invertable matrix.
    result_mtx[2][0] = 0;
    result_mtx[2][1] = 0;
    result_mtx[2][2] = 0;
    result_mtx[2][3] = 0;
#endif
    
    result_mtx[3][0] = 0;
    result_mtx[3][1] = 0;
    result_mtx[3][2] = 0;
    result_mtx[3][3] = 1;
  } else {
    // When the point is on one of the end caps a Jacobian is created from the
    // functions.
    // s(x,y,z) = (x + radius) / (2 * radius)
    // t(x,y,z) = (-z + radius) / (2 * radius)
    // u(x,y,z) = 0.
    // In order to get an invertable matrix the cross-product is taken
    // between the two first rows of the jacobian. This is not entirely
    // correct since a movement perpendicular to the center segment does
    // not affect the third texture coordinate at all. The z value of a result
    // between this matrix and a point should therefore be ignored.
    result_mtx[0][0] = 1 / ( 2 * radius);
    result_mtx[0][1] = 0;
    result_mtx[0][2] = 0;
    result_mtx[0][3] = 0;
    
    result_mtx[1][0] = 0;
    result_mtx[1][1] = 0;
    result_mtx[1][2] = -result_mtx[0][0];
    if( part == END_CAP ) {
      // result_matrix is multiplied by a transform matrix to transform
      // the origo into the end cap plane. Since the transformation is
      // only in the y-direction this is the only component of the matrix
      // that changes
      result_mtx[1][3] = -height * result_mtx[1][2];
    } else {
      result_mtx[1][3] = 0;
    }
    
#if 1
    // Invertable matrix
    result_mtx[2][0] = 0;
    result_mtx[2][1] = 0;
    result_mtx[2][2] = - result_mtx[0][0] * result_mtx[0][0];
    result_mtx[2][3] = 0;
#else
    // Non invertable matrix
    result_mtx[2][0] = 0;
    result_mtx[2][1] = 0;
    result_mtx[2][2] = 0;
    result_mtx[2][3] = 0;
#endif
    result_mtx[3][0] = 0;
    result_mtx[3][1] = 0;
    result_mtx[3][2] = 0;
    result_mtx[3][3] = 1;
  }
}

void Cylinder::closestPoint( const Vec3 &p,
                             Vec3 &closest_point,
                             Vec3 &normal,
                             Vec3 &tex_coord ) {
  bool inside;
  CylinderPart part;
  closestPoint( p, closest_point, normal, tex_coord, inside, part );
}

void Cylinder::closestPoint( const Vec3 &p,
                             Vec3 &closest_point,
                             Vec3 &normal,
                             Vec3 &tex_coord,
                             bool &inside,
                             CylinderPart &part ) {
  inside = false;
  // Check if point is outside or within center segment.
  if( p.y > height || p.y < 0 ) {
    // Point is on cap points
    HAPIFloat cap_center_point_y = 0;
    HAPIFloat cap_normal_y;
    if ( p.y > height ) {
      cap_center_point_y = height;
      cap_normal_y = 1;
      if( !end_cap ) part = CYLINDER;
      else part = END_CAP;
    } else {
      cap_center_point_y = 0;
      cap_normal_y = -1;
      if( !start_cap ) part = CYLINDER;
      else part = START_CAP;
    }
    // Project onto end-cap plane.
    // In the following calculations only the y component is
    // used in those cases where the x and z component are 0.
    // For example for all dot-products with the cap_center_point
    // which is Vec3( 0, 0 or height, 0 ) and cap_normal which
    // is Vec3( 0, -1 or 1, 0 )
    Vec3 point_on_plane = Vec3( p.x, cap_center_point_y, p.z );
    Vec3 center_edge = Vec3( p.x, 0, p.z );
    HAPIFloat center_edge_length_sqr = center_edge.lengthSqr();
    if( center_edge_length_sqr > radius * radius || part == CYLINDER ) {
      center_edge = center_edge / H3DUtil::H3DSqrt( center_edge_length_sqr );
      closest_point = center_edge * radius;
      closest_point.y = cap_center_point_y;
      normal = p - closest_point;
      normal.normalizeSafe();
    } else {
      closest_point = point_on_plane;
      normal = Vec3( 0, cap_normal_y, 0 );
    }
    tex_coord = getTextureCoordinate( closest_point, part );
  } else {
    // Point is between the end cap planes.
    // Point might be on cylinder curvature or on any of the caps.
    // Project point onto center cylinder segment
    Vec3 center_edge = Vec3( p.x, 0, p.z );
    HAPIFloat center_edge_length_sqr = center_edge.lengthSqr();
    HAPIFloat dist_sqr = center_edge_length_sqr - radius * radius;
    // If the squared distance is negative then point is inside cylinder.
    if( dist_sqr < 0.0f ) {
      inside = true;
      dist_sqr = -dist_sqr;
      bool on_cylinder_curv = false;
      // This section contains some duplicated code inside the if cases
      // in order to avoid extra multiplications.
      // It simply checks which distances to compare and chose the closest
      // point.
      if( end_cap && start_cap ) {
        HAPIFloat dist_end_plane = height - p.y;
        HAPIFloat dist_end_sqr = dist_end_plane * dist_end_plane;
        HAPIFloat dist_start_sqr = p.y * p.y;

        if( ( dist_end_sqr < Constants::epsilon ||
              dist_start_sqr < Constants::epsilon) &&
            dist_sqr < Constants::epsilon ) {
          // normal should be a combination of several things.
          assert(0);
        }

        // Compare distances to figure out which part of the cylinder the
        // point is closest to.
        if( dist_end_plane < p.y ) {
          if( dist_end_sqr < dist_sqr ) {
            closest_point = Vec3( p.x, height, p.z );
            normal = Vec3( 0, 1, 0 );
            part = END_CAP;
            tex_coord = getTextureCoordinate( closest_point, part );
          } else {
            on_cylinder_curv = true;
          }
        } else {
          if( dist_start_sqr < dist_sqr ) {
            closest_point = center_edge;
            normal = Vec3( 0, -1, 0 );
            tex_coord = getTextureCoordinate( closest_point, part );
          } else {
            on_cylinder_curv = true;
          }
        }
      } else if( end_cap ) {
        HAPIFloat dist_end_plane = height - p.y;
        if( dist_end_plane * dist_end_plane < dist_sqr ) {
          closest_point = Vec3( p.x, height, p.z );
          normal = Vec3( 0, 1, 0 );
          part = END_CAP;
          tex_coord = getTextureCoordinate( closest_point, part );
        } else
          on_cylinder_curv = true;
      } else if( start_cap ) {
        if( p.y * p.y < dist_sqr ) {
          closest_point = center_edge;
          normal = Vec3( 0, -1, 0 );
          part = START_CAP;
          tex_coord = getTextureCoordinate( closest_point, part );
        } else {
          on_cylinder_curv = true;
        }
      } else
        on_cylinder_curv = true;

      if( on_cylinder_curv ) {
        part = CYLINDER;
        if( H3DUtil::H3DAbs( center_edge_length_sqr ) > Constants::epsilon ) {
          center_edge = center_edge /
                        H3DUtil::H3DSqrt( center_edge_length_sqr );
          closest_point = center_edge * radius;
          closest_point.y = p.y;
          normal = center_edge;
          tex_coord = getTextureCoordinate( closest_point, CYLINDER );
        } else {
          // If for some reason the point is exactly in the middle of the
          // cylinder (not very likely when dealing with haptics) then
          // just choose any of the closest point on the cylinder surfaces.
          // We choose the one that is on the x-axis.
          normal = Vec3( 1, 0, 0 );
          closest_point = p + normal * radius;
          tex_coord = getTextureCoordinate( closest_point, CYLINDER );
        }
      }
    } else {
      // Point is outside cylinder between end cap planes.
      // This means that the closest_point is on the cylinder surface.
      center_edge = center_edge /
                    H3DUtil::H3DSqrt( center_edge_length_sqr );
      closest_point = center_edge * radius;
      closest_point.y = p.y;
      normal = center_edge;
      tex_coord = getTextureCoordinate( closest_point, CYLINDER );
      part = CYLINDER;
    }
  }
}

// Texture coordinates
// Page 197 in Real time collision detection, with corrections according
// to errata on book home page.
// Added checks for detection of line segement with cylinder surface when
// line segment starts inside cylinder.
bool Cylinder::lineIntersect( const Vec3 &from, 
                              const Vec3 &to,
                              IntersectionInfo &result,
                              FaceType face ) {
  CylinderPart part;
  return lineIntersect( from, to, 0, height, 
                        radius, result, part, face );
}

bool Cylinder::lineIntersect( const Vec3 &from, 
                              const Vec3 &to,
                              const HAPIFloat &start_point,
                              const HAPIFloat &end_point,
                              HAPIFloat _radius,
                              IntersectionInfo &result,
                              CylinderPart &part,
                              FaceType face ) {
  // tmp_from and tmp_to might be switched to correctly detect collisions
  // when from is inside cylinder.
  Vec3 tmp_from = from;
  Vec3 tmp_to = to;
  Vec3 m = Vec3( tmp_from.x, tmp_from.y - start_point, tmp_from.z);
  HAPIFloat d = end_point - start_point;
  HAPIFloat md = m.y * d;
  HAPIFloat dd = d * d;

  Vec3 n = tmp_to - tmp_from;
  HAPIFloat nd = n.y * d;

  // Test if segment fully outside either endcap of cylinder
  if( md < 0.0f && md + nd < 0.0f )
    return false; // Segment outside '_start_point' side of cylinder
  if( md > dd && md + nd > dd )
    return false; // Segment outside '_end_point' side of cylinder

  HAPIFloat r2 = _radius * _radius;
  HAPI::Collision::FaceType face_type = FRONT;
  bool switched = false;
  // Test if from inside cylinder, added check not in the book.
  if( face != FRONT ) {
    // Check if from is inside.
    if( md >= 0.0f && md <= dd ) {
      // From is between the cylinder caps, check if outside _radius.
      Vec3 dist_vec = Vec3( tmp_from.x, 0, tmp_from.z );
      if( dist_vec * dist_vec >= r2 ) {
        // From is outside, if collision face is back only return false.
        if( face == BACK )
          return false;
      } else {
        // Check if to is inside
        bool to_inside = false;
        if( tmp_to.y >= start_point && tmp_to.y <= end_point ) {
          // To is between the cylinder caps, check if outside _radius.
          dist_vec = Vec3( tmp_to.x, 0, tmp_to.z );
          if( dist_vec * dist_vec < r2 ) {
            to_inside = true;
          }
        }

        if( to_inside ) {
          // Line segment is completely inside, return false.
          return false;
        } else {
          // Switch to and from.
          tmp_from = to;
          tmp_to = from;
          m = Vec3( tmp_from.x, tmp_from.y - start_point, tmp_from.z);
          md = m.y * d;
          face_type = BACK;
          n = -n;
          nd = -nd;
          switched = true;
        }
      }
    } else if( face == BACK ) {
      // From is outside, if collision face is back only return false.
      return false;
    }
  }

  HAPIFloat nn = n * n;
  HAPIFloat mn = m * n;
  HAPIFloat a = dd * nn - nd * nd;
  HAPIFloat k = m * m - r2;
  HAPIFloat c = dd * k - md * md;
  if( H3DUtil::H3DAbs(a) < Constants::epsilon ) {
    // Segment runs parallel to cylinder axis
    if( c > 0.0f )
      return false; // 'tmp_from' and thus the segment lie outside cylinder.
    // Now known that segment intersects cylinder; figure out how it intersects
    if( md < 0.0f ) {
      result.t = -mn / nn; // Intersect segment against '_start_point' endcap
      result.normal = Vec3( 0, -1, 0 );
      part = START_CAP;
    } else if( md > dd ) {
      result.t = (nd - mn ) / nn; // Intersect segment 
                                  // against '_end_point' endcap
      result.normal = Vec3( 0, 1, 0 );
      part = END_CAP;
    } else {
      if( face == FRONT )
        return false;
      // Intersect segment from the inside. It is parallell and we know
      // already that tmp_from is inside the cylinder.
      // The algorithm should only come here for very short lines.
      if( tmp_to.y < start_point || tmp_to.y > end_point ) {
        HAPIFloat t = -mn / nn;
        result.t = ( nd - mn ) / nn;
        if( t > 0.0f && t < result.t ) {
          part = START_CAP;
          result.t = t;
          result.normal = Vec3( 0, -1, 0 );
        } else {
          result.normal = Vec3( 0, 1, 0 );
          part = END_CAP;
        }
        if( switched ) result.t = 1.0 - result.t;
      } else
        return false;
    }

    result.intersection = true;
    result.primitive = this;
    result.point = tmp_from + result.t * n;
    result.face = face_type;
    result.tex_coord = getTextureCoordinate( result.point, START_CAP );
    return true;
  }

  HAPIFloat b = dd * mn - nd * md;
  HAPIFloat discr = b * b - a * c;
  if( discr < 0.0f )
    return false; // no real roots; no intersection

  HAPIFloat t = (-b - H3DUtil::H3DSqrt( discr ) ) / a;
  HAPIFloat t0 = t;
  if( md + t * nd < 0.0f ) {
    // Intersection outside cylinder on '_start_point' side
    if( nd <= 0.0f )
      return false; //Segment pointing away from endcap.
    t = -md / nd;
    // Not in the book or the errata, but without this there will be
    // false positives. Did I make a mistake somewhere else?
    if( t < 0.0f )
      return false;
    // Keep intersection if Dot(S(t) - p, S(t) - p) <= r^2
    if(k + t * (2.0f * mn + t * nn) <= 0.0f) {
      if( start_cap ) {
        result.face = face_type;
        result.intersection = true;
        result.point = tmp_from + t * n;
        result.normal = Vec3( 0, -1, 0 );
        result.primitive = this;
        if( switched ) result.t = 1.0 - t;
        else result.t = t;
        part = START_CAP;
        result.tex_coord = getTextureCoordinate( result.point, START_CAP );
        return true;
      } else {
        if( face == FRONT )
          return false;

        HAPIFloat t1 = (-b + H3DUtil::H3DSqrt( discr ) ) / a;
        face_type = BACK;
        if( end_cap ) {
          HAPIFloat t2 = (dd - md ) / nd;
          if( t2 >= 0.0f && t2 <= 1.0f ) {
            if( t1 <= t2 ) {
              t0 = t1;
            } else {
              result.intersection = true;
              result.point = tmp_from + t * n;
              result.normal = Vec3( 0, 1, 0 );
              result.primitive = this;
              result.face = face_type;
              if( switched ) result.t = 1.0 - t;
              else result.t = t;
              result.tex_coord = getTextureCoordinate( result.point, END_CAP );
              part = END_CAP;
              return true;
            }
          } else {
            t0 = t1;
          }
        } else {
          t0 = t1;
        }
      }
    }
  } else if (md + t * nd > dd) {
    // Intersection outside cylinder on \91_end_point\92 side
    if (nd >= 0.0f)
      return false; // Segment pointing away from endcap
    t = (dd - md) / nd;
    // Not in the book or the errata, but without this there will be
    // false positives. Did I make a mistake somewhere else?
    if( t < 0.0f )
      return false;
    // Keep intersection if Dot(S(t) - q, S(t) - q) <= r^2
    if(k + dd - 2.0f * md + t * (2.0f * (mn - nd) + t * nn) <= 0.0f) {
      if( end_cap ) {
        result.face = face_type;
        result.intersection = true;
        result.point = tmp_from + t * n;
        result.normal = Vec3( 0, 1, 0 );
        result.primitive = this;
        if( switched ) result.t = 1.0 -t;
        else result.t = t;
        part = END_CAP;
        result.tex_coord = getTextureCoordinate( result.point, part );
        return true;
      } else {
        if( face == FRONT )
          return false;

        HAPIFloat t1 = (-b + H3DUtil::H3DSqrt( discr ) ) / a;
        face_type = BACK;
        if( start_cap ) {
          HAPIFloat t2 = -md / nd;
          if( t2 >= 0.0f && t2 <= 1.0f ) {
            if( t1 <= t2 ) {
              t0 = t1;
            } else {
              result.intersection = true;
              result.point = tmp_from + t * n;
              result.normal = Vec3( 0, -1, 0 );
              result.primitive = this;
              result.face = face_type;
              if( switched ) result.t = 1.0 - t;
              else result.t = t;
              // Tex coord.
              part = START_CAP;
              result.tex_coord = getTextureCoordinate( result.point, part );
              return true;
            }
          } else {
            t0 = t1;
          }
        } else {
          t0 = t1;
        }
      }
    }
  }
  t = t0;
  // Intersection if segment intersects cylinder between the end-caps
  if( t >= 0.0f && t <= 1.0f ) {
    result.face = face_type;
    result.intersection = true;
    result.point = tmp_from + t*n;
    result.normal = Vec3( result.point.x, 0, result.point.z );
    result.normal.normalize();
    result.primitive = this;
    if( switched ) result.t = 1.0 - t;
    else result.t = t;
    part = CYLINDER;
    result.tex_coord = getTextureCoordinate( result.point, CYLINDER );
    return true;
  } else
    return false;
}

// Capsule capsule test, not so important that the test is completely
// accurate. The test is performed by calculating the closest points on
// two lines and then comparing the distance between these points to
// the sum of the radii.
bool Cylinder::movingSphereIntersect( HAPIFloat _radius,
                                      const Vec3 &from, 
                                      const Vec3 &to ) {
  LineSegment line( Vec3(), Vec3( 0, height, 0) );
  HAPIFloat s, t;
  Vec3 c0, c1;
  line.closestPointOnLine( from, to, s, t, c0, c1 );
  HAPIFloat r_sum = _radius + radius;
  if( (c0 - c1).lengthSqr() > r_sum * r_sum )
    return false;
  else return true;
}

bool Cylinder::movingSphereIntersect( HAPIFloat _radius,
                                      const Vec3 &from, 
                                      const Vec3 &to,
                                      IntersectionInfo &result) {
  // movingSphereIntersect will always be done on the full cylinder.
  // The reason for this is because at the moment it is assumed that the moving
  // sphere and the cylinder intersects if the moving sphere starts inside the
  // cylinder.
  Vec3 closest_point, normal, tex_coord;
  bool inside;
  CylinderPart part;
  closestPoint( from, closest_point, normal, tex_coord, inside, part );
  if( inside || (closest_point - from).lengthSqr() <= _radius * _radius ) {
    result.face = FRONT;
    result.intersection = true;
    result.normal = normal;
    result.tex_coord = tex_coord;
    result.point = closest_point;
    result.t = 0;
    result.primitive = this;
    return true;
  }

  // This will give false positives but it is simplest this way.
  HAPIFloat r_sum = radius + _radius;
  if( lineIntersect( from, to, -_radius,
                     height + _radius, r_sum, result,
                     part, FRONT ) ) {
    if( part == END_CAP ) {
      // end cap
      Vec2 diff = Vec2( result.point.x, result.point.z );
      HAPIFloat diff_sqr = diff.lengthSqr();
      if( diff_sqr > radius * radius ) {
        diff = diff / H3DUtil::H3DSqrt( diff_sqr );
        result.point = Vec3( radius * diff.x, height, radius * diff.y );
      } else
        result.point.y = height;
      result.tex_coord = getTextureCoordinate( result.point, END_CAP );
    } else if( part == START_CAP ) {
      // start cap
      Vec2 diff = Vec2( result.point.x, result.point.z );
      HAPIFloat diff_sqr = diff.lengthSqr();
      if( diff_sqr > radius * radius ) {
        diff = diff / H3DUtil::H3DSqrt( diff_sqr );
        result.point = Vec3( radius * diff.x, 0, radius * diff.y );
      } else
        result.point.y = 0;
      result.tex_coord = getTextureCoordinate( result.point, START_CAP );
    } else {
      // cylinder side
      result.point = result.point - result.normal * _radius;
      if( result.point.y < 0 )
        result.point.y = 0;
      else if( result.point.y > height )
        result.point.y = height;
      result.tex_coord = getTextureCoordinate( result.point, CYLINDER );
    }
    return true;
  }

  return false;
}

void Cylinder::render() {
#ifdef HAVE_OPENGL
  CollisionInternals::createCylinderDisplayList();
  glMatrixMode( GL_MODELVIEW );
  glPushMatrix();
  glRotated( -90, 1, 0, 0 );
  glScaled( radius, radius, height );
  glCallList( CollisionInternals::cylinder_display_list_id );
  glPopMatrix();
  int nr_faces = 20;
  if ( end_cap ) {
    // render end_cap
    glBegin( GL_POLYGON );
    glNormal3f( 0, 1, 0 );
    for( int i = 0; i < nr_faces; ++i ) {
      float angle = (float)( i * (H3DUtil::Constants::pi*2) /(float) nr_faces);
      float sina = sin( angle );
      float cosa = cos( angle );
      glVertex3d( -radius * sina, height, -radius * cosa );
    }
    glEnd();
  }
  
  if ( start_cap ) {
    // render start_cap
    glBegin( GL_POLYGON );
    glNormal3f( 0, -1, 0 );
    for( int i = nr_faces; i >= 0; --i ) {
      float angle = (float)( i * (H3DUtil::Constants::pi*2) /(float) nr_faces);
      float sina = sin( angle );
      float cosa = cos( angle );
      glVertex3d( -radius * sina, 0, -radius * cosa );
    }
    glEnd();
  }
#endif
}

void Cylinder::getConstraints( const Vec3 &point,
                               Constraints &constraints,
                               FaceType face,
                               HAPIFloat _radius ) {
  if( point.y > 0.0f && point.y < height ) {
    // If inside and close enough to cap return cap constraint to.
    Vec3 closest_cylinder_normal = Vec3( point.x, 0, point.z );
    HAPIFloat dist = closest_cylinder_normal.length();
    if( dist < radius ) {
      if( face == FRONT )
        return;

      Vec3 proj_pos = Vec3( 0, point.y, 0 );
      // Calculate closest point to cylinder
      HAPIFloat cyl_dist = radius - dist;
      if( _radius < 0 ||
          ( dist > Constants::epsilon && cyl_dist <= _radius ) ) {
        closest_cylinder_normal = closest_cylinder_normal / dist;
#if 1
        // This removes vibrations and reduces fall through when touching
        // the cylinder from inside. The idea is simply to have two planes
        // intersecting at a line in the direction of the cylinder axis
        // through the closest point on the cylinder.y
        Rotation offset( Vec3( 0, 1, 0), 0.01 );
        Vec3 plane_point = proj_pos + radius * closest_cylinder_normal;
        constraints.push_back(
          PlaneConstraint( plane_point,
                           offset * (-closest_cylinder_normal),
                           getTextureCoordinate( plane_point, CYLINDER ),
                           NULL, this ) );
        offset.angle = -offset.angle;
        constraints.push_back(
          PlaneConstraint( plane_point,
                           offset * (-closest_cylinder_normal),
                           getTextureCoordinate( plane_point, CYLINDER ),
                           NULL, this ) );
#else
        constraints.push_back(
          PlaneConstraint( plane_point,
                           -closest_cylinder_normal,
                           getTextureCoordinate( plane_point, CYLINDER ),
                           NULL, this ) );
#endif
      }

      // Distance from point to start_point cap.
      if( start_cap ) {
        if( _radius < 0 || point.y < _radius ) {
          Vec3 plane_point = Vec3( point.x, 0, point.z );
          constraints.push_back(
            PlaneConstraint( plane_point,
                             Vec3( 0, 1, 0 ),
                             getTextureCoordinate( plane_point, START_CAP ),
                             NULL, this ) );
        }
      }

      // Distance from point to end_point cap.
      if( end_cap ) {
        if( _radius < 0 || height - point.y < _radius ) {
          Vec3 plane_point = Vec3( point.x, height, point.z );
          constraints.push_back(
            PlaneConstraint( plane_point,
                             Vec3( 0, -1, 0 ),
                             getTextureCoordinate( plane_point, END_CAP ),
                             NULL, this ) );
        }
      }
      return;
    }
  }
  // If outside return one constraint.
  if( face == BACK )
    return;
  GeometryPrimitive::getConstraints( point, constraints, face, _radius );
}

void AABox::getTangentSpaceMatrix( const Vec3 &point,
                                   Matrix4 &result_mtx ) {
  Vec3 closest, normal, tex_coord;
  PointLocation point_location;
  closestPoint( point, min, max, closest, normal, tex_coord, point_location );
  if( point_location == INSIDE )
    normal = -normal;

  // Matrices are calculated from the jacobian. The third non-zero
  // value of all matrices are simply there to make it invertable.
  if( normal.z > Constants::epsilon ) {
    result_mtx[0][0] = 1 / (max.x - min.x);
    result_mtx[0][1] = 0;
    result_mtx[0][2] = 0;
    result_mtx[0][3] = 0;
    result_mtx[1][0] = 0;
    result_mtx[1][1] = 1 / (max.y - min.y);
    result_mtx[1][2] = 0;
    result_mtx[1][3] = 0;
    result_mtx[2][0] = 0;
    result_mtx[2][1] = 0;
    result_mtx[2][2] = result_mtx[0][0] * result_mtx[1][1];
    result_mtx[2][3] = 0;
    result_mtx[3][0] = 0;
    result_mtx[3][1] = 0;
    result_mtx[3][2] = 0;
    result_mtx[3][3] = 1;
  } else if( normal.z < -Constants::epsilon ) {
    result_mtx[0][0] = 1 / (min.x - max.x);
    result_mtx[0][1] = 0;
    result_mtx[0][2] = 0;
    result_mtx[0][3] = 0;
    result_mtx[1][0] = 0;
    result_mtx[1][1] = 1 / (max.y - min.y);
    result_mtx[1][2] = 0;
    result_mtx[1][3] = 0;
    result_mtx[2][0] = 0;
    result_mtx[2][1] = 0;
    result_mtx[2][2] = result_mtx[0][0] * result_mtx[1][1];
    result_mtx[2][3] = 0;
    result_mtx[3][0] = 0;
    result_mtx[3][1] = 0;
    result_mtx[3][2] = 0;
    result_mtx[3][3] = 1;
  } else if( normal.x > Constants::epsilon ) {
    result_mtx[0][0] = 0;
    result_mtx[0][1] = 0;
    result_mtx[0][2] = 1 / (min.z - max.z);
    result_mtx[0][3] = 0;
    result_mtx[1][0] = 0;
    result_mtx[1][1] = 1 / (max.y - min.y);
    result_mtx[1][2] = 0;
    result_mtx[1][3] = 0;
    result_mtx[2][0] = -result_mtx[0][2] * result_mtx[1][1];
    result_mtx[2][1] = 0;
    result_mtx[2][2] = 0;
    result_mtx[2][3] = 0;
    result_mtx[3][0] = 0;
    result_mtx[3][1] = 0;
    result_mtx[3][2] = 0;
    result_mtx[3][3] = 1;
  } else if( normal.x < -Constants::epsilon ) {
    result_mtx[0][0] = 0;
    result_mtx[0][1] = 0;
    result_mtx[0][2] = 1 / (max.z - min.z);
    result_mtx[0][3] = 0;
    result_mtx[1][0] = 0;
    result_mtx[1][1] = 1 / (max.y - min.y);
    result_mtx[1][2] = 0;
    result_mtx[1][3] = 0;
    result_mtx[2][0] = -result_mtx[0][2] * result_mtx[1][1];
    result_mtx[2][1] = 0;
    result_mtx[2][2] = 0;
    result_mtx[2][3] = 0;
    result_mtx[3][0] = 0;
    result_mtx[3][1] = 0;
    result_mtx[3][2] = 0;
    result_mtx[3][3] = 1;
  } else if( normal.y > Constants::epsilon ) {
    result_mtx[0][0] = 1 / (max.x - min.x);
    result_mtx[0][1] = 0;
    result_mtx[0][2] = 0;
    result_mtx[0][3] = 0;
    result_mtx[1][0] = 0;
    result_mtx[1][1] = 0;
    result_mtx[1][2] = 1 / (min.z - max.z);
    result_mtx[1][3] = 0;
    result_mtx[2][0] = 0;
    result_mtx[2][1] = -result_mtx[0][0] * result_mtx[1][2];
    result_mtx[2][2] = 0;
    result_mtx[2][3] = 0;
    result_mtx[3][0] = 0;
    result_mtx[3][1] = 0;
    result_mtx[3][2] = 0;
    result_mtx[3][3] = 1;
  } else if( normal.y < -Constants::epsilon ) {
    result_mtx[0][0] = 1 / (max.x - min.x);
    result_mtx[0][1] = 0;
    result_mtx[0][2] = 0;
    result_mtx[0][3] = 0;
    result_mtx[1][0] = 0;
    result_mtx[1][1] = 0;
    result_mtx[1][2] = 1 / (max.z - min.z);
    result_mtx[1][3] = 0;
    result_mtx[2][0] = 0;
    result_mtx[2][1] = -result_mtx[0][0] * result_mtx[1][2];
    result_mtx[2][2] = 0;
    result_mtx[2][3] = 0;
    result_mtx[3][0] = 0;
    result_mtx[3][1] = 0;
    result_mtx[3][2] = 0;
    result_mtx[3][3] = 1;
  }
  return;
}

void AABox::closestPoint( const Vec3 &p,
                          Vec3 &closest_point,
                          Vec3 &normal,
                          Vec3 &tex_coord ) {
  PointLocation tmp;
  closestPoint( p, min, max, closest_point, normal, tex_coord, tmp );
}

void AABox::closestPoint( const Vec3 &p,
                          const Vec3 &_min,
                          const Vec3 &_max,
                          Vec3 &closest_point,
                          Vec3 &normal,
                          Vec3 &tex_coord,
                          PointLocation &point_location ) {
  Vec3 _normal;
  point_location = INSIDE;
  for( int i = 0; i < 3; ++i ) {
    HAPIFloat v = p[i];
    if( v < _min[i] ) {
      _normal[i] = v - _min[i];
      v = _min[i];
      point_location = OUTSIDE;
    } else if( v > _max[i] ) {
      _normal[i] = v - _max[i];
      v = _max[i];
      point_location = OUTSIDE;
    }
    closest_point[i] = v;
  }
  if( point_location == INSIDE ) {
    int closest_i = -1;
    HAPIFloat smallest_distance = std::numeric_limits<HAPIFloat>::max();
    bool on_surface = false;
    bool max_value = false;
    for( int i = 0; i < 3; ++i ) {
      HAPIFloat v = p[i];
      HAPIFloat v_min = v - _min[i];
      HAPIFloat v_max = _max[i] - v;
      if( v_min < v_max ) {
        if( v_min < Constants::epsilon ) {
          smallest_distance = 0;
          closest_i = i;
          _normal[i] = -1;
          if( !on_surface ) {
            on_surface = true;
            for( int j = 0; j < i; ++j )
              _normal[j] = 0;
          }
        } else if( closest_i == -1 || v_min < smallest_distance ) {
          smallest_distance = v_min;
          closest_i = i;
          _normal[i] = 1;
          max_value = false;
        }
      } else {
        if( v_max < Constants::epsilon ) {
          smallest_distance = 0;
          closest_i = i;
          _normal[i] = 1;
          if( !on_surface ) {
            on_surface = true;
            for( int j = 0; j < i; ++j )
              _normal[j] = 0;
          }
        } else if( closest_i == -1 || v_max < smallest_distance ) {
          smallest_distance = v_max;
          closest_i = i;
          _normal[i] = -1;
          max_value = true;
        }
      }

      if( on_surface ) {
        point_location = ON_SURFACE;
        normal = _normal;
        normal.normalize();
        closest_point = p;
      } else if( closest_i == i ) {
        for( int j = 0; j < 3; ++j ) {
          if( j != closest_i )
            _normal[j] = 0;
        }
        normal = _normal;
        closest_point = p;
        if( max_value ) {
          closest_point[closest_i] = _max[closest_i];
        } else
          closest_point[closest_i] = _min[closest_i];
      }
    }
  } else {
    normal = _normal;
    normal.normalize();
  }
  tex_coord = getTextureCoordinate( closest_point, normal );
}

bool AABox::lineIntersect( const Vec3 &from, 
                           const Vec3 &to,
                           IntersectionInfo &result,
                           FaceType face ) {
  PointLocation from_location;
  return lineIntersect( from, to, min, max, result, from_location, face );
}

// Page 181 in Real time collision detection, with corrections according
// to errata on book home page.
// Added checks for detection of line segement with AABox surface when
// line segment starts inside AABox.
bool AABox::lineIntersect( const Vec3 &from, 
                           const Vec3 &to,
                           const Vec3 &_min,
                           const Vec3 &_max,
                           IntersectionInfo &result,
                           PointLocation &from_location,
                           FaceType face ) {
  HAPIFloat tmin = 0.0f;
  Vec3 d = to - from;
  Vec3 normal;
  FaceType return_face = face == BACK ? BACK : FRONT;
  if( !CollisionInternals::findTMin( tmin, normal, d, _min, _max, from, to ) )
    return false;

  from_location = OUTSIDE;
  if( H3DUtil::H3DAbs( tmin ) < H3DUtil::Constants::f_epsilon ) {
    Vec3 closest_point, tmp_normal, tex_coord;
    PointLocation point_location;
    closestPoint( from, _min, _max,
                  closest_point, tmp_normal, tex_coord, point_location );
    from_location = point_location;
    if( point_location == INSIDE ) {
      if( face == FRONT )
        return false;
      closestPoint( to, _min, _max, 
                    closest_point, tmp_normal, tex_coord, point_location );
      if( point_location == INSIDE )
        return false;

      return_face = BACK;
      if( point_location == ON_SURFACE ) {
        tmin = 1;
        normal = tmp_normal;
      } else {
        tmin = 0.0f;
        normal = Vec3();
        if( !CollisionInternals::findTMin( tmin, normal, -d, 
                                           _min, _max, to, from ) )
          return false;
        tmin = 1 - tmin;

      }
    } else if( point_location == ON_SURFACE ) {
      normal = tmp_normal;
    }
  }
  // Segment intersects all 3 slabs. Return point (q) and
  // intersection t value( tmin );
  result.face = return_face;
  result.intersection = true;
  result.normal = normal;
  result.point = from + d * tmin;
  result.primitive = this;
  result.t = tmin;
  result.tex_coord = getTextureCoordinate( result.point, normal );
  return true;
}

bool AABox::movingSphereIntersect( HAPIFloat radius,
                                   const Vec3 &from, 
                                   const Vec3 &to ) {
  // More a rough estimate instead of exactly correct.
  Vec3 radius_vec( radius, radius, radius );
  IntersectionInfo result;
  FaceType face = FRONT_AND_BACK;
  PointLocation from_location;
  return lineIntersect( from, to, min - radius_vec,
                        max + radius_vec, result, from_location, face );
}

bool AABox::movingSphereIntersect( HAPIFloat radius,
                                   const Vec3 &from, 
                                   const Vec3 &to,
                                   IntersectionInfo &result) {
  Vec3 closest_point, normal, tex_coord;
  PointLocation point_location;
  closestPoint( from, min, max, closest_point,
                normal, tex_coord, point_location );
  if( point_location != OUTSIDE ||
    ( closest_point - from ).lengthSqr() <= radius * radius ) {
      if( point_location == INSIDE )
        result.face = BACK;
      else
        result.face = FRONT;
      result.intersection = true;
      result.normal = normal;
      result.point = closest_point;
      result.primitive = this;
      result.t = 0;
      result.tex_coord = tex_coord;
      return true;
  }
  Vec3 radius_vec( radius, radius, radius );
  // Intersect segment against expanded AAB. Exit with no intersection if
  // segment misses, else use the result in later calculations.
  FaceType face = FRONT;
  PointLocation from_location;
  if( !lineIntersect( from, to, min - radius_vec,
                      max + radius_vec, result, from_location, face ) ) {
    if( from_location == INSIDE ) {
      result.t = 0;
      result.point = from;
    } else return false;
  }

  // Compute which min and max faces of the box intersection point
  // result.point lies outside of. Note, u and v cannot have the same bits
  // set and then must have at least one bit set among them.
  int u = 0, v = 0;
  if( result.point.x < min.x ) u |= 1;
  if( result.point.x > max.x ) v |= 1;
  if( result.point.y < min.y ) u |= 2;
  if( result.point.y > max.y ) v |= 2;
  if( result.point.z < min.z ) u |= 4;
  if( result.point.z > max.z ) v |= 4;

  // 'Or' all set bits together into a bit mask 
  // ( note: here u + v == u | v )
  int m = u + v;

  // if all 3 bits set (m == 7) then result.point is in a vertex region.
  if( m == 7 ) {
    // Must now intersect segment against the capsules of the three
    // edges meeting at vertex and return the best time. If one or
    // more hit.
    HAPIFloat tmin = std::numeric_limits<HAPIFloat>::max();
    HAPIFloat t;
    Vec3 the_corner = corner( v );
    if( CollisionInternals::intersectSegmentCapsule( from, to, the_corner,
                                 corner( v ^ 1 ), radius, t ) ) {
      tmin = H3DUtil::H3DMin( t, tmin );
    }
    if( CollisionInternals::intersectSegmentCapsule( from, to, the_corner,
                                 corner( v ^ 2 ), radius, t ) ) {
      tmin = H3DUtil::H3DMin( t, tmin );
    }
    if( CollisionInternals::intersectSegmentCapsule( from, to, the_corner,
                                 corner( v ^ 4 ), radius, t ) ) {
      tmin = H3DUtil::H3DMin( t, tmin );
    }

    if( tmin == std::numeric_limits<HAPIFloat>::max() ) return false;
    result.face = FRONT;
    result.intersection = true;
    result.point = the_corner;
    result.normal = from + tmin * (to - from);
    result.normal = result.normal - result.point;
    result.normal.normalize();
    result.tex_coord = getTextureCoordinate( result.point, result.normal );
    result.primitive = this;
    result.t = tmin;
    return true;
  }

  // If only one bit set in m, then result.point is in a face region.
  if( (m & (m-1)) == 0 ) {
    // Do nothing. Time t from intersection with expanded box is
    // correct intersection time. We need to change the result.point
    // though. Should only be one of the dimensions to change
    // since we already is in a face region.
    for( int i = 0; i < 3; ++i ) {
      if( result.point[i] < min[i] ) {
        result.point[i] += radius;
        break;
      } else if( result.point[i] > max[i] ) {
        result.point[i] -= radius;
        break;
      }
    }
    result.tex_coord = getTextureCoordinate( result.point, result.normal );
    return true;
  }

  // result.point is in an edge region. Intersect against the capsule at
  // the edge.
  HAPIFloat t;
  if( CollisionInternals::intersectSegmentCapsule( from, to, corner( u ^ 7 ),
                                                   corner( v ), radius, t ) ) {
    result.t = t;
    result.face = FRONT;
    result.intersection = true;
    result.primitive = this;
    LineSegment line_seg( corner( u ^ 7 ), corner( v ) );
    line_seg.closestPoint( from + (to - from) * t, result.point,
                           result.normal, result.tex_coord );
    result.tex_coord = getTextureCoordinate( result.point, result.normal );
    return true;
  }
  return false;
}

void AABox::render() {
#ifdef HAVE_OPENGL
  glBegin( GL_QUADS );
  //+z
  glNormal3f( 0.f, 0.f, 1.0f );
  glVertex3f( (GLfloat)max.x, (GLfloat)max.y, (GLfloat)max.z );
  glVertex3f( (GLfloat)min.x, (GLfloat)max.y, (GLfloat)max.z );
  glVertex3f( (GLfloat)min.x, (GLfloat)min.y, (GLfloat)max.z );
  glVertex3f( (GLfloat)max.x, (GLfloat)min.y, (GLfloat)max.z );

  // -z
  glNormal3f( 0.f, 0.f, -1.0f );
  glVertex3f( (GLfloat)max.x, (GLfloat)max.y, (GLfloat)min.z );
  glVertex3f( (GLfloat)max.x, (GLfloat)min.y, (GLfloat)min.z );
  glVertex3f( (GLfloat)min.x, (GLfloat)min.y, (GLfloat)min.z );
  glVertex3f( (GLfloat)min.x, (GLfloat)max.y, (GLfloat)min.z );

  // +x
  glNormal3f( 1.0f, 0.f, 0.f );
  glVertex3f( (GLfloat)max.x, (GLfloat)max.y, (GLfloat)max.z );
  glVertex3f( (GLfloat)max.x, (GLfloat)min.y, (GLfloat)max.z );
  glVertex3f( (GLfloat)max.x, (GLfloat)min.y, (GLfloat)min.z );
  glVertex3f( (GLfloat)max.x, (GLfloat)max.y, (GLfloat)min.z );

  // -x
  glNormal3f( -1.0f, 0.f, 0.f );
  glVertex3f( (GLfloat)min.x, (GLfloat)max.y, (GLfloat)max.z );
  glVertex3f( (GLfloat)min.x, (GLfloat)max.y, (GLfloat)min.z );
  glVertex3f( (GLfloat)min.x, (GLfloat)min.y, (GLfloat)min.z );
  glVertex3f( (GLfloat)min.x, (GLfloat)min.y, (GLfloat)max.z );

  // +y
  glNormal3f( 0.f, 1.0f, 0.f );
  glVertex3f( (GLfloat)max.x, (GLfloat)max.y, (GLfloat)max.z );
  glVertex3f( (GLfloat)max.x, (GLfloat)max.y, (GLfloat)min.z );
  glVertex3f( (GLfloat)min.x, (GLfloat)max.y, (GLfloat)min.z );
  glVertex3f( (GLfloat)min.x, (GLfloat)max.y, (GLfloat)max.z );

  // -y
  glNormal3f( 0.f, -1.0f, 0.f );
  glVertex3f( (GLfloat)max.x, (GLfloat)min.y, (GLfloat)max.z );
  glVertex3f( (GLfloat)min.x, (GLfloat)min.y, (GLfloat)max.z );
  glVertex3f( (GLfloat)min.x, (GLfloat)min.y, (GLfloat)min.z );
  glVertex3f( (GLfloat)max.x, (GLfloat)min.y, (GLfloat)min.z );

  glEnd();
#endif
}

void AABox::getConstraints( const Vec3 &point,
                            Constraints &constraints,
                            FaceType face,
                            HAPIFloat radius ) {
  Vec3 closest_point, normal, tex_coord;
  PointLocation point_location;
  closestPoint( point, min, max,
                closest_point, normal, tex_coord, point_location );
  if( radius < 0 || (closest_point - point).lengthSqr() < radius * radius ) {
    if( ( face != FRONT && point_location == INSIDE ) ||
      ( face == BACK && point_location == ON_SURFACE ) ) {
        for( int i = 0; i < 3; ++i ) {
          if( point[i] - min[i] <= radius ) {
            Vec3 closest = point;
            closest[i] = min[i];
            Vec3 tmp_normal;
            tmp_normal[i] = 1;
            Vec3 tmp_tex_coord;
            constraints.push_back( PlaneConstraint( closest, tmp_normal, 
                                                    tmp_tex_coord, 0, this ) );
          }
          if( max[i] - point[i] <= radius ) {
            Vec3 closest = point;
            closest[i] = max[i];
            Vec3 tmp_normal;
            tmp_normal[i] = -1;
            Vec3 tmp_tex_coord;
            constraints.push_back( PlaneConstraint( closest, tmp_normal,
                                                    tmp_tex_coord, 0, this ) );
          }
        }
    } else if( face != BACK && point_location != INSIDE ) {
      constraints.push_back( PlaneConstraint( closest_point, normal,
                                              tex_coord, 0, this ) );
    }
  }
}

#ifdef MACOSX
#ifdef __clang__
#pragma clang diagnostic pop
#endif
#endif
