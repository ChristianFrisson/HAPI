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
/// \file CollisionStructures.cpp
/// \brief Cpp file for CollisionStructures
///
//
//////////////////////////////////////////////////////////////////////////////

#include "CollisionObjects.h"
#include <stack>

using namespace HAPI;
using namespace Bounds;

bool PlaneConstraint::lineIntersect( const Vec3 &from, 
                                     const Vec3 &to,
                                     Bounds::IntersectionInfo &result ) {
      Vec3 from_to = to - from;
      if( normal * from_to > Constants::epsilon ) { 
        /*if( result.normal.z == 0.5 ) {
          cerr << "a" << endl;
          cerr << normal * from_to << endl;
          }*/
        return false;
      }


      HAPIFloat denom = normal * from_to;
      if( denom * denom < Constants::epsilon ) {
        /*        if( result.normal.z == 0.5 )
                  cerr << "b" << endl; */
        return false;
      }
      HAPIFloat u = ( normal * ( point - from ) ) / denom; 
      /*      if( result.normal.z == 0.5 )
              cerr << u << endl;*/
      if( u <= 1 + Constants::epsilon && u >= 0 - Constants::epsilon ) {

        if( u < 0 ) u = 0;
        if( u > 1 ) u = 1;
                //cerr << u << endl;
        result.point = from + u * from_to;
        result.normal = normal;
        return true;
      }
      return false;
    }


void AABoxBound::render() {
  glDisable( GL_LIGHTING );
  if( collided )
    glColor3d( 1, 0, 0 );
  else
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
}

void AABoxBound::fitAroundPoints( const vector< Vec3 > &points ) {
  if( points.size() == 0 ) return;
  min = points[0];
  max = points[0];
  
  for(unsigned int i=1 ; i<points.size() ; i++) {
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

  //	bias with an epsilon to increase robustness when segment is parallel 
  // to a box plane...
	HAPIFloat epsilon = (e*e)/100000;
	adx += epsilon;
	ady += epsilon;
	adz += epsilon;

	if (H3DUtil::H3DAbs(m.y*d.z - m.z*d.y) > e.y*adz + e.z*ady) return false;
	if (H3DUtil::H3DAbs(m.z*d.x - m.x*d.z) > e.x*adz + e.z*adx) return false;
	if (H3DUtil::H3DAbs(m.x*d.y - m.y*d.x) > e.x*ady + e.y*adx) return false;

	return true;
}

Vec3 AABoxBound::closestPoint( const Vec3 &p ) {
  assert( false );
  return Vec3();
}

bool AABoxBound::insideBound( const Vec3 &p ) {
  return ( p.x <= max.x &&
           p.x >= min.x &&
           p.y <= max.y &&
           p.y >= min.y &&
           p.z <= max.z &&
           p.z >= min.z );
}

Vec3 SphereBound::closestPoint( const Vec3 &p ) {
  Vec3 dir = p - center;
  if( dir * dir < Constants::epsilon )
    return center + Vec3( radius, 0, 0 ); 
  else {
    dir.normalize();
    return center + dir * radius;
  }
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

bool SphereBound::lineIntersect( const Vec3 &from, 
                                 const Vec3 &to,
                                 IntersectionInfo &result ) {
  Vec3 start_point = from - center;
  Vec3 end_point = to - center;
  bool flip_normal = false;

  // p is the starting point of the ray used for determinining intersection
  // v is the vector between this starting point and the end point.
  // the starting point must be outside the sphere.
  Vec3 p, v;
  HAPIFloat r2 = radius * radius;

  HAPIFloat a0  = start_point * start_point - r2;
  if (a0 <= 0) {
    // start_point is inside sphere
    a0 = end_point * end_point - r2;
    if( a0 <= 0 ) {
      // both start_point and end_point are inside sphere so no intersection
      return false;
    } else {
      // start_point is inside and end_point is outside. We will have an 
      // intersection.
      p = end_point;
      v = start_point - end_point;
      flip_normal = true;
    }
  } else {
    // start_point is outside sphere
    p = start_point;
    v = end_point - start_point;
    
    // check that the line will intersect 
    HAPIFloat a1 = v * p;
    if (a1 >= 0) {
      // v is pointing away from the sphere so no intersection
      return false;
    }
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
  result.normal.normalize();
  if( flip_normal ) 
    result.normal = -result.normal;
  return true;
}

void SphereBound::render( ) {
  if( !gl_quadric ) gl_quadric = gluNewQuadric();
  glDisable( GL_LIGHTING );
  glMatrixMode( GL_MODELVIEW );
  glPushMatrix();
  glTranslated( center.x, center.y, center.z );
  if( collided )
    glColor3d( 1, 0, 0 );
  else
    glColor3d( 1, 1, 0 );
  glPolygonMode( GL_FRONT_AND_BACK, GL_LINE );
  gluSphere( gl_quadric, radius, 10, 10 );
  glPopMatrix();
  glPolygonMode( GL_FRONT_AND_BACK, GL_FILL );
  glEnable( GL_LIGHTING );
}

void SphereBound::fitAroundPoints( const vector< Vec3 > &points ) {
	AABoxBound box;
  box.fitAroundPoints( points );
	Vec3 c = 0.5 * (box.min + box.max);
	HAPIFloat r = 0;
	
  //	find largest square distance
	for(unsigned int i=0 ; i<points.size() ; i++) {
    Vec3 from_center = points[i]-c;
    HAPIFloat d = from_center * from_center;
    if (d > r) r = d;
  }
	
  center = c;
  radius = H3DUtil::H3DSqrt( r );
}

struct StackElement {
  BinaryBoundTree *tree;
  vector< Triangle > triangles;
}; 

BinaryBoundTree::BinaryBoundTree( BoundNewFunc func, 
                                  const vector< Triangle > &triangle_vector ):
  new_func( func ), left( NULL ), right( NULL ) {
  
  std::stack< StackElement > stack;
	
  if( triangle_vector.size() == 0 ) return;
 
  //	add the starting subtree to stack...
	StackElement start;
	start.tree = this;
	start.triangles = triangle_vector;
	stack.push( start );
  //cerr << start.triangles.size() << endl;
  //	BUILD TREE
	while( !stack.empty() ) {
		const std::vector< Triangle > &stack_triangles = stack.top().triangles;
		BinaryBoundTree* stack_tree = stack.top().tree;

	  if (stack_triangles.size() == 1 ) {
      //	build a leaf
			stack_tree->triangles = stack_triangles;
      		stack.pop();
			continue;
		}

		std::vector<Vec3 > points;
		points.resize( stack_triangles.size() * 3 );
		for (unsigned int i=0 ; i<stack_triangles.size() ; i++ ) {
      points[i*3]   = stack_triangles[i].a;
      points[i*3+1] = stack_triangles[i].b;
      points[i*3+2] = stack_triangles[i].c;
    }
    
    //	build bounding shape
		stack_tree->bound.reset( new_func() );
    stack_tree->bound->fitAroundPoints( points );
		
    //	DIVIDE SUBSET AND RECURSE
    //	longest axis
		Vec3 axis = stack_tree->bound->longestAxis();

    //	middle point of current set
		Vec3 mid(0,0,0);

    for(unsigned int i = 0 ; i < stack_triangles.size() ; i++) {
      mid = mid + stack_triangles[i].pointRepresentation();
    }

		mid = (1.0f/stack_triangles.size()) * mid;	//divide by N
    
    //	build subsets based on axis/middle point
		std::vector<Triangle> left;
		std::vector<Triangle> right;
    
		for( unsigned int i = 0 ; i<stack_triangles.size() ; i++ ) {
      Vec3 mid_to_point = stack_triangles[i].pointRepresentation() - mid;
      if ( mid_to_point * axis < 0 )
				left.push_back(stack_triangles[i]);
			else
				right.push_back(stack_triangles[i]);
    }
		
    //	anity check... sometimes current subset cannot be divided by longest 
    // axis: change axis or just half
		if ( (left.size() == 0) || (right.size() == 0) ) {
			left.clear();
			right.clear();
			for (unsigned int i = 0 ; i<stack_triangles.size()/2 ; i++) 
        left.push_back(stack_triangles[i]);
			for (unsigned int i = stack_triangles.size()/2 ; i<stack_triangles.size() ; i++) 
        right.push_back(stack_triangles[i]);
		}
		
		stack.pop();

    //	do recurse
		if ( left.size() != 0) {
      stack_tree->left.reset( new BinaryBoundTree );
			
			StackElement element;
			element.tree = stack_tree->left.get();
			element.triangles = left;
			stack.push( element );
		}
		
		if ( right.size() != 0) {
			stack_tree->right.reset( new BinaryBoundTree );
			
			StackElement element;
			element.tree = stack_tree->right.get();
			element.triangles = right;
			stack.push( element );
		}
	}
} 

void BinaryBoundTree::render() {
  render( 0 );
}

void Triangle::render() {
//  if( !collided ) return;
  glDisable( GL_LIGHTING );
  if( collided )
    glColor3d( 1, 0, 0 );
  else {
    glColor3d( 0, 0, 1 );
  }

  glBegin( GL_LINE_STRIP );
  glVertex3d( a.x, a.y, a.z );
  glVertex3d( b.x, b.y, b.z );
  glVertex3d( c.x, c.y, c.z );
  glVertex3d( a.x, a.y, a.z );
  glEnd();
  glEnable( GL_LIGHTING );
}

void BinaryBoundTree::render( int depth) {
  if( !isLeaf() ) {
    if( depth == 0 ) {
      bound->render();
    } else {
      if( left.get() ) left->render( depth - 1 );
      if( right.get() ) right->render( depth - 1 );
    }
  } else {
    for( unsigned int i = 0; i < triangles.size(); i++ ) { 
      triangles[i].render();
    }
  }
}

Vec3 Triangle::closestPoint( const Vec3 &p ) {
 
  Vec3 ab = Vec3(b)-Vec3(a);
  Vec3 ac = Vec3(c)-Vec3(a);
  Vec3 ap = Vec3(p)-Vec3(a) ;

  HAPIFloat d1 = ab * ap;
  HAPIFloat d2 = ac * ap;
  if( d1 <= Constants::epsilon && d2 <= Constants::epsilon ) {
    //cerr << "1" << endl;
    return a;
  }

  Vec3 bp = p - b;
  HAPIFloat d3 = ab * bp;
  HAPIFloat d4 = ac * bp;

  if( d3 >= -Constants::epsilon && d4 <= d3 ) {
    //    cerr << "2" << endl;
    return b;
  }
  
  HAPIFloat vc = d1*d4 - d3*d2;
  if( vc <= Constants::epsilon && 
      d1 >= -Constants::epsilon && 
      d3 <= Constants::epsilon ) {
    // cerr << "3" << endl;
    HAPIFloat v = d1 / ( d1 - d3 );
    return Vec3(a) + v * ab;
  }

  Vec3 cp = p - c;
  HAPIFloat d5 = ab * cp;
  HAPIFloat d6 = ac * cp;
  if( d6 >= -Constants::epsilon && d5 <= d6 ) {
    //cerr << "4" << endl;
    return c;
  }

  HAPIFloat vb = d5*d2 - d1*d6;
  if( vb <= Constants::epsilon && 
      d2 >= -Constants::epsilon &&
      d6 <= Constants::epsilon ) {
    //cerr << "5" << endl;
    HAPIFloat w = d2 / ( d2 - d6 );
    return a + w * ac;
  }
  
  HAPIFloat va = d3*d6 - d5*d4;
  if( va <= Constants::epsilon &&
      (d4-d3) >= -Constants::epsilon &&
      (d5-d6) >= -Constants::epsilon ) {
    //cerr << "6" << endl;
    HAPIFloat w = (d4-d3) /((d4 -d3) + (d5 - d6 ) );
    return Vec3(b) + w * (Vec3(c)-Vec3(b));
  }

  //cerr << "7" << endl;
  HAPIFloat denom = 1 / ( va + vb + vc );
  HAPIFloat v = vb * denom;
  HAPIFloat w = vc * denom;
  return Vec3(a) + ab * v + ac * w;
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
  HAPIFloat ab_length2 = ab.lengthSqr();

  HAPIFloat ca_t = (cp * ca) / ca_length2;
  HAPIFloat cb_t = (cp * cb) / cb_length2;
  HAPIFloat ab_t = (ap * ab) / ab_length2;

  if( ca_t < 0 && cb_t < 0 )
    return c;

  if( cb_t > 1 && ab_t > 1 )
    return b;

  if( ca_t > 1 && ab_t < 0 )
    return a;
  
  bool discard_ca = ca_t < 0 || ca_t > 1;
  bool discard_cb = cb_t < 0 || cb_t > 1;;
  bool discard_ab = ab_t < 0 || ab_t > 1;;

  Vec3 ca_p = c + ca * ca_t;
  Vec3 cb_p = c + cb * cb_t;
  Vec3 ab_p = a + ab * ab_t;
  
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

  collided = true;
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
                              IntersectionInfo &result ) {
  
  if( q.z < 0 )
    HAPIFloat a = 4;//cerr << "F" << endl;
  
  Vec3 v0 = a;
  Vec3 v1 = b;
  Vec3 v2 = c;
 
  Vec3 ab = v1 - v0;
  Vec3 ac = v2 - v0;
  Vec3 qp = p - q;

  // Compute normal.
  Vec3 n = ab % ac;

  HAPIFloat epsilon = 1e-5;

  HAPIFloat d = qp * n;
  if ( d < 0 ) {
    d = -d;
    n = -n;
    Vec3 tmp = v1;
    v1 = v2;
    v2 = tmp;
    tmp = ab;
    ab = ac;
    ac = tmp;
  } else if( d == 0 ) {
    return false;
  }

  Vec3 ap = p - v0;
  HAPIFloat t = ap * n;

  if( t < -epsilon || t > d + epsilon ) return false;

  Vec3 e = qp % ap;
  HAPIFloat v = ac * e;
  if( v < -epsilon || v > d + epsilon ) return false;
  HAPIFloat w = -(ab * e );
  if( w < -epsilon || v + w > d + epsilon) return false;

  HAPIFloat ood = 1 / d;
  t *= ood; 
  v *= ood;
  w *= ood;
  HAPIFloat u = 1 - v - w;

  result.point = u*v0 + v*v1 + w*v2;
  result.normal = n;
  result.normal.normalizeSafe();
  //if( (-qp) * result.normal > 0 ) 
  //  result.normal = -result.normal;
  return true;
}

#endif

void BinaryBoundTree::clearCollidedFlag() {
  if( !isLeaf() ) {
    bound->collided = false;
    left->clearCollidedFlag();
    right->clearCollidedFlag();
  } else {
    for( unsigned int i = 0; i < triangles.size(); i++ )
      triangles[i].collided = false;
  }
}

bool BinaryBoundTree::lineIntersect( const Vec3 &from, 
                                     const Vec3 &to,
                                     IntersectionInfo &result ) {
  if ( isLeaf() )	{
    // TODO: find closest?
    for( unsigned int i = 0; i < triangles.size(); i++ ) {
      Triangle &t = triangles[i];
      if( t.lineIntersect( from, to, result ) )	return true;
		}
		return false;
	}	else 	{
		if ( bound->boundIntersect( from, to ) )	{
			bool overlap = false;
      bound->collided = true;
			
			if (left.get()) overlap |= left->lineIntersect( from, to, result );
			if (right.get()) overlap |= right->lineIntersect( from, to, result );
			
			return overlap;
		}
		else return false;
	}
}

Matrix3 covarianceMatrix( const vector< Vec3 >&points ) {
  // TODO: if no points?

  unsigned int nr_points = points.size();

	Vec3 c = points[0];
	for(unsigned int i=1 ; i < nr_points; i++) 
    c+= points[i];

	c /= points.size();
	
  HAPIFloat e00 = 0, e01 = 0, e02 = 0,
                    e11 = 0, e12 = 0,
                             e22 = 0;
	for (unsigned int i = 0; i < nr_points; i++)	{
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
  // TODO: epsilon??
	if (H3DUtil::H3DAbs(A[p][q]) > 0.0001f) {
		HAPIFloat r = (A[q][q] - A[p][p]) / (2.0f * A[p][q]);
		HAPIFloat t = (r >= 0.0f ? 
                  1.0f / (r + H3DUtil::H3DSqrt(1.0f + r*r)) : 
                  -1.0f / (-r + H3DUtil::H3DSqrt(1.0f + r*r)) );
		
		c = 1.0f / H3DUtil::H3DSqrt(1.0f + t*t);
		s = t * c;
	}	else{
		c = 1.0f;
		s = 0.0f;
	}
}

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
	for (n = 0; n < maxIterations; n++)	{
		p = 0; q = 1;
		for ( i = 0; i < 3; i++) for (j = 0; j < 3; j++ ) {
			if (i == j) continue;
			if (H3DUtil::H3DAbs(a[i][j]) > H3DUtil::H3DAbs(a[p][q])) {p = i; q = j;}
		}

		symmetricSchurDecomposition(a, p, q, c, s);
		for (i = 0; i < 3; i++) {
			J[i][0] = J[i][1] = J[i][2] = 0.0f;
			J[i][i] = 1.0f;
		}
		J[p][p] =  c; J[p][q] = s;
		J[q][p] = -s; J[q][q] = c;

    // cumulate rotations
		v = v * J;

    // diagonalize
		a = (J.transpose() * a) * J;
    
    //	eval norm
		HAPIFloat off = 0.0;
		for (i = 0; i < 3; i++) for (j = 0; j < 3; j++)	{
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
	
	
  //	guess an oriented reference frame
	unsigned int length_order[3];
  //	initialize
	for(unsigned int i=0 ; i<3 ; i++) length_order[i] = i;
	
  //	do a bubble sort on indices
	for( unsigned int i=0 ; i<2 ; i++) {
    for(unsigned int j=i+1 ; j<3 ; j++)	{
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
		
  //	build box in the new frame
  min = frame * points[0];
	max = min;
	for(unsigned int i=1 ; i<points.size() ; i++)	{
		Vec3 pt = frame * points[i];
		
		if (pt.x < min.x) min.x = pt.x;
		if (pt.y < min.y) min.y = pt.y;
		if (pt.z < min.z) min.z = pt.z;
		
		if (pt.x > max.x) max.x = pt.x;
		if (pt.y > max.y) max.y = pt.y;
		if (pt.z > max.z) max.z = pt.z;
	}
	
  //	fill oriented box
  orientation = Rotation( frame );
  
  cerr << frame * (0.5f*(min+max)) << endl;
	center = frame.transpose() * (0.5f*(min+max)) ;
  halfsize = 0.5f*(max-min);
}

bool OrientedBoxBound::boundIntersect( const Vec3 &from, 
                                       const Vec3 &to ) {
  Vec3 rp = orientation * from;
	Vec3 rq = orientation * to;
	
	return AABoxBound::boundIntersect( rp, rq );
}

Vec3 OrientedBoxBound::closestPoint( const Vec3 &p ) {
  assert( false );
  return Vec3();
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

void OrientedBoxBound::render( ) {
  Vec3 center = -orientation * ((max + min)/2);
  glMatrixMode( GL_MODELVIEW );
  glPushMatrix();
  glDisable( GL_LIGHTING );
  if( collided )
    glColor3d( 1, 0, 0 );
  else
    glColor3d( 1, 1, 0 );
  glTranslated( center.x, center.y, center.z );
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
}

void GeometryPrimitive::getConstraints( const Vec3 &point,
                           std::vector< PlaneConstraint > &constraints ) {
  Vec3 closest_point = closestPoint( point );
  //cerr << closest_point << endl;
  Vec3 normal = point - closest_point;
  HAPIFloat n = normal.length();
  //cerr << normal.length() << endl;
  normal.normalizeSafe();
  //cerr << closest_point << endl;
  constraints.push_back( PlaneConstraint( closest_point, normal ) );
}

void Triangle::getConstraints( const Vec3 &point,
                                        const Matrix4 &matrix,
                                        std::vector< PlaneConstraint > &constraints ) {
  Vec3 oa = a;
  Vec3 ob = b;
  Vec3 oc = c;
  a = (matrix * a);
  b = (matrix * b);
  c = (matrix * c);
  GeometryPrimitive::getConstraints( point, constraints );
  a = oa;
  b = ob;
  c = oc;
}


void BinaryBoundTree::getConstraints( const Vec3 &point,
                                      std::vector< PlaneConstraint > &constraints ) {
  if ( isLeaf() )	{
    for( unsigned int i = 0; i < triangles.size(); i++ ) {
      Triangle &t = triangles[i];
      t.getConstraints( point, constraints );
		}
	}	else 	{
		//if ( bound->boundIntersect( from, to ) )	{
    if (left.get()) left->getConstraints( point, constraints );
    if (right.get()) right->getConstraints( point, constraints );
    
  }
}

void BinaryBoundTree::getConstraints( const Vec3 &point,
                                      const Matrix4 &matrix,
                                      std::vector< PlaneConstraint > &constraints ) {
  if ( isLeaf() )	{
    for( unsigned int i = 0; i < triangles.size(); i++ ) {
      Triangle &t = triangles[i];
      t.getConstraints( point, matrix, constraints );
		}
	}	else 	{
		//if ( bound->boundIntersect( from, to ) )	{
    if (left.get()) left->getConstraints( point, matrix, constraints );
    if (right.get()) right->getConstraints( point, matrix, constraints );
    
  }
}


void BinaryBoundTree::getTrianglesWithinRadius( const Vec3 &p,
                                                HAPIFloat radius,
                                                vector< Triangle > &result ) {
  HAPIFloat r2 = radius * radius;
  if ( isLeaf() )	{
    for( vector< Triangle >::iterator i = triangles.begin();
         i != triangles.end(); i++ ) {
      Vec3 cp = (*i).closestPoint( p );
      if( (cp - p).lengthSqr() <= r2 )
        result.push_back( *i );
    }
    //cerr << "!: " << triangles.size() << endl;
           //result.insert( result.end(), triangles.begin(), triangles.end() );
	}	else 	{
    Vec3 cp = bound->boundClosestPoint( p );
    if( (cp - p).lengthSqr() > r2 && !bound->insideBound(p) ) return;

    if (left.get()) left->getTrianglesWithinRadius( p, radius, result );
    if (right.get()) right->getTrianglesWithinRadius( p, radius, result );
  }
}

void LineSegment::render() {
  glDisable( GL_LIGHTING );
  if( collided )
    glColor3d( 1, 0, 0 );
  else {
    glColor3d( 0, 0, 1 );
  }

  glBegin( GL_LINES );
  glVertex3d( start.x, start.y, start.z );
  glVertex3d( end.x, end.y, end.z );
  glEnd();
  glEnable( GL_LIGHTING );
}

/// Returns the closest point on the object to the given point p.
Vec3 LineSegment::closestPoint( const Vec3 &p ) {
  Vec3 ab = end - start;
  HAPIFloat ab2 = ab * ab;
  if( ab2 < Constants::epsilon ) {
    return start;
  }
    
  HAPIFloat t = 
    (p - start).dotProduct( ab ) / ab2;

  if( t < 0 ) t = 0;
  if( t > 1 ) t = 1;
  Vec3 closest_point = start + t * ab;
}

/// Detect collision between a line segment and the object.
bool LineSegment::lineIntersect( const Vec3 &from, 
                                 const Vec3 &to,
                                 IntersectionInfo &result ) {
  // TODO: implement
  return false;
}

void LineSegment::getConstraints( const Vec3 &point,
                                  const Matrix4 &matrix,
                                  std::vector< PlaneConstraint > &constraints ) {
  Vec3 oa = start;
  Vec3 ob = end;

  start = (matrix * start);
  end = (matrix * end);

  getConstraints( point, constraints );
  start = oa;
  end = ob;
}

void Point::render() {
  glDisable( GL_LIGHTING );
  if( collided )
    glColor3d( 1, 0, 0 );
  else {
    glColor3d( 0, 0, 1 );
  }

  glBegin( GL_POINTS );
  glVertex3d( position.x, position.y, position.z );
  glEnd();
  glEnable( GL_LIGHTING );
}

/// Detect collision between a line segment and the object.
bool Point::lineIntersect( const Vec3 &from, 
                                 const Vec3 &to,
                                 IntersectionInfo &result ) {
  // TODO: implement
  return false;
}
      

void Point::getConstraints( const Vec3 &point,
                            const Matrix4 &matrix,
                            std::vector< PlaneConstraint > &constraints ) {
  Vec3 oa = position;
  position = (matrix * position);
  getConstraints( point, constraints );
  position = oa;
}
