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
/// \file GodObjectRenderer.cpp
/// \brief cpp file for GodObjectRenderer.
///
//
//////////////////////////////////////////////////////////////////////////////

#include <HAPI/GodObjectRenderer.h>
#include <HAPI/HAPIHapticsDevice.h>
#include <HAPI/PlaneConstraint.h>

#include <H3DUtil/H3DMath.h>

using namespace HAPI;

HAPIHapticsRenderer::HapticsRendererRegistration 
GodObjectRenderer::renderer_registration(
                            "GodObject",
                            &(newInstance< GodObjectRenderer >)
                            );

// the minimum distance the proxy will be from the surface. The proxy
// will be moved above the surface with the given factor in order to
// avoid roundoff errors and fallthrough problems.
const HAPIFloat min_distance = 1e-3;

// epsilon value for deciding if a point is the same
const HAPIFloat length_sqr_point_epsilon = 1e-12; //12

// epsilon value for deciding if a normal is the same.
const HAPIFloat length_sqr_normal_epsilon = 1e-12;

inline bool planeIntersect( Vec3 p1, Vec3 n1, Vec3 p2, Vec3 n2,
                            Vec3 &p, Vec3 &dir ) {
  HAPIFloat d1 = n1 * p1;
  HAPIFloat d2 = n2 * p2;
  
  // Direction of intersection line
  dir = n1 % n2;

  // Check if planes are parallell
  HAPIFloat denom = dir * dir;
  if( denom < Constants::epsilon ) return false;
  
  // Compute point on intersection line.
  p = ( (d1 * n2 - d2 * n1 ) % dir ) / denom;
  return true;
}
  
inline Vec3 projectOntoLine( Vec3 p, 
                             Vec3 point1, 
                             Vec3 point2, 
                             bool segment = false ) {
  Vec3 ab = point2 - point1;

  // Project c onto ab.
  HAPIFloat t = (p - point1) * ab / (ab * ab );

  if( segment ) {
    if( t < 0 ) t = 0;
    if( t > 1 ) t = 1;
  }
  
  return  point1 + t * ab;
} 

inline Vec3 projectOntoPlane( Vec3 p, Vec3 point_on_plane, Vec3 normal ) {
  HAPIFloat t = normal * ( p - point_on_plane );
  return p - t * normal;
}                         


inline Vec3 projectOntoPlaneIntersection( const Vec3 &p, 
                            Vec3 p1,
                            Vec3 n1,
                            Vec3 p2,
                            Vec3 n2 ) {
  Vec3 lp, ld;
  if( !planeIntersect( p1, n1, p2, n2, lp, ld ) ) {
     return projectOntoPlane( p, p1, n1 ); 
  }

  return projectOntoLine( p, lp, lp + ld );
}


void GodObjectRenderer::onOnePlaneContact( 
                            const PlaneConstraint &c, 
                            HAPISurfaceObject::ContactInfo &contact,
                            const HapticShapeVector &shapes ) {
  
  // create a local coordinate system with the PlaneConstraint normal
  // as y-axis
  contact.y_axis = c.normal;
  Vec3 a( contact.y_axis.z, contact.y_axis.x, contact.y_axis.y );
  contact.x_axis = contact.y_axis % a;
  contact.z_axis = contact.x_axis % contact.y_axis;
  contact.x_axis.normalizeSafe();
  contact.z_axis.normalizeSafe();
  contact.setGlobalOrigin( contact.contact_point_global );
  assert( c.haptic_shape.get() );

  contact.primitive = c.primitive;
  c.haptic_shape->getSurface()->getProxyMovement( contact );

  Vec3 new_proxy_pos = 
	contact.contact_point_global + 
    contact.y_axis * min_distance +
    contact.proxy_movement_local.x * contact.x_axis + 
    contact.proxy_movement_local.y * contact.z_axis;

  // try to move the proxy from proxy_pos -> new_proxy_pos and check for 
  // intersection with shapes. If intersection the new goal will be 
  // new_proxy_pos projected onto the plane created by intersection point
  // and its normal and the previous constraint planes.
  Vec3 o;
  tryProxyMovement( proxy_position, 
                    new_proxy_pos,
                    1, 
                    c, 
                    shapes, 
                    o );

  contact.setGlobalOrigin( o );
  
  // call the surface to determine forces and proxy movement
  c.haptic_shape->getSurface()->getForces( contact );
  
  // add a contact
  tmp_contacts.push_back( make_pair( c.haptic_shape.get(), contact ) );
}

void GodObjectRenderer::onTwoPlaneContact( 
           const PlaneConstraint &p0,
           const PlaneConstraint &p1,
           HAPISurfaceObject::ContactInfo &contact,
           const HapticShapeVector &shapes ) {
  
  Vec3 contact_global = contact.globalContactPoint();

  // the direction along the intersection of the two planes. 
  Vec3 line_dir = p0.normal % p1.normal;
  line_dir.normalizeSafe();

  // transformation matrix from local coordinate system with the normals
  // of the constraint planes as axis to global space
  Matrix4 m( p0.normal.x, p1.normal.x, line_dir.x, contact_global.x, 
             p0.normal.y, p1.normal.y, line_dir.y, contact_global.y, 
             p0.normal.z, p1.normal.z, line_dir.z, contact_global.z,
             0, 0, 0, 1 );

  // global -> local
  Matrix4 m_inv = m.inverse();
  
  // probe position in local coordinates
  Vec3 local_pos = m_inv * contact.globalProbePosition();

  // check that both planes constrain, if not just do one plane contact
  // calculations
  if( local_pos.x > Constants::epsilon ) {
    onOnePlaneContact( p1, contact, shapes );
  } else if( local_pos.y > Constants::epsilon ) {
    onOnePlaneContact( p0, contact, shapes );
  } else {
    // both planes constrains

    HAPIFloat sum = local_pos.x + local_pos.y;
    
    HAPIFloat weight = 0;
    if( sum < 0 )
      weight = local_pos.x / sum;
    
    // create a local coordinate system with the PlaneConstraint normal
    // as y-axis
    contact.y_axis = p0.normal * local_pos.x + p1.normal * local_pos.y;
    contact.y_axis.normalizeSafe();
    
    Vec3 a( contact.y_axis.z, contact.y_axis.x, contact.y_axis.y );
    contact.x_axis = contact.y_axis % a;
    contact.z_axis = contact.x_axis % contact.y_axis;
    contact.z_axis.normalizeSafe();
    contact.x_axis.normalizeSafe();
    contact.setGlobalOrigin( contact.contact_point_global );
 
    Vec3 line_dir_local = contact.vectorToLocal( line_dir );

    assert( p0.haptic_shape.get() );

    contact.primitive = p0.primitive;
    // calculate the force and proxy movement for the first plane
    p0.haptic_shape->getSurface()->getProxyMovement( contact );
    
    // constrain the proxy movement to the intersection between 
    // the two planes
    HAPIFloat p0_proxy_movement = 
      Vec3( contact.proxy_movement_local.x, 
            0, 
            contact.proxy_movement_local.y ) *
      line_dir_local;

    assert( p1.haptic_shape.get() );
    contact.primitive = p1.primitive;
    // calculate the force and proxy movement for the second plane
    p1.haptic_shape->getSurface()->getProxyMovement( contact );

    // constrain the proxy movement to the intersection between 
    // the two planes
    HAPIFloat p1_proxy_movement = 
      Vec3( contact.proxy_movement_local.x, 
             0, 
             contact.proxy_movement_local.y ) *
      line_dir_local;

    // calculate and set the final output parameters for force and 
    // proxy movement
    Vec3 proxy_movement = 
      H3DUtil::H3DMin( p0_proxy_movement, p1_proxy_movement ) * 
      line_dir_local;

    contact.proxy_movement_local = Vec2( proxy_movement.x, proxy_movement.z ); 
 
    Vec3 new_proxy_pos = 
      contact.contact_point_global + 
      contact.y_axis * min_distance +
      contact.proxy_movement_local.x * contact.x_axis + 
      contact.proxy_movement_local.y * contact.z_axis;

    // try to move the proxy from proxy_pos -> new_proxy_pos and check for 
    // intersection with shapes. If intersection the new goal will be 
    // new_proxy_pos projected onto the plane created by intersection point
    // and its normal and the previous constraint planes.
    Vec3 o;
    tryProxyMovement( proxy_position,
                      new_proxy_pos,
                      2, 
                      p0, 
                      shapes, 
                      o );
    contact.setGlobalOrigin( o );
    
    contact.primitive = p0.primitive;
    p0.haptic_shape->getSurface()->getForces( contact );
    Vec3 p0_force = contact.force_global;
    contact.primitive = p1.primitive;
    p1.haptic_shape->getSurface()->getForces( contact );
    Vec3 p1_force = contact.force_global;

    contact.force_global = p0_force * weight + p1_force * ( 1 - weight );

    // add contacts
    tmp_contacts.push_back( make_pair( p0.haptic_shape.get(), contact ) );
    if( p0.haptic_shape.get() != p1.haptic_shape.get() )
      tmp_contacts.push_back( make_pair( p1.haptic_shape.get(), contact ) );
  }
}


void GodObjectRenderer::onThreeOrMorePlaneContact(  
          vector< PlaneConstraint > &constraints,
          HAPISurfaceObject::ContactInfo &contact,
          const HapticShapeVector &shapes ) {

  assert( constraints.size() >= 3 );

  // find the first three planes that all constrain the proxy
  vector< PlaneConstraint >::iterator i = constraints.begin();
  PlaneConstraint &p0 = (*i++);
  PlaneConstraint &p1 = (*i++);
  PlaneConstraint &p2 = (*i++);

  Vec3 contact_global = contact.globalContactPoint();
  Vec3 probe_local_pos;

    if( i == constraints.end() ) {
    // transformation matrix from local coordinate system with the normals
    // of the constraint planes as axis to global space
    Matrix4 m( p0.normal.x, p1.normal.x, p2.normal.x, contact_global.x, 
               p0.normal.y, p1.normal.y, p2.normal.y, contact_global.y, 
               p0.normal.z, p1.normal.z, p2.normal.z, contact_global.z,
               0, 0, 0, 1 );
    Matrix4 m_inv = m.inverse();
    probe_local_pos = m_inv * contact.globalProbePosition();
  } else {
    while( i != constraints.end() ) {
      // transformation matrix from local coordinate system with the normals
      // of the constraint planes as axis to global space
      Matrix4 m( p0.normal.x, p1.normal.x, p2.normal.x, contact_global.x, 
                 p0.normal.y, p1.normal.y, p2.normal.y, contact_global.y, 
                 p0.normal.z, p1.normal.z, p2.normal.z, contact_global.z,
                 0, 0, 0, 1 );
      Matrix4 m_inv = m.inverse();
      probe_local_pos = m_inv * contact.globalProbePosition();
      
      if( probe_local_pos.x > Constants::epsilon ) {
        // discard p0
        std::swap( p0, *i++ ); 
      } else if( probe_local_pos.y > Constants::epsilon ) {
        // discard p1
        std::swap( p1, *i++ ); 
      } else if( probe_local_pos.z > Constants::epsilon ) {
        // discard p2
        std::swap( p2, *i++ );
      } else {
        break;
      }
    }
  }

  // check that all three planes constrain, if not just do two plane contact
  // calculations
  if( probe_local_pos.x > Constants::epsilon ) {
    onTwoPlaneContact( p1, p2, contact, shapes );
  } else if( probe_local_pos.y > Constants::epsilon ) {
    onTwoPlaneContact( p0, p2, contact, shapes );
  } else if( probe_local_pos.z > Constants::epsilon ) {
    onTwoPlaneContact( p0, p1, contact, shapes );
  } else {

    // find weighting factors between different planes
    HAPIFloat sum = probe_local_pos.x + probe_local_pos.y + probe_local_pos.z;
    
    Vec3 weight;
    if( sum < 0 )
      weight = probe_local_pos / sum;

    // create a local coordinate system with the x-axis pointing towards the
    // proxy
    contact.y_axis = 
      contact.globalContactPoint() - contact.globalProbePosition();
    contact.y_axis.normalizeSafe();

    Vec3 a( contact.y_axis.z, contact.y_axis.x, contact.y_axis.y );
    contact.x_axis = contact.y_axis % a;
    contact.z_axis = contact.x_axis % contact.y_axis;

    contact.x_axis.normalizeSafe();
    contact.z_axis.normalizeSafe();
    contact.setGlobalOrigin( contact.contact_point_global );

    // calculate the force and proxy movement for the first plane
    assert( p0.haptic_shape.get() );
    p0.haptic_shape->getSurface()->getForces( contact );
    Vec3 p0_force = contact.force_global;
    
    // calculate the force and proxy movement for the second plane
    assert( p1.haptic_shape.get() );
    p1.haptic_shape->getSurface()->getForces( contact );
    Vec3 p1_force = contact.force_global;

    // calculate the force and proxy movement for the third plane
    assert( p2.haptic_shape.get() );
    p2.haptic_shape->getSurface()->getForces( contact );
    Vec3 p2_force = contact.force_global;
    
    // proxy is constrained by three planes and cannot move in any 
    // direction
    contact.proxy_movement_local = Vec2( 0, 0 ); 
    
    // calulate final force
    contact.force_global = 
      p0_force * weight.x + 
      p1_force * weight.y + 
      p2_force * weight.z;


    // add contacts
    tmp_contacts.push_back( make_pair( p0.haptic_shape.get(), contact ) );
    if( p0.haptic_shape.get() != p1.haptic_shape.get() ) {
      tmp_contacts.push_back( make_pair( p1.haptic_shape.get(), contact ) );
    }
    
    if( p0.haptic_shape.get() != p2.haptic_shape.get() &&
        p1.haptic_shape.get() != p2.haptic_shape.get() ) {
      tmp_contacts.push_back( make_pair( p2.haptic_shape.get(), contact ) );
    }
  }    
}





HAPIForceEffect::EffectOutput 
GodObjectRenderer::renderHapticsOneStep( HAPIHapticsDevice *hd,
                                         const HapticShapeVector &shapes ) {
  // get the current device values
  HAPIHapticsDevice::DeviceValues input = hd->getDeviceValues();

  // clear all previous contacts
  tmp_contacts.clear();

  Vec3 proxy_pos = proxy_position;
  HAPIForceEffect::EffectOutput output;
  bool has_intersection = false;
    
  HAPIFloat d2;
  Collision::IntersectionInfo closest_intersection;
  
  // the intersections that are closest to the user (several if the 
  // intersections are in the same point)
  vector< PlaneConstraint > closest_constraints;

  // calculate the forces generated by the active haptic shapes
  for( HapticShapeVector::const_iterator i = shapes.begin();
       i != shapes.end();
       i++ ) {
    Collision::IntersectionInfo intersection;
    if( (*i)->lineIntersect( proxy_pos, input.position, intersection,
                             (*i)->getTouchableFace()) ){
      // shape is intersected, create a plane constraint at the
      // intersection point
      PlaneConstraint pc( intersection.point, 
                          intersection.face == Collision::FRONT ?
                          intersection.normal : -intersection.normal,
                          intersection.tex_coord,
                          *i,
                          intersection.primitive );

      if( !has_intersection ) {
        // first shape that was intersected, and therefore also the
        // closest intersection
        closest_intersection = intersection;
        Vec3 v = intersection.point - proxy_pos;
        d2 = v * v;
        has_intersection = true;
        closest_constraints.push_back( pc  );
      } else {
        // this is not the first intersection, check if it closer than
        // previous ones
        Vec3 v = intersection.point - proxy_pos;
        HAPIFloat distance = v * v; 
      
        if( (closest_intersection.point - intersection.point).lengthSqr()
            < length_sqr_point_epsilon ) {
          // intersection point is the same as previous intersections,
          // check if the normal is the same as a previous untersection.
          // if it is the new intersection is discarded.
          bool unique_constraint = true;
          for( vector< PlaneConstraint >::iterator j = 
                 closest_constraints.begin();
               j != closest_constraints.end(); j++ ) {
            if( ( intersection.normal - (*j).normal ).lengthSqr() < 
                length_sqr_normal_epsilon ) {
              // discard new intersectsion
              unique_constraint = false;
            }
          }

          // only add the constraint to the closest_constraints vector
          // if it is an unique constraint.
          if( unique_constraint ) {
            closest_constraints.push_back( pc );
          } 
        } else {
          // intersection point is not the same as previous intersection
          if( distance < d2 ) {
            // new intersection is closer than old intersection
            closest_intersection = intersection;
            d2 = distance;
            closest_constraints.clear();
            closest_constraints.push_back( pc );
          } 
        }
      }
    } 
  }
   
  unsigned int nr_constraints = closest_constraints.size();

  Vec3 new_proxy_pos, new_force;

  HAPISurfaceObject::ContactInfo contact;
  contact.hd = hd;

  // fill in contact information
  contact.contact_point_global = closest_intersection.point;
  contact.probe_position_global = input.position;
  contact.proxy_radius = 0;
  contact.tex_coord = closest_intersection.tex_coord;

  if( nr_constraints == 0 ) {
    // TODO: 0.05??
    new_proxy_pos = proxy_pos + (input.position-proxy_pos)*0.05;
  } else {
    if( nr_constraints == 1 ) {
      onOnePlaneContact( closest_constraints[0], contact, shapes );
    } else if( nr_constraints == 2 ) {
      onTwoPlaneContact( closest_constraints[0],
                         closest_constraints[1], contact, shapes );
    } if( nr_constraints >= 3 ) {
      onThreeOrMorePlaneContact( closest_constraints,
                                 contact, shapes );
    } 

    new_proxy_pos = 
      contact.globalOrigin(); 

    new_force = contact.force_global;
  }
  
  proxy_position = new_proxy_pos;
  output.force = new_force;
    
  contacts_lock.lock();
  contacts.swap( tmp_contacts );
  contacts_lock.unlock();

  return output;
}

bool GodObjectRenderer::tryProxyMovement( Vec3 from, Vec3 to, 
                                          int nr_constraints,
                                          const PlaneConstraint &pc,
                                          const HapticShapeVector &shapes,
                                          Vec3 &point ) {
  // the point from which to move

  Vec3 from_point = from;
  Vec3 to_point = to;

  bool have_new_goal = true;
  bool first_loop = true;
  bool has_intersection;

  point = to;

  HAPIFloat d2;
  Collision::IntersectionInfo closest_intersection;

  while( have_new_goal ) {
    have_new_goal = false;
    
    // find the closest intersection
    has_intersection = false;
    for( HapticShapeVector::const_iterator i = shapes.begin();
         i != shapes.end();
         i++ ) {
      Collision::IntersectionInfo intersection;
      if( (*i)->lineIntersect( from_point, to_point, intersection,
                               (*i)->getTouchableFace()) ){
        if( !has_intersection ) {
          closest_intersection = intersection;
          Vec3 v = intersection.point - from_point;
          d2 = v * v;
          has_intersection = true;
        } else {
          Vec3 v = intersection.point - from_point;
          HAPIFloat distance = v * v; 
          if( distance < d2 ) {
            closest_intersection = intersection;
            d2 = distance;
          } 
        }
      } 
    }
        
    if( has_intersection ) {
      Vec3 normal = 
        closest_intersection.face == Collision::FRONT ?
        closest_intersection.normal : -closest_intersection.normal;
      from_point = closest_intersection.point + normal * min_distance;
      
      if( nr_constraints == 1 && first_loop ) {
        // project onto plane intersection between intersection plane and
        // closest constraint
        from_point =  
          projectOntoPlaneIntersection( from_point, 
                                        closest_intersection.point + 
                                        normal * min_distance,
                                        normal,
                                        pc.point + 
                                        pc.normal * min_distance,
                                        pc.normal );
        to_point = 
          projectOntoPlaneIntersection( to_point, 
                                        closest_intersection.point + 
                                        normal * min_distance,
                                        normal,
                                        pc.point + 
                                        pc.normal * min_distance,
                                        pc.normal );
        have_new_goal = true;
      } else {
        Vec3 move = closest_intersection.point - from_point;
        HAPIFloat l = move.length();
        if( l >= min_distance + 1e-8) {
          move = move * ( (l-min_distance) / l );
          point = from_point + move;
        } else {
          point = from_point;
        }
      }
    } else {
      point = to_point;
      if( first_loop ) return true;
      else return false;
    } 
    first_loop = false;
  }

  return false;
}

