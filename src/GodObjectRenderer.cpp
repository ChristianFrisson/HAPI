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
/// \file GodObjectRenderer.cpp
/// \brief cpp file for GodObjectRenderer.
///
//
//////////////////////////////////////////////////////////////////////////////

#include <HAPI/GodObjectRenderer.h>
#include <HAPI/HAPIHapticsDevice.h>
#include <H3DUtil/H3DMath.h>

using namespace HAPI;
using namespace std;

HAPIHapticsRenderer::HapticsRendererRegistration 
GodObjectRenderer::renderer_registration(
                            "GodObject",
                            &(newInstance< GodObjectRenderer >)
                            );

namespace GodObjectRendererConstants {

  const Vec3 UNINITIALIZED_PROXY_POS = Vec3( -200, -200, -202 );

  // epsilon value for deciding if a point is the same
  const HAPIFloat length_sqr_point_epsilon = 1e-15; //12

  // epsilon value for deciding if a normal is the same.
  const HAPIFloat length_sqr_normal_epsilon = 1e-15;
}

GodObjectRenderer::GodObjectRenderer( HAPIFloat _min_distance, bool _secondary_collisions):
  proxy_position( GodObjectRendererConstants::UNINITIALIZED_PROXY_POS ),
  min_distance ( _min_distance ), secondary_collisions ( _secondary_collisions ) {
}

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


inline Vec3 moveAboveConstraints(  
          const Vec3 &pos,                                        
          vector< PlaneConstraint > &constraints,
          HAPIFloat min_distance) {
  Vec3 moved_pos = pos;
  bool done = false;
  int counter = 0;
  while( !done && counter < 25 ) {
    done = true;
    for( vector< PlaneConstraint >::iterator i = constraints.begin();
         i != constraints.end(); ++i ) {
      HAPIFloat d = (*i).normal * (moved_pos - (*i).point );
      if( d <= 0 ) {
        moved_pos = moved_pos + (*i).normal * (-d+min_distance);
        done = false;
      }
    }
    ++counter;
  }

  return moved_pos;
}

void GodObjectRenderer::onOnePlaneContact( 
                            const Vec3& proxy_pos,
                            const PlaneConstraint &c, 
                            HAPISurfaceObject::ContactInfo &contact,
                            const HapticShapeVector &shapes ) {
  
  // create a local coordinate system with the PlaneConstraint normal
  // as y-axis
  contact.y_axis = c.normal;
  // In the following link there is a possible speedup to the code below.
  // The problem is that it only works by assuming a certain architecture and
  // bit size of integer.
  // http://akpeters.metapress.com/content/x5up35714j28v881/fulltext.pdf
  HAPIFloat abs_y_axis_x = H3DUtil::H3DAbs( contact.y_axis.x );
  HAPIFloat abs_y_axis_y = H3DUtil::H3DAbs( contact.y_axis.y );
  HAPIFloat abs_y_axis_z = H3DUtil::H3DAbs( contact.y_axis.z );
  if( abs_y_axis_x < abs_y_axis_y && abs_y_axis_x < abs_y_axis_z ) {
    contact.x_axis = Vec3( 0, contact.y_axis.z, -contact.y_axis.y );
  } else if( abs_y_axis_y < abs_y_axis_x && abs_y_axis_y < abs_y_axis_z ) {
    contact.x_axis = Vec3( -contact.y_axis.z, 0, contact.y_axis.x );
  } else {
    // Need to handle the case when y_axis = 0,0,1
    if( abs_y_axis_y > Constants::epsilon ||
        abs_y_axis_x > Constants::epsilon )
      contact.x_axis = Vec3( contact.y_axis.y, -contact.y_axis.x, 0 );
    else
      contact.x_axis = Vec3( -contact.y_axis.z, 0, 0 );
  }
  contact.z_axis = contact.x_axis % contact.y_axis;
  contact.x_axis.normalizeSafe();
  contact.z_axis.normalizeSafe();
  contact.setGlobalOrigin( contact.contact_point_global );
  assert( c.haptic_shape.get() );

  contact.haptic_shape = c.haptic_shape.get();
  contact.geom_primitive = c.primitive;
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
  tryProxyMovement( proxy_pos, 
                    new_proxy_pos,
                    1, 
                    c, 
                    shapes, 
                    o );

  contact.setGlobalOrigin( o );
  
  // call the surface to determine forces and proxy movement
  c.haptic_shape->getSurface()->getForces( contact );
  
  // add a contact
  tmp_contacts.push_back( make_pair( c.haptic_shape, contact ) );
}

void GodObjectRenderer::onTwoPlaneContact( 
           const Vec3& proxy_pos,                           
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
    onOnePlaneContact( proxy_pos, p1, contact, shapes );
  } else if( local_pos.y > Constants::epsilon ) {
    onOnePlaneContact( proxy_pos, p0, contact, shapes );
  } else {
    // both planes constrains

    HAPIFloat sum = local_pos.x + local_pos.y;
    
    HAPIFloat weight = 0;
    if( sum < 0 )
      weight = local_pos.x / sum;
    
    // create a local coordinate system with the PlaneConstraint normal
    // as y-axis. The y-axis should point towards the proxy.
    contact.y_axis = -( p0.normal * local_pos.x + p1.normal * local_pos.y );
    contact.y_axis.normalizeSafe();
    
    Vec3 a( contact.y_axis.z, contact.y_axis.x, contact.y_axis.y );
    contact.x_axis = contact.y_axis % a;
    contact.z_axis = contact.x_axis % contact.y_axis;
    contact.z_axis.normalizeSafe();
    contact.x_axis.normalizeSafe();
    contact.setGlobalOrigin( contact.contact_point_global );
 
    Vec3 line_dir_local = contact.vectorToLocal( line_dir );

    assert( p0.haptic_shape.get() );

    contact.haptic_shape = p0.haptic_shape.get();
    contact.geom_primitive = p0.primitive;
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
    contact.haptic_shape = p1.haptic_shape.get();
    contact.geom_primitive = p1.primitive;
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
    tryProxyMovement( proxy_pos,
                      new_proxy_pos,
                      2, 
                      p0, 
                      shapes, 
                      o );
    contact.setGlobalOrigin( o );
    
    contact.haptic_shape = p0.haptic_shape.get();
    contact.geom_primitive = p0.primitive;
    p0.haptic_shape->getSurface()->getForces( contact );
    Vec3 p0_force = contact.force_global;
    contact.haptic_shape = p1.haptic_shape.get();
    contact.geom_primitive = p1.primitive;
    p1.haptic_shape->getSurface()->getForces( contact );
    Vec3 p1_force = contact.force_global;

    contact.force_global = p0_force * weight + p1_force * ( 1 - weight );

    // add contacts
    tmp_contacts.push_back( make_pair( p0.haptic_shape, contact ) );
    if( p0.haptic_shape.get() != p1.haptic_shape.get() )
      tmp_contacts.push_back( make_pair( p1.haptic_shape, contact ) );
  }
}


void GodObjectRenderer::onThreeOrMorePlaneContact(  
          const Vec3 &proxy_pos,                                        
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
    onTwoPlaneContact( proxy_pos, p1, p2, contact, shapes );
  } else if( probe_local_pos.y > Constants::epsilon ) {
    onTwoPlaneContact( proxy_pos, p0, p2, contact, shapes );
  } else if( probe_local_pos.z > Constants::epsilon ) {
    onTwoPlaneContact( proxy_pos, p0, p1, contact, shapes );
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
    contact.setGlobalOrigin( contact.contact_point_global + min_distance * contact.y_axis );

    // calculate the force and proxy movement for the first plane
    assert( p0.haptic_shape.get() );
    contact.haptic_shape = p0.haptic_shape.get();
    contact.geom_primitive = p0.primitive;
    p0.haptic_shape->getSurface()->getForces( contact );
    Vec3 p0_force = contact.force_global;
    
    // calculate the force and proxy movement for the second plane
    assert( p1.haptic_shape.get() );
    contact.haptic_shape = p1.haptic_shape.get();
    contact.geom_primitive = p1.primitive;
    p1.haptic_shape->getSurface()->getForces( contact );
    Vec3 p1_force = contact.force_global;

    // calculate the force and proxy movement for the third plane
    assert( p2.haptic_shape.get() );
    contact.haptic_shape = p2.haptic_shape.get();
    contact.geom_primitive = p2.primitive;
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
    tmp_contacts.push_back( make_pair( p0.haptic_shape, contact ) );
    if( p0.haptic_shape.get() != p1.haptic_shape.get() ) {
      tmp_contacts.push_back( make_pair( p1.haptic_shape, contact ) );
    }
    
    if( p0.haptic_shape.get() != p2.haptic_shape.get() &&
        p1.haptic_shape.get() != p2.haptic_shape.get() ) {
      tmp_contacts.push_back( make_pair( p2.haptic_shape, contact ) );
    }
  }    
}





HAPIForceEffect::EffectOutput 
GodObjectRenderer::renderHapticsOneStep( HAPIHapticsDevice *hd,
                                         const HapticShapeVector &shapes,
                                         HAPITime dt ) {
  // get the current device values
  HAPIHapticsDevice::DeviceValues input = hd->getDeviceValues();

  proxy_info_lock.lock();
  if( proxy_position == GodObjectRendererConstants::UNINITIALIZED_PROXY_POS ) {
    proxy_position = input.position;
  }
  Vec3 proxy_pos = proxy_position;
  proxy_info_lock.unlock();

  Vec3 moved_proxy_pos = proxy_pos;

  // set to true of any of the shapes we are to render is dynamic.
  bool have_dynamic_shape = false;

  // move proxy pos with moving shapes.
  for( HapticShapeVector::const_iterator i = shapes.begin();
       i != shapes.end();
       ++i ) {
    if( (*i)->isDynamic() ) { 
      // we have a dynamic shape so set 
      have_dynamic_shape = true;
      Collision::IntersectionInfo intersection;
      // by using the same position in from and to the intersection will be the 
      // point to which the proxy has been moved by the shape itself.
      if( (*i)->lineIntersect( moved_proxy_pos, moved_proxy_pos, intersection,
                               (*i)->getTouchableFace(), true ) ){
        moved_proxy_pos = intersection.point + min_distance * intersection.normal;
        // add an extra constraint for this shape at the new position. It might not have
        // been intersected last loop and ignoring it might cause fallthrough because
        // it will not be present when we move proxy above all constraint planes.
        all_constraints.push_back( PlaneConstraint( intersection.point, intersection.normal ));
      }
    }
  }
  
  if( have_dynamic_shape ) {
    // move plane with last transformation of the shape it belongs to
    for( vector< PlaneConstraint >::iterator i = all_constraints.begin();
         i != all_constraints.end(); ++i ) {
      if( (*i).haptic_shape.get() && (*i).haptic_shape->isDynamic() ) {
        (*i).point = (*i).haptic_shape->getTransform() * (*i).haptic_shape->getLastInverse() * (*i).point; 
      }
    }
    
    // make sure the new proxy position is above the constraint planes
    // from the last loop in order to prevent fallthrough.
    moved_proxy_pos = 
      moveAboveConstraints( moved_proxy_pos, all_constraints, min_distance );
  }

  proxy_pos = moved_proxy_pos;

  // clear all previous contacts
  tmp_contacts.clear();

  HAPIForceEffect::EffectOutput output;
  bool has_intersection = false;

  Collision::IntersectionInfo closest_intersection;
  
  // the intersections that are closest to the user (several if the 
  // intersections are in the same point)
  closest_constraints.clear();

  // clear the all_constraints(only valid when we have a dynamic shape so ignore if not)
  if( have_dynamic_shape )
    all_constraints.clear();

  // calculate the forces generated by the active haptic shapes
  for( HapticShapeVector::const_iterator i = shapes.begin();
       i != shapes.end();
       ++i ) {
    Collision::IntersectionInfo intersection;
    if( (*i)->lineIntersect( proxy_pos, input.position, intersection,
                             (*i)->getTouchableFace()  ) ){
      // shape is intersected, create a plane constraint at the
      // intersection point
      PlaneConstraint pc( intersection.point, 
                          intersection.face == Collision::FRONT ?
                          intersection.normal : -intersection.normal,
                          intersection.tex_coord,
                          *i,
                          intersection.primitive );

      // add constraints(only when dynamic shape, since it is only used then)
      if( have_dynamic_shape )
        all_constraints.push_back( pc );

      if( !has_intersection ) {
        // first shape that was intersected, and therefore also the
        // closest intersection
        closest_intersection = intersection;
        has_intersection = true;
        closest_constraints.push_back( pc  );
      } else {
        // this is not the first intersection, check if it closer than
        // previous ones
      
        if( (closest_intersection.point - intersection.point).lengthSqr()
            < GodObjectRendererConstants::length_sqr_point_epsilon ) {
          // intersection point is the same as previous intersections,
          // check if the normal is the same as a previous intersection.
          // if it is the new intersection is discarded.
          bool unique_constraint = true;
          for( vector< PlaneConstraint >::iterator j = 
                 closest_constraints.begin();
               j != closest_constraints.end(); ++j ) {
            if( ( intersection.normal - (*j).normal ).lengthSqr() < 
                GodObjectRendererConstants::length_sqr_normal_epsilon ) {
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
          if( intersection.t < closest_intersection.t ) {
            // new intersection is closer than old intersection
            closest_intersection = intersection;
            closest_constraints.clear();
            closest_constraints.push_back( pc );
          } 
        }
      }
    } 
  }
   
  unsigned int nr_constraints = (unsigned int ) closest_constraints.size();

  Vec3 new_proxy_pos, new_force;

  HAPISurfaceObject::ContactInfo contact;
  contact.hd = hd;

  // fill in contact information
  contact.contact_point_global = closest_intersection.point;
  contact.probe_position_global = input.position;
  contact.probe_velocity_global = input.velocity;
  contact.proxy_radius = 0;
  contact.tex_coord = closest_intersection.tex_coord;

  if( nr_constraints == 0 ) {
    new_proxy_pos = proxy_pos + (input.position-proxy_pos)*0.05;
  } else {
    if( nr_constraints == 1 ) {
      onOnePlaneContact( proxy_pos, closest_constraints[0], contact, shapes );
    } else if( nr_constraints == 2 ) {
      onTwoPlaneContact( proxy_pos,
                         closest_constraints[0],
                         closest_constraints[1], contact, shapes );
    } if( nr_constraints >= 3 ) {
      onThreeOrMorePlaneContact( proxy_pos,
                                 closest_constraints,
                                 contact, shapes );
    } 

    new_proxy_pos = 
      contact.globalOrigin(); 

    new_force = contact.force_global;
  }

  output.force = new_force;
  proxy_info_lock.lock();
  proxy_position = new_proxy_pos;
  proxy_info_lock.unlock();
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

  Collision::IntersectionInfo closest_intersection;
  Collision::Plane plane( Vec3(0, 0, 0), Vec3(0, 0, 0) );

  while( have_new_goal ) {
    have_new_goal = false;
    
    // find the closest intersection
    has_intersection = false;
    for( HapticShapeVector::const_iterator i = shapes.begin();
         i != shapes.end();
         ++i ) {
      Collision::IntersectionInfo intersection;
      if( (*i)->lineIntersect( from_point, to_point, intersection,
                               (*i)->getTouchableFace()) ){
        if( !has_intersection ) {
          closest_intersection = intersection;
          has_intersection = true;
        } else {
          if( intersection.t < closest_intersection.t ) {
            closest_intersection = intersection;
          } 
        }
      } 
    }
        
    if( has_intersection ) {      
      if( nr_constraints == 1 && first_loop ) {
        Vec3 normal = 
          closest_intersection.face == Collision::FRONT ?
          closest_intersection.normal : -closest_intersection.normal;

        // Find intersection with moved plane along the line from point to
        // closest intersection with shapes.
        // Needed in order to not accidently project behind other constraints
        // when projecting along the intersection of the two moved planes.
        // May not be the best solution but it minimizes the problem at 
        // relatively low time cost. It is certainly better then to move
        // the from_point to an arbitrary position in the direction of the
        // closest_intersection normal.
        plane.normal = normal;
        plane.point = closest_intersection.point + normal * min_distance;
        Collision::IntersectionInfo intersection;
        if( plane.lineIntersect( from_point,
                                 closest_intersection.point,
                                 intersection ) ) {
          from_point = intersection.point;
        }

        Vec3 old_from_point = from_point;

        // project onto plane intersection between intersection plane and
        // closest constraint.
        from_point =  
          projectOntoPlaneIntersection( from_point, 
                                        closest_intersection.point + 
                                        normal * min_distance,
                                        normal,
                                        pc.point + 
                                        pc.normal * min_distance,
                                        pc.normal );

        if (secondary_collisions) {
          for (HapticShapeVector::const_iterator i = shapes.begin();
            i != shapes.end();
            ++i) {
            Collision::IntersectionInfo intersection;
            if ((*i)->lineIntersect(old_from_point, from_point, intersection,
              (*i)->getTouchableFace())) {
              from_point = old_from_point;
              break;
            }
          }
        }

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
        if( l >= min_distance + 1e-11) {
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

