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
/// \file HAPIVariableDepthSurface.cpp
/// \brief cpp file for HAPIVariableDepthSurface
///
//
//////////////////////////////////////////////////////////////////////////////
#include "HAPIVariableDepthSurface.h"
#include <ExtremaFindingAlgorithms.h>

using namespace HAPI;

HAPIVariableDepthSurface::HAPIVariableDepthSurface(
        HAPIFloat _stiffness,
        HAPIFloat _static_friction,
        HAPIFloat _dynamic_friction,
        HAPIFloat (*_func)( const Vec2 &local_point, void *data ),
        int _max_iterations,
        HAPIFloat _minimization_epsilon ) :
  stiffness( _stiffness ),
  static_friction( _static_friction ),
  dynamic_friction( _dynamic_friction ),
  func( _func ),
  max_iterations( _max_iterations ),
  minimization_epsilon( _minimization_epsilon ),
  in_static_contact( true ),
  depth_invert( false ) {
}

void HAPIVariableDepthSurface::getProxyMovement( ContactInfo &contact ) {
  if( H3DUtil::H3DAbs( static_friction ) < Constants::epsilon &&
      H3DUtil::H3DAbs( dynamic_friction ) < Constants::epsilon ) {
    this_contact_info = &contact;
    Vec3 local_probe = contact.localProbePosition();
    if( local_probe.y > 0 )
      depth_invert = true;
    else
      depth_invert = false;
    Vec2 movement = Vec2( local_probe.x, local_probe.z );
    Vec2 half_movement = movement / 2;
    Vec2 v[] = { Vec2(0, 0 ), movement, half_movement +
      Vec2( half_movement.y, -half_movement.x ) };
    Vec2 res;
    
    depth_get_lock.lock();
    H3DUtil::DownhillSimplexMethod::amoeba< Vec2, HAPIFloat, 2>(
                                      v,
                                      &HAPIVariableDepthSurface::localPtToDist,
                                      res,
                                      this,
                                      minimization_epsilon,
                                      max_iterations );
    depth_get_lock.unlock();
    
    contact.proxy_movement_local = res;
  } else {
    this_contact_info = &contact;

    Vec3 local_probe = contact.localProbePosition();
    if( local_probe.y > 0 )
      depth_invert = true;
    else
      depth_invert = false;
    Vec2 movement = Vec2( local_probe.x, local_probe.z );
    Vec2 half_movement = movement / 2;
    Vec2 v[] = { Vec2(0, 0 ), movement, half_movement +
      Vec2( half_movement.y, -half_movement.x ) };
    Vec2 res;

    depth_get_lock.lock();
    HAPIFloat start_depth = getDepth( Vec2());
    Vec3 force = ( Vec3( 0, start_depth, 0 ) - contact.localProbePosition() )
                  * stiffness;
    Vec2 force_t( force.x, force.z );
    HAPIFloat normal_force = force.y;
    HAPIFloat tangent_force = force_t.length();

    H3DUtil::DownhillSimplexMethod::amoeba< Vec2, HAPIFloat, 2>(
                                      v,
                                      &HAPIVariableDepthSurface::localPtToDist,
                                      res,
                                      this,
                                      minimization_epsilon,
                                      max_iterations );
    HAPIFloat res_l_sqr = res.lengthSqr();
    HAPIFloat res_l = H3DUtil::H3DSqrt( res_l_sqr );
    // If we have movement we need to calculate how friction affects it.
    if( res_l_sqr > Constants::epsilon ) {
      HAPIFloat force_res_dot = 2;
      Vec2 force_t_n = -force_t;
      Vec2 res_n = res;
      if( tangent_force > Constants::epsilon ) {
        force_t_n = force_t_n / tangent_force;
        res_n = res_n / res_l;
        force_res_dot = force_t_n * res_n;
      }

      HAPIFloat depth = getDepth( res );
      bool normal_found = false;
      Vec3 found_normal;
      // If the direction of the force and the direction of movement differs
      // a lot then use the cross product to calculate a normal direction for
      // the plane sliding along.
      if( force_res_dot < 0.3 ) {
        HAPIFloat depth_f = getDepth( force_t_n * res_l );
        depth_get_lock.unlock();
        found_normal = Vec3( res.x, depth - start_depth, res.y ) %
                       Vec3( force_t_n.x, depth_f - start_depth, force_t_n.y );

        if( found_normal.lengthSqr() > Constants::epsilon ) {
          normal_found = true;
        } 
      } else
        depth_get_lock.unlock();

      // If the direction of force and the direction of movement is kind of the
      // the same then calculate the normal a bit differently.
      // This is an ad hoc solution that seems to give the best result.
      // TODO: review this when the friction system works better.
      if( !normal_found ) {
        HAPIFloat depth_diff = depth - start_depth;
        if( H3DUtil::H3DAbs( depth_diff ) > Constants::epsilon &&
            ( ( local_probe.y >= 0 && depth <= local_probe.y ) ||
            ( local_probe.y < 0 && depth >= local_probe.y ) ) ) {
          HAPIFloat depth_diff_squared = depth_diff * depth_diff;
          HAPIFloat a1_sqr_rt = res_l;
          HAPIFloat a2_sqr_rt = H3DUtil::H3DSqrt( res_l_sqr +
                                                  depth_diff_squared );
          HAPIFloat y_coord = depth_diff * a1_sqr_rt / a2_sqr_rt;
          found_normal = - H3DUtil::H3DSqrt( depth_diff_squared -
                                             y_coord * y_coord ) *
                           Vec3( res.x, 0, res.y ) / a1_sqr_rt;
          found_normal.y = y_coord;
          normal_found = true;
        }
      }

      if( normal_found ) {
        found_normal.normalize();
        if( found_normal.y < 0 )
          found_normal = -found_normal;
        normal_force = force * found_normal;
        tangent_force = force.length() - H3DUtil::H3DAbs( normal_force );
        if( normal_force < 0 )
          normal_force = 0;
      }
    } else
      depth_get_lock.unlock();

    if( in_static_contact ) {
      if( tangent_force <= static_friction * normal_force ) {
        contact.proxy_movement_local = Vec2( 0, 0 );
      } else {
        in_static_contact = false;
      }
    }

    if( !in_static_contact ) {

      HAPIFloat b = 1;
      HAPIFloat dt = 1e-3;
      HAPIFloat velocity = 
        ( tangent_force - dynamic_friction * normal_force ) / b;

      if( velocity < Constants::epsilon ) {
        in_static_contact = true;
        velocity = 0;
        contact.proxy_movement_local = Vec2( 0, 0 ); 
      } else {
        // The max_movement should be in mm. The velocity gotten is calculated
        // using forces which is in newton (since stiffness is in N/mm), this
        // means that in order to get the maximum movement on one frame we do:
        // max_movement = velocity * 1000 * 0.001.
        // Where 1000 is conversion from mm to m and
        // 0.001 is the maximum time (in seconds) of one frame. This simplifies
        // the expression to max_movement = velocity
        HAPIFloat max_movement = velocity;
        
        HAPIFloat l = res_l;
        if( l > max_movement ) {
          res *= max_movement / l; 
        }
        contact.proxy_movement_local = res;
      }
    }
  }
}

void HAPIVariableDepthSurface::getForces( ContactInfo &contact_info ) {
  Vec3 temp_point = contact_info.globalOrigin();
  Vec3 glbl_vect = temp_point - contact_info.contact_point_global;
  Vec3 glbl_local_probe_vector = contact_info.globalProbePosition() -
    - contact_info.contact_point_global;
  depth_get_lock.lock();
  this_contact_info = &contact_info;
  Vec3 temp_local_vec = contact_info.vectorToLocal( glbl_vect );
  HAPIFloat depth = getDepth( Vec2( temp_local_vec.x, temp_local_vec.z ) );
  depth_get_lock.unlock();
  HAPIFloat local_probe_pos = contact_info.localProbePosition().y;
  if( ( local_probe_pos >= 0 && depth > local_probe_pos ) ||
    ( local_probe_pos < 0 && depth < local_probe_pos ) ) {
      contact_info.setGlobalForce( Vec3() );
  }
  else {

    //depth = depth_invert * depth;
    Vec3 real_contact_point = temp_point + depth * contact_info.y_axis;

    Vec3 probe_to_origin = 
      real_contact_point - contact_info.globalProbePosition();
    contact_info.setGlobalForce(  probe_to_origin * stiffness );
  }
}

HAPIFloat HAPIVariableDepthSurface::localPtToDist( Vec2 local_point,
                                                   void *user_data ) {
  HAPIVariableDepthSurface *bmhs =
    static_cast< HAPIVariableDepthSurface * >(user_data);
  if( bmhs ) {
    Vec3 glbl_vect = bmhs->this_contact_info->
      vectorToGlobal( Vec3( local_point.x, 0, local_point.y ) );
    HAPIFloat depth = bmhs->getDepth( local_point );
    Vec3 real_contact_point = bmhs->this_contact_info->origin_global +
                               glbl_vect + depth *
                               bmhs->this_contact_info->y_axis;
    return ( real_contact_point -
             bmhs->this_contact_info->globalProbePosition() ).lengthSqr();
  }
  return (std::numeric_limits< HAPIFloat >::max)();
}
