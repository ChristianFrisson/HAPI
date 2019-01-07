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
/// \file HAPI/src/HapticTexturesSurface.cpp
/// \brief cpp file for HapticTexturesSurface
///
//
//////////////////////////////////////////////////////////////////////////////

#include <HAPI/HapticTexturesSurface.h>
#include <HAPI/HAPIHapticsDevice.h>

using namespace HAPI;

HapticTexturesSurface::HapticTexturesSurface( HAPIFloat _stiffness,
                                  HAPIFloat _damping,
                                  HAPIFloat _static_friction,
                                  HAPIFloat _dynamic_friction,
                                  H3DUtil::Image * _stiffness_image,
                                  HAPIFloat _max_stiffness_value,
                                  HAPIFloat _min_stiffness_value,
                                  H3DUtil::Image * _damping_image,
                                  HAPIFloat _max_damping_value,
                                  HAPIFloat _min_damping_value,
                                  H3DUtil::Image * _static_friction_image,
                                  HAPIFloat _max_static_friction_value,
                                  HAPIFloat _min_static_friction_value,
                                  H3DUtil::Image * _dynamic_friction_image,
                                  HAPIFloat _max_dynamic_friction_value,
                                  HAPIFloat _min_dynamic_friction_value,
                                  bool _use_relative_values,
                                  bool _use_ref_count_lock ) :
  HAPIFrictionSurface( _stiffness, _damping, _static_friction,
                       _dynamic_friction, _use_relative_values,
                       _use_ref_count_lock ),
  max_stiffness_value( _max_stiffness_value ),
  max_damping_value( _max_damping_value ),
  max_static_friction_value( _max_static_friction_value ),
  max_dynamic_friction_value( _max_dynamic_friction_value ),
  min_stiffness_value( _min_stiffness_value ),
  min_damping_value( _min_damping_value ),
  min_static_friction_value( _min_static_friction_value ),
  min_dynamic_friction_value( _min_dynamic_friction_value ) {

  if( _stiffness_image )
    stiffness_image.reset( new ImageInterfaceObject( _stiffness_image,
                                                     &stiffness_lock ) );
  if( _damping_image )
    damping_image.reset( new ImageInterfaceObject( _damping_image,
                                                   &damping_lock ) );
  if( _static_friction_image )
    static_friction_image.reset( 
      new ImageInterfaceObject( _static_friction_image,
                                &static_friction_lock ) );
  if( _dynamic_friction_image )
    dynamic_friction_image.reset( 
      new ImageInterfaceObject( _dynamic_friction_image,
                                &dynamic_friction_lock ) );
}

void HapticTexturesSurface::getProxyMovement( ContactInfo &contact_info ) {
  Vec3 contact_tex_coord = contact_info.contactPointTexCoord();
  HAPIFloat local_static_friction = static_friction;
  static_friction_lock.lock();
  if( static_friction_image.get() ) {
    local_static_friction = min_static_friction_value +
      static_friction_image->getSample( contact_tex_coord ) *
      ( max_static_friction_value - min_static_friction_value );
  }
  static_friction_lock.unlock();

  HAPIFloat local_dynamic_friction = dynamic_friction;
  dynamic_friction_lock.lock();
  if( dynamic_friction_image.get() ) {
    local_dynamic_friction = min_dynamic_friction_value +
      dynamic_friction_image->getSample( contact_tex_coord ) *
        ( max_dynamic_friction_value - min_dynamic_friction_value );
  }
  dynamic_friction_lock.unlock();

  if( H3DUtil::H3DAbs( local_static_friction ) < Constants::epsilon &&
      H3DUtil::H3DAbs( local_dynamic_friction ) < Constants::epsilon ) {
    // SmoothSurface
    Vec3 local_probe = contact_info.localProbePosition();
    contact_info.setLocalProxyMovement( Vec2( local_probe.x , local_probe.z ) );
  } else {
    // FrictionalSurface
    // Friction model calculation modeled after the information in
    // section 4 of "Haptic Interaction in Virtual Environments" by
    // Diego C. Ruspini, Krasimir Kolarov and Oussama Khatib.
    // Published 1997.
    Vec3 local_probe = contact_info.localProbePosition();

    HAPIFloat local_stiffness = stiffness;
    stiffness_lock.lock();
    if( stiffness_image.get() ) {
      local_stiffness = min_stiffness_value + 
        stiffness_image->getSample( contact_tex_coord ) *
        ( max_stiffness_value - min_stiffness_value );
    }
    stiffness_lock.unlock();
    if( use_relative_values ) {
      local_stiffness = stiffness *
        contact_info.hapticsDevice()->getMaxStiffness();
    }
    Vec3 force = local_probe * -local_stiffness;
    Vec2 force_t( force.x, force.z );
    //This part is needed to compile on linux.
    Vec2 local_probe_2d =  Vec2( local_probe.x, local_probe.z );

    setLocalProxyMovement( contact_info,
                           force_t.length(),
                           force.y,
                           local_probe_2d,
                           local_static_friction,
                           local_dynamic_friction );
  }
}

void HapticTexturesSurface::getForces( ContactInfo &contact_info ) {
  HAPIFloat local_stiffness = stiffness;
  Vec3 contact_tex_coord = contact_info.contactPointTexCoord();
  stiffness_lock.lock();
  if( stiffness_image.get() ) {
    local_stiffness = min_stiffness_value +
      stiffness_image->getSample( contact_tex_coord ) *
      ( max_stiffness_value - min_stiffness_value );
  }
  stiffness_lock.unlock();

  HAPIFloat local_damping = damping;
  damping_lock.lock();
  if( damping_image.get() ) {
    local_damping = min_damping_value +
      damping_image->getSample( contact_tex_coord ) *
      ( max_damping_value - min_damping_value );
  }
  damping_lock.unlock();
  getForcesInternal( contact_info, contact_info.globalOrigin(),
                     local_stiffness, local_damping, use_relative_values );
}

void HapticTexturesSurface::setParameterImage( H3DUtil::Image * image,
                                               ParameterType param ) {
  switch( param ) {
    case STIFFNESS: {
      if( image ) {
        if( !stiffness_image.get() ) {
          ImageInterfaceObject * new_image_interface =
            new ImageInterfaceObject( image, &stiffness_lock );
          stiffness_lock.lock();
          stiffness_image.reset( new_image_interface );
          stiffness_lock.unlock();
        } else stiffness_image->setImage( image );
      } else if( stiffness_image.get() ) {
        stiffness_lock.lock();
        stiffness_image.reset(0);
        stiffness_lock.unlock();
      }
      break;
    }
    case DAMPING: {
      if( image ) {
        if( !damping_image.get() ) {
          ImageInterfaceObject * new_image_interface =
            new ImageInterfaceObject( image, &damping_lock );
          damping_lock.lock();
          damping_image.reset( new_image_interface );
          damping_lock.unlock();
        } else damping_image->setImage( image );
      } else if( damping_image.get() ) {
        damping_lock.lock();
        damping_image.reset( 0 );
        damping_lock.unlock();
      }
      break;
    }
    case STATIC_FRICTION: {
      if( image ) {
        if( !static_friction_image.get() ) {
          ImageInterfaceObject * new_image_interface =
            new ImageInterfaceObject( image, &static_friction_lock );
          static_friction_lock.lock();
          static_friction_image.reset( new_image_interface );
          static_friction_lock.unlock();
        } else static_friction_image->setImage( image );
      } else if( static_friction_image.get() ) {
        static_friction_lock.lock();
        static_friction_image.reset( 0 );
        static_friction_lock.unlock();
      }
      break;
    }
    case DYNAMIC_FRICTION: {
      if( image ) {
        if( !dynamic_friction_image.get() ) {
          ImageInterfaceObject * new_image_interface =
            new ImageInterfaceObject( image, &dynamic_friction_lock );
          dynamic_friction_lock.lock();
          dynamic_friction_image.reset( new_image_interface );
          dynamic_friction_lock.unlock();
        } else dynamic_friction_image->setImage( image );
      } else if( dynamic_friction_image.get() ) {
        dynamic_friction_lock.lock();
        dynamic_friction_image.reset( 0 );
        dynamic_friction_lock.unlock();
      }
      break;
    }
    default: {}
  }
}

void HapticTexturesSurface::setParameterMaxValue( HAPIFloat value,
                                                  ParameterType param ) {
  switch( param ) {
    case STIFFNESS: {
      max_stiffness_value = value;
    }
    case DAMPING: {
      max_damping_value = value;
    }
    case STATIC_FRICTION: {
      max_static_friction_value = value;
    }
    case DYNAMIC_FRICTION: {
      max_dynamic_friction_value = value;
    }
    default: {}
  }
}

HAPIFloat HapticTexturesSurface::getParameterMaxValue( ParameterType param ) {
  switch( param ) {
    case STIFFNESS: {
      return max_stiffness_value;
    }
    case DAMPING: {
      return max_damping_value;
    }
    case STATIC_FRICTION: {
      return max_static_friction_value;
    }
    case DYNAMIC_FRICTION: {
      return max_dynamic_friction_value;
    }
    default: { return -1; }
  }
}

void HapticTexturesSurface::setParameterMinValue( HAPIFloat value,
                                                  ParameterType param ) {
  switch( param ) {
    case STIFFNESS: {
      min_stiffness_value = value;
    }
    case DAMPING: {
      min_damping_value = value;
    }
    case STATIC_FRICTION: {
      min_static_friction_value = value;
    }
    case DYNAMIC_FRICTION: {
      min_dynamic_friction_value = value;
    }
    default: {}
  }
}

HAPIFloat HapticTexturesSurface::getParameterMinValue( ParameterType param ) {
  switch( param ) {
    case STIFFNESS: {
      return min_stiffness_value;
    }
    case DAMPING: {
      return min_damping_value;
    }
    case STATIC_FRICTION: {
      return min_static_friction_value;
    }
    case DYNAMIC_FRICTION: {
      return min_dynamic_friction_value;
    }
    default: { return -1; }
  }
}

