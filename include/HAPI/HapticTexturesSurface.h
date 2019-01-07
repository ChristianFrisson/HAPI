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
/// \file HAPI/HapticTexturesSurface.h
/// \brief Header file for HapticTexturesSurface
///
//
//////////////////////////////////////////////////////////////////////////////
#ifndef __HAPI_HAPTICTEXTURESSURFACE_H__
#define __HAPI_HAPTICTEXTURESSURFACE_H__

#include <HAPI/HAPIFrictionSurface.h>
#include <HAPI/ImageInterfaceObject.h>
#include <memory>

namespace HAPI {
  /// \ingroup Surfaces
  /// \class HapticTexturesSurface
  /// \brief A surface with friction capabilities in which each parameter can
  /// be modified by a texture.
  ///
  /// If an image is specified for a parameter the value for that parameter
  /// will be the value at the contact point. The surface is optimized for one
  /// channel gray scale images using unsigned values from 0 to 255. Black (0)
  /// corresponds to a low value and White(255) corresponds to a high value.
  /// Image values are interpolated (linear) between min and max values
  /// specified for a parameter.
  /// NOTE: No check is done to ensure that the min value is lower than the
  /// corresponding max value.
  class HAPI_API HapticTexturesSurface : public HAPIFrictionSurface {
  public:

    /// Used to control which image is set for which parameter.
    typedef enum {
      STIFFNESS = 0,
      DAMPING = 1,
      STATIC_FRICTION = 2,
      DYNAMIC_FRICTION = 3 
    } ParameterType;

    /// Constructor
    HapticTexturesSurface( HAPIFloat _stiffness = 0.35,
                           HAPIFloat _damping = 0,
                           HAPIFloat _static_friction = 0,
                           HAPIFloat _dynamic_friction = 0,
                           H3DUtil::Image * _stiffness_image = 0,
                           HAPIFloat _max_stiffness_value = 1.0,
                           HAPIFloat _min_stiffness_value = 0.0,
                           H3DUtil::Image * _damping_image = 0,
                           HAPIFloat _max_damping_value = 1.0,
                           HAPIFloat _min_damping_value = 0.0,
                           H3DUtil::Image * _static_friction_image = 0,
                           HAPIFloat _max_static_friction_value = 1.0,
                           HAPIFloat _min_static_friction_value = 0.0,
                           H3DUtil::Image * _dynamic_friction_image = 0,
                           HAPIFloat _max_dynamic_friction_value = 1.0,
                           HAPIFloat _min_dynamic_friction_value = 0.0,
                           bool _use_relative_values = true,
                           bool _use_ref_count_lock = true );

    /// Moves the proxy along the surface in the direction of the
    /// probe. The friction parameters limit the movement and can
    /// sometimes stop the proxy from being moved at all.
    virtual void getProxyMovement( ContactInfo &contact_info );

    /// Calculate a force from the probe towards the proxy.
    virtual void getForces( ContactInfo &contact_info );

    /// Set the maximum allowed value for one of the parameters stiffness,
    /// damping static_friction and dynamic_friction. The maximum value is
    /// used only when the parameter value is obtained from an image.
    /// \param value The new maximum value.
    /// \param param The parameter type to change the maximum value for.
    void setParameterMaxValue( HAPIFloat value, ParameterType param );

    /// Get the maximum allowed value of a parameter obtained from an image.
    /// \param param The parameter type to get.
    /// \returns The value of the chosen maximum parameter.
    HAPIFloat getParameterMaxValue( ParameterType param );

    /// Set the minimum allowed value for one of the parameters stiffness,
    /// damping static_friction and dynamic_friction. The minimum value is
    /// used only when the parameter value is obtained from an image.
    /// \param value The new minimum value.
    /// \param param The parameter type to change the minimum value for.
    void setParameterMinValue( HAPIFloat value, ParameterType param );

    /// Get the minimum allowed value of a parameter obtained from an image.
    /// \param param The parameter type to get.
    /// \returns The value of the chosen minimum parameter.
    HAPIFloat getParameterMinValue( ParameterType param );

    /// Set the image used to control the value of a parameter.
    /// \param image The image to use.
    /// \param param The parameter type the image should control.
    void setParameterImage( H3DUtil::Image * image, ParameterType param );

  protected:
    // Maximum allowed values for the different parameters when the values
    // of the parameters are obtained from the image.
    HAPIFloat max_stiffness_value;
    HAPIFloat max_damping_value;
    HAPIFloat max_static_friction_value;
    HAPIFloat max_dynamic_friction_value;

    // Minimum allowed values for the different parameters when the values
    // of the parameters are obtained from the image.
    HAPIFloat min_stiffness_value;
    HAPIFloat min_damping_value;
    HAPIFloat min_static_friction_value;
    HAPIFloat min_dynamic_friction_value;

    // Containers used to access values in image.
    std::auto_ptr< ImageInterfaceObject > stiffness_image;
    std::auto_ptr< ImageInterfaceObject > damping_image;
    std::auto_ptr< ImageInterfaceObject > static_friction_image;
    std::auto_ptr< ImageInterfaceObject > dynamic_friction_image;

    // Lock used when getting values from images.
    H3DUtil::MutexLock stiffness_lock;
    H3DUtil::MutexLock damping_lock;
    H3DUtil::MutexLock static_friction_lock;
    H3DUtil::MutexLock dynamic_friction_lock;
  };
}

#endif
