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
//
//////////////////////////////////////////////////////////////////////////////

#ifndef __HAPITYPES_H__
#define __HAPITYPES_H__

#include <H3DUtil/LinAlgTypes.h>
#include <H3DUtil/TimeStamp.h>

#include <HAPI/HAPI.h>

namespace HAPI {
  // TODO: double versions of Rotation and Quaternion
  typedef H3DUtil::Rotation Rotation;
  typedef H3DUtil::Quaternion Quaternion;

  namespace Constants {
    const double epsilon = 1e-13;
  }
  
  typedef H3DUtil::TimeStamp TimeStamp;
  typedef int HAPIInt32;
  typedef double HAPITime;
  typedef H3DUtil::RGBA RGBA;

#ifdef USE_DOUBLE_PRECISION
  typedef H3DUtil::Vec2d Vec2;
  typedef H3DUtil::Vec3d Vec3;
  typedef H3DUtil::Vec4d Vec4;
  typedef H3DUtil::Matrix3d Matrix3;
  typedef H3DUtil::Matrix4d Matrix4;
  typedef double HAPIFloat ;
#else
  typedef H3DUtil::Vec2f Vec2;
  typedef H3DUtil::Vec3f Vec3;
  typedef H3DUtil::Vec4f Vec4;
  typedef H3DUtil::Matrix3f Matrix3;
  typedef H3DUtil::Matrix4f Matrix4;
  typedef float HAPIFloat ;
#endif

  // defining "types" for doxygen. That is, different groups used to
  // give better structured documentation.

  /// \defgroup AbstractClasses Abstract Classes
  /// An abstract classe is a common base class for other classes.

  /// \defgroup Renderers Haptic geometry rendering
  /// These classes takes care of geometry based haptic rendering in HAPI.

  /// \defgroup ForceEffects Haptics force effects
  /// These classes implements some kind of force effect to be renderered
  /// on the haptics device.

  /// \defgroup Shapes Haptic geometries
  /// These classes can be used by a renderer to allow for geometry based
  /// haptic rendering in HAPI.

  /// \defgroup CollisionStructures Collision structures
  /// These classes contains functions and structures to simplify collision
  /// detection for other classes. 

  /// \defgroup Surfaces Haptic surfaces
  /// These classes are used to define the behaviour on the surface of
  /// shapes.

  /// \defgroup HapticsDevices Haptics devices
  /// These classes interfaces to a haptics device of some kind.

  /// \defgroup Others Other classes
  /// These classes does not fit in any other categories and can be any kind
  /// of class used as help functions or interfaces to other classes.
}

#endif
