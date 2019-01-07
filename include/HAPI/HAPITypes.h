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
/// \file HAPITypes.h
/// \brief Header file for types in the HAPI namespace.
//
//////////////////////////////////////////////////////////////////////////////

#ifndef __HAPITYPES_H__
#define __HAPITYPES_H__

#include <H3DUtil/LinAlgTypes.h>
#include <H3DUtil/TimeStamp.h>
#include <H3DUtil/H3DMath.h>

#include <HAPI/HAPI.h>

namespace HAPI {

  /// Namespace containing various useful constants
  namespace Constants {
    /// \brief epsilon for HAPI.
    const double epsilon = 1e-13;
  }
  
  using H3DUtil::TimeStamp;
  /// \brief Integer in HAPI.
  typedef int HAPIInt32;
  /// \brief Time in HAPI.
  typedef double HAPITime;
  using H3DUtil::RGBA;
  using H3DUtil::H3DMax;
  using H3DUtil::H3DMin;

#ifdef USE_DOUBLE_PRECISION
  /// \brief HAPI use double precision. Vec2 is a H3DUtil::Vec2d.
  typedef H3DUtil::Vec2d Vec2;
  /// \brief HAPI use double precision. Vec3 is a H3DUtil::Vec3d.
  typedef H3DUtil::Vec3d Vec3;
  /// \brief HAPI use double precision. Vec4 is a H3DUtil::Vec4d.
  typedef H3DUtil::Vec4d Vec4;
  /// \brief HAPI use double precision. Matrix3 is a H3DUtil::Matrix3d.
  typedef H3DUtil::Matrix3d Matrix3;
  /// \brief HAPI use double precision. Matrix4 is a H3DUtil::Matrix4d.
  typedef H3DUtil::Matrix4d Matrix4;
  /// \brief HAPI use double precision. HAPIFloat is a double.
  typedef double HAPIFloat ;
  /// \brief HAPI use double precision. Rotation is a H3DUtil::Rotationd.
  typedef H3DUtil::ArithmeticTypes::Rotationd Rotation;
  /// \brief HAPI use double precision. Quaternion is a H3DUtil::Quaterniond.
  typedef H3DUtil::ArithmeticTypes::Quaterniond Quaternion;
#else
  /// \brief HAPI use float precision. Vec2 is a H3DUtil::Vec2f.
  typedef H3DUtil::Vec2f Vec2;
  /// \brief HAPI use float precision. Vec3 is a H3DUtil::Vec3f.
  typedef H3DUtil::Vec3f Vec3;
  /// \brief HAPI use float precision. Vec4 is a H3DUtil::Vec4f.
  typedef H3DUtil::Vec4f Vec4;
  /// \brief HAPI use float precision. Matrix3 is a H3DUtil::Matrix3f.
  typedef H3DUtil::Matrix3f Matrix3;
  /// \brief HAPI use float precision. Matrix4 is a H3DUtil::Matrix4f.
  typedef H3DUtil::Matrix4f Matrix4;
  /// \brief HAPI use float precision. HAPIFloat is a float.
  typedef float HAPIFloat ;
  /// \brief HAPI use float precision. Rotation is a H3DUtil::Rotation.
  typedef H3DUtil::ArithmeticTypes::Rotation Rotation;
  /// \brief HAPI use float precision. Quaternion is a H3DUtil::Quaternion.
  typedef H3DUtil::ArithmeticTypes::Quaternion Quaternion;
#endif

  // defining "types" for doxygen. That is, different groups used to
  // give better structured documentation.

  /// \defgroup HAPIClasses HAPI classes
  /// All grouped classes in HAPI should be in this group.

  /// \ingroup HAPIClasses
  /// \defgroup AbstractClasses Abstract Classes
  /// An abstract class is a common base class for other classes.

  /// \ingroup HAPIClasses
  /// \defgroup Renderers Haptic geometry rendering
  /// These classes takes care of geometry based haptic rendering in HAPI.

  /// \ingroup HAPIClasses
  /// \defgroup ForceEffects Haptics force effects
  /// These classes implements some kind of force effect to be renderered
  /// on the haptics device.

  /// \ingroup HAPIClasses
  /// \defgroup Shapes Haptic geometries
  /// These classes can be used by a renderer to allow for geometry based
  /// haptic rendering in HAPI.

  /// \ingroup HAPIClasses
  /// \defgroup CollisionStructures Collision structures
  /// These classes contains functions and structures to simplify collision
  /// detection for other classes. 

  /// \ingroup HAPIClasses
  /// \defgroup Surfaces Haptic surfaces
  /// These classes are used to define the behaviour on the surface of
  /// shapes.

  /// \ingroup HAPIClasses
  /// \defgroup HapticsDevices Haptics devices
  /// These classes interfaces to a haptics device of some kind.

  /// \ingroup HAPIClasses
  /// \defgroup Others Other classes
  /// These classes does not fit in any other categories and can be any kind
  /// of class used as help functions or interfaces to other classes.
}

#endif
