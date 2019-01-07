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
/// \file HapticForceField.cpp
/// \brief cpp file for HapticForceField
///
//
//////////////////////////////////////////////////////////////////////////////

#include <HAPI/HapticForceField.h> 

using namespace HAPI;

HapticForceField::HapticForceField( const Vec3 &_force,
                                    const Vec3 &_torque,
                                    bool _use_ref_count_lock ):
  HAPIForceEffect( _use_ref_count_lock ),
  force( _force ),
  torque( _torque ) {}


