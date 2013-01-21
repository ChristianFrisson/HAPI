//////////////////////////////////////////////////////////////////////////////
//    Copyright 2004-2013, SenseGraphics AB
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
/// \file HAPIForceEffect.cpp
/// \brief cpp file for HAPIForceEffect
///
//
//////////////////////////////////////////////////////////////////////////////
#include <HAPI/HAPIForceEffect.h>
#include <HAPI/HAPIHapticsDevice.h>
using namespace HAPI;

HAPIForceEffect::EffectInput::EffectInput( HAPIHapticsDevice *_hd,
                                           const TimeStamp &dt ) :
                                           deltaT( dt ),
                                           hd( _hd ){
  position = hd->getPosition();
  velocity = hd->getVelocity();
  orientation = hd->getOrientation();
}
