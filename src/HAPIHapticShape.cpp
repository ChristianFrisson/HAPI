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
/// \file HAPISurfaceObject.cpp
/// \brief cpp file for HAPISurfaceObject
///
//
//////////////////////////////////////////////////////////////////////////////

#include <HAPIHapticShape.h>

using namespace HAPI;

int HAPIHapticShape::current_max_id = 0;
list< int > HAPIHapticShape::free_ids;

int HAPIHapticShape::genShapeId() {
  if( free_ids.empty() ) {
    return current_max_id++;
  } else {
    int id = free_ids.front();
    free_ids.pop_front();
    return id;
  }
}

void HAPIHapticShape::delShapeId( int id ) {
  // todo: remove all resources used by id, e.g. hl shape id
  free_ids.push_front( id );
}
