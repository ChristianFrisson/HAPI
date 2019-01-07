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
/// \file PlaneConstraint.cpp
/// \brief CPP file for PlaneConstraint
///
//
//
//////////////////////////////////////////////////////////////////////////////

#include <HAPI/PlaneConstraint.h>

using namespace HAPI;
using namespace Collision;

bool PlaneConstraint::lineIntersect( const Vec3 &from, 
                                     const Vec3 &to,
                                     Collision::IntersectionInfo &result ) {
      Vec3 from_to = to - from;
      HAPIFloat denom = normal * from_to;
      if( denom > Constants::epsilon ) { 
        return false;
      }


      if( denom * denom < Constants::epsilon ) {
        return false;
      }
      HAPIFloat u = ( normal * ( point - from ) ) / denom; 
      if( u <= 1 + Constants::epsilon && u >= 0 - Constants::epsilon ) {

        if( u < 0 ) u = 0;
        if( u > 1 ) u = 1;

        result.point = from + u * from_to;
        result.t = u;
        result.normal = normal;

        result.tex_coord = tex_coord;
        return true;
      }
      return false;
}
