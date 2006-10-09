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
//
//////////////////////////////////////////////////////////////////////////////

#ifndef __H3DTYPES_H__
#define __H3DTYPES_H__

#include <Types/H3DBasicTypes.h>
#include <Types/H3DTypeOperators.h>

namespace H3D {

  // move the Arithmetic types into the H3D namespace.
  using ArithmeticTypes::Vec2f;
  using ArithmeticTypes::Vec2d;
  using ArithmeticTypes::Vec3f;
  using ArithmeticTypes::Vec3d;
  using ArithmeticTypes::Vec4f;
  using ArithmeticTypes::Vec4d;
  using ArithmeticTypes::Rotation;
  using ArithmeticTypes::Matrix3f;
  using ArithmeticTypes::Matrix3d;
  using ArithmeticTypes::Matrix4f;
  using ArithmeticTypes::Matrix4d;
  using ArithmeticTypes::Quaternion;
#undef RGB
  /// \defgroup BasicTypes Basic types.
  /// The basic types used in the H3D API.
  /// \{

  /// Structure for RGB values.
  struct RGB {
    /// Constructor.
    inline RGB():
      r( (H3DFloat) 0 ),
      g( (H3DFloat) 0 ),
      b( (H3DFloat) 0 ) {}

    /// Constructor.
    inline RGB( H3DFloat _r,
                H3DFloat _g,
                H3DFloat _b ): r(_r), g(_g), b(_b) {}

    /// Returns a HSV color scheme representation of the RGB color.
    inline Vec3f toHSV() const {
      int i; 	
      H3DFloat v, x, f; 	
      x = H3DMin(r, H3DMin( g, b ) ); 	
      v = H3DMax(r, H3DMax( g, b ) ); 	
      if(v == x) return Vec3f(0, 0, v); 	
      f = (r == x) ? g - b : ((g == x) ? b - r : r - g); 	
      i = (r == x) ? 3 : ((g == x) ? 5 : 1); 	
      return Vec3f(i - f /(v - x), (v - x)/v, v);
    }

    /// Sets the RGB from a HSV color scheme representation.
    inline void fromHSV( const Vec3f &hsv ) {
      H3DFloat h = hsv.x, s = hsv.y, v = hsv.z, m, n, f; 	
      int i; 	
      if( h == 0.0 ) r = g = b = v;
      i = (int) H3DFloor(h); 	
      f = h - i; 	
      if(!(i & 1)) f = 1 - f; // if i is even 	
      m = v * (1 - s); 	
      n = v * (1 - s * f); 	
      switch (i) { 	
      case 6: 	
      case 0: r = v; g = n; b = m; break;
      case 1: r = n; g = v; b = m; break;
      case 2: r = m; g = v; b = n; break;
      case 3: r = m; g = n; b = v; break;
      case 4: r = n; g = m; b = v; break;
      case 5: r = v; g = m; b = n; break;
        // should never happen
      default: r = g = b = 0;
      }
    }

    H3DFloat r, g, b;
  }; 
    
  /// Structure for RGBA values.
  struct RGBA {
    /// Constructor.
    inline RGBA():
      r( (H3DFloat) 0 ),
      g( (H3DFloat) 0 ),
      b( (H3DFloat) 0 ),
      a( (H3DFloat) 0 ) {}

	  /// Constructor.
    RGBA( H3DFloat _r,
          H3DFloat _g,
          H3DFloat _b,
          H3DFloat _a ) : r(_r), g(_g), b(_b), a(_a) {}
    H3DFloat r, g, b, a;
  };

  /// Function for printing a RGB to an ostream.
  inline ostream& operator<<( ostream &os, const RGB &r ) {
    os << r.r << " " << r.g << " " << r.b;
    return os;
  }
  
  /// Function for printing a Rotation to an ostream.
  inline ostream& operator<<( ostream &os, const RGBA &r ) {
    os << r.r << " " << r.g << " " << r.b << " " << r.a;
    return os;
  }

  /// Test two RGB instances for equality.
  inline bool operator==( const RGB &r1, const RGB &r2 ) {
    return r1.r == r2.r && r1.g == r2.g && r1.b == r2.b;
  }

  /// Test two RGBA instances for equality.
  inline bool operator==( const RGBA &r1, const RGBA &r2 ) {
    return r1.r == r2.r && r1.g == r2.g && r1.b == r2.b && r1.a == r2.a;
  }

  /// Test two RGB instances for inequality.
  inline bool operator!=( const RGB &r1, const RGB &r2 ) {
    return !(r1==r2);
  }

  /// Test two RGBA instances for inequality.
  inline bool operator!=( const RGBA &r1, const RGBA &r2 ) {
    return !(r1==r2);
  }

  /// \}
}

#endif
