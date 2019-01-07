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
/// \file ImageInterfaceObject.cpp
/// \brief cpp file for ImageInterfaceObject
///
//
//////////////////////////////////////////////////////////////////////////////
#include <HAPI/ImageInterfaceObject.h>

using namespace HAPI;

ImageInterfaceObject::ImageInterfaceObject( H3DUtil::Image * _image_object ) :
  image_object( _image_object ) {
  image_lock = 0;
  if( image_object.get() ) {
    image_data = (unsigned char *) image_object->getImageData();
    byte_rem = image_object->bitsPerPixel() % 8;
    bytes_per_pixel = image_object->bitsPerPixel() / 8;
    isUnsignedLuminance = image_object->pixelType() ==
                          H3DUtil::Image::LUMINANCE &&
                          image_object->pixelComponentType() ==
                          H3DUtil::Image::UNSIGNED &&
                          bytes_per_pixel == 1;
    width = image_object->width();
    height = image_object->height();
    depth = image_object->depth();
  }
}

ImageInterfaceObject::ImageInterfaceObject( H3DUtil::Image * _image_object,
                                           H3DUtil::MutexLock *_image_lock ) :
  image_object( _image_object ) {
  image_lock = _image_lock;
  if( image_object.get() ) {
    image_data = (unsigned char *) image_object->getImageData();
    byte_rem = image_object->bitsPerPixel() % 8;
    bytes_per_pixel = image_object->bitsPerPixel() / 8;
    isUnsignedLuminance = image_object->pixelType() ==
                          H3DUtil::Image::LUMINANCE &&
                          image_object->pixelComponentType() ==
                          H3DUtil::Image::UNSIGNED &&
                          bytes_per_pixel == 1;
    width = image_object->width();
    height = image_object->height();
    depth = image_object->depth();
  }
}

HAPIFloat ImageInterfaceObject::getSample( Vec3 &tex_coord ) {

  HAPIFloat val = 0;

  if( image_object.get() )
  {
    
    assert( byte_rem == 0 );
    
    HAPIFloat px = tex_coord.x * width - 0.5f;
    HAPIFloat py = tex_coord.y * height- 0.5f;
    HAPIFloat pz = tex_coord.z * depth - 0.5f;

    if( px < 0 ) px = 0;
    if( py < 0 ) py = 0;
    if( pz < 0 ) pz = 0;

    HAPIFloat fx = H3DUtil::H3DFloor( px );
    HAPIFloat fy = H3DUtil::H3DFloor( py );
    HAPIFloat fz = H3DUtil::H3DFloor( pz );

    HAPIFloat cx = H3DUtil::H3DCeil( px );
    HAPIFloat cy = H3DUtil::H3DCeil( py );
    HAPIFloat cz = H3DUtil::H3DCeil( pz );

    if( cx >= width ) --cx;
    if( cy >= height ) --cy;
    if( cz >= depth ) --cz;

    HAPIFloat xd = px - fx;
    HAPIFloat yd = py - fy;
    HAPIFloat zd = pz - fz;
    
    // interpolate in z
    if( !isUnsignedLuminance ) {
      HAPIFloat fff = getPixel( (int)fx, (int)fy, (int)fz );
      HAPIFloat ffc = getPixel( (int)fx, (int)fy, (int)cz );
      HAPIFloat fcf = getPixel( (int)fx, (int)cy, (int)fz );
      HAPIFloat fcc = getPixel( (int)fx, (int)cy, (int)cz );
      HAPIFloat cff = getPixel( (int)cx, (int)fy, (int)fz );
      HAPIFloat cfc = getPixel( (int)cx, (int)fy, (int)cz );
      HAPIFloat ccf = getPixel( (int)cx, (int)cy, (int)fz );
      HAPIFloat ccc = getPixel( (int)cx, (int)cy, (int)cz );

      HAPIFloat i1 = fff * (1-zd) + ffc * zd;
      HAPIFloat i2 = fcf * (1-zd) + fcc * zd;
      HAPIFloat j1 = cff * (1-zd) + cfc * zd;
      HAPIFloat j2 = ccf * (1-zd) + ccc * zd;

      HAPIFloat w1 = i1 * (1-yd) + i2 * yd;
      HAPIFloat w2 = j1 * (1-yd) + j2 * yd;

      HAPIFloat v = w1 * (1-xd) + w2 * xd;

      val = v;
    } else {
      
      HAPIFloat fff = getValue( (int)fx, (int)fy, (int)fz );
      HAPIFloat ffc = getValue( (int)fx, (int)fy, (int)cz );
      HAPIFloat fcf = getValue( (int)fx, (int)cy, (int)fz );
      HAPIFloat fcc = getValue( (int)fx, (int)cy, (int)cz );
      HAPIFloat cff = getValue( (int)cx, (int)fy, (int)fz );
      HAPIFloat cfc = getValue( (int)cx, (int)fy, (int)cz );
      HAPIFloat ccf = getValue( (int)cx, (int)cy, (int)fz );
      HAPIFloat ccc = getValue( (int)cx, (int)cy, (int)cz );

      HAPIFloat i1 = fff * (1-zd) + ffc * zd;
      HAPIFloat i2 = fcf * (1-zd) + fcc * zd;
      HAPIFloat j1 = cff * (1-zd) + cfc * zd;
      HAPIFloat j2 = ccf * (1-zd) + ccc * zd;

      HAPIFloat w1 = i1 * (1-yd) + i2 * yd;
      HAPIFloat w2 = j1 * (1-yd) + j2 * yd;

      HAPIFloat v = w1 * (1-xd) + w2 * xd;

      val = v;

    }

  }

  return val;

}

HAPIFloat ImageInterfaceObject::getValue( int x, int y, int z ) {
  unsigned long v = image_data[ ( ( z * height + y ) * width + x ) ];
  return v / 255.0f;
}

HAPIFloat ImageInterfaceObject::getPixel( int x, int y, int z ) {
  assert( byte_rem == 0 );
  if( bytes_per_pixel <= 8 ) {
    char pixel_data[8];
        
    getElement( pixel_data, x, y, z );
    
    H3DUtil::RGBA rgba = image_object->imageValueToRGBA( pixel_data );
    
    return rgba.r;
  } else {
    char *pixel_data = new char[bytes_per_pixel];
    getElement( pixel_data, x, y, z );
    H3DUtil::RGBA rgba = image_object->imageValueToRGBA( pixel_data );
    delete [] pixel_data;
    return rgba.r;
  }
}

void ImageInterfaceObject::getElement( void *value, int x, int y, int z ) {
  assert( byte_rem == 0 );

  if( byte_rem != 0 ) {
    ++bytes_per_pixel;
  }

  memcpy( value, 
    &image_data[ ( ( z * height + y ) * width + x ) * bytes_per_pixel ],
    bytes_per_pixel );
}

