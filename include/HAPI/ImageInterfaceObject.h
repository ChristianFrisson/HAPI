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
/// \file ImageInterfaceObject.h
/// \brief Header file for ImageInterfaceObject
///
//
//////////////////////////////////////////////////////////////////////////////
#ifndef __IMAGEINTERFACEOBJECT_H__
#define __IMAGEINTERFACEOBJECT_H__

#include <Image.h>
#include <AutoRef.h>
#include <HAPI/HAPI.h>
#include <HAPI/HAPITypes.h>

namespace HAPI {
  /// \ingroup HAPINodes
  /// \class ImageInterfaceObject
  /// Base class for HAPIObjects that want access to information in an image.
  /// It is optimized for gray scale 8 bit images and slightly faster than 
  /// using Image functinos directly in other cases.
  class HAPI_API ImageInterfaceObject {

  public:

    // Constructor
    ImageInterfaceObject( H3DUtil::Image * _image_object = 0 );

    // Set internal values thread safe.
    inline void setImage( H3DUtil::Image * image ) {
      if( image_lock )
        image_lock->lock();
      image_object.reset( image );
      image_data = 0;
      if( image ) {
        image_data = (unsigned char *) image->getImageData();
        byte_rem = image->bitsPerPixel() % 8;
        bytes_per_pixel = image->bitsPerPixel() / 8;
        isUnsignedLuminance = image->pixelType() == H3DUtil::Image::LUMINANCE &&
                              image->pixelComponentType() == H3DUtil::Image::UNSIGNED &&
                              bytes_per_pixel == 1;
        width = image->width();
        height = image->height();
        depth = image->depth();
      }
      if( image_lock )
        image_lock->unlock();
    }

  protected:
    /// Sample the image at a given normalized position(texture coordinate), 
    /// i.e. coordinates between 0 and 1. Pixel data will be trilinearly
    /// interpolated to  calculate the result.
    ///
    /// \param tex_coord The texture coordinate from which to get data.
    HAPIFloat getSample( Vec3 &tex_coord );

    /// Help function to getSample
    HAPIFloat getPixel( int x = 0, int y = 0, int z = 0 );

    /// Help function to getValue
    HAPIFloat getValue( int x = 0, int y = 0, int z = 0 );

    /// Help function to getValue
    void getElement( void *value, int x = 0, int y = 0, int z = 0 );

    H3DUtil::AutoRef< H3DUtil::Image > image_object;

    /// Pointer to a lock. ImageInterfaceObject does not have its own lock
    /// because of the simple fact that getSample might be called up to 100
    /// times per haptic loop. Subclasses to ImageInterfaceObject can set this
    /// pointer and use its own lock to lock before all calls to getSample.
    H3DUtil::MutexLock *image_lock;

    /// Help variables for faster access.
    bool isUnsignedLuminance;
    unsigned char *image_data;
    unsigned int byte_rem;
    unsigned int bytes_per_pixel;
    unsigned int width;
    unsigned int height;
    unsigned int depth;

  };
}

#endif
