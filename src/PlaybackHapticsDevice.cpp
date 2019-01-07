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
/// \file PlaybackHapticsDevice.cpp
/// \brief Cpp file for PlaybackHapticsDevice.
///
//
//////////////////////////////////////////////////////////////////////////////


#include <HAPI/PlaybackHapticsDevice.h>
#include <H3DUtil/ReadWriteH3DTypes.h>

using namespace HAPI;

void PlaybackHapticsDevice::updateDeviceValues( DeviceValues &dv, HAPI::HAPITime dt ) {
  HAPIHapticsDevice::updateDeviceValues( dv, dt );

  playback_lock.lock();
  if ( playing ) {
    if ( !getPlaybackValuesAtTime ( dv, playback_speed * (H3DUtil::TimeStamp()-playback_start_time) ) ) {
      // end of playback or error
      playing= false;
    }
  } else {
    dv= playback_device_values;
  }
  playback_lock.unlock();
}

PlaybackHapticsDevice::StringList PlaybackHapticsDevice::readColumnNamesBinary () {
  StringList fields;

  int c= playback_file.get();
  while ( c != '\0' ) {
    std::string field_name;
    while ( c != '\0' ) {
      field_name+= c;
      c= playback_file.get();
    }
    fields.push_back ( field_name );
    c= playback_file.get();
  }

  return fields;
}

PlaybackHapticsDevice::StringList PlaybackHapticsDevice::readColumnNamesText () {
  StringList fields;

  std::string line;
  std::getline ( playback_file, line );

  std::istringstream iss(line);
  while ( iss ) {
    std::string field_name;
    iss >> field_name;
    if ( !field_name.empty() ) {
      fields.push_back ( field_name );
    }
  }

  return fields;
}

bool PlaybackHapticsDevice::loadRecording ( const std::string& _filename, bool _binary ) {
  playback_lock.lock();
  
  binary= _binary;
  if ( playback_file.is_open() ) {
    playback_file.close();
  }
  playback_file.open ( _filename.c_str(), std::ios::binary );
  if ( !playback_file.good() ) {
    return false;
  }

  // Read the column names
  if ( binary ) {
    field_names= readColumnNamesBinary();
  } else {
    field_names= readColumnNamesText();
  }

  data_start_pos= playback_file.tellg();
  playback_time= 0;

  playback_lock.unlock();
  
  return true;
}

void PlaybackHapticsDevice::closeRecording () {
  playback_lock.lock();

  playing= false;
  if ( playback_file.is_open() ) {
    playback_file.close();
  }

  playback_lock.unlock();
}

void PlaybackHapticsDevice::startPlayback () {
  playback_lock.lock();

  if ( !playing ) {
    if ( seek_to_time >= 0 ) {
      if ( seek_to_time < playback_time || playback_time < 0 ) {
        playback_file.clear();
        playback_file.seekg ( data_start_pos );
      }
      playback_time= seek_to_time;
      seek_to_time= -1;
    }

    playback_start_time= H3DUtil::TimeStamp()-playback_time;
    playing= true;
    got_first_value = false;
  }

  playback_lock.unlock();
}

void PlaybackHapticsDevice::stopPlayback () {
  playback_lock.lock();

  if ( playing ) {
    playing= false;
  }

  playback_lock.unlock();
}

void PlaybackHapticsDevice::seekToTime ( HAPITime _time ) {
  playback_lock.lock();
  seek_to_time= _time;
  playback_lock.unlock();
}

void PlaybackHapticsDevice::setPlaybackSpeed ( HAPIFloat _speed ) {
  playback_lock.lock();
  playback_speed= _speed<0 ? 0 : _speed;
  playback_lock.unlock();
}

HAPITime PlaybackHapticsDevice::getPlaybackTime () {
  return playback_time;
}

bool PlaybackHapticsDevice::isPlaying () {
  return playing;
}

void PlaybackHapticsDevice::setDefaultDeviceValues ( const HAPIHapticsDevice::DeviceValues& _dv ) {
  playback_lock.lock();
  playback_device_values= _dv;
  playback_lock.unlock();
}

void PlaybackHapticsDevice::addDataField ( const std::string& _name ) {
  use_field_names.push_back ( _name );
}

void PlaybackHapticsDevice::clearDataFields () {
  use_field_names.clear();
}

bool PlaybackHapticsDevice::getPlaybackValuesNext ( HAPIHapticsDevice::DeviceValues& _dv, HAPITime& _time, HAPITime& _timestamp ) {
  if ( binary ) {
    return getPlaybackValuesNextBinary ( _dv, _time, _timestamp );
  } else {
    return getPlaybackValuesNextText ( _dv, _time, _timestamp );
  }
}

bool PlaybackHapticsDevice::getPlaybackValuesNextText ( HAPIHapticsDevice::DeviceValues& _dv, HAPITime& _time, HAPITime& _timestamp ) {
  if ( !playback_file.good() ) {
    return false;
  }

  HAPIHapticsDevice::DeviceValues dv_tmp;
  HAPITime time_tmp= -1;
  HAPITime timestamp_tmp= -1;
  readFieldsValuesText ( dv_tmp, time_tmp, timestamp_tmp );

  if ( playback_file.good() ) {
    _time= time_tmp;
    _timestamp= timestamp_tmp;
    _dv= dv_tmp;
    return true;
  } else {
    return false;
  }
}

bool PlaybackHapticsDevice::getPlaybackValuesNextBinary ( HAPIHapticsDevice::DeviceValues& _dv, HAPITime& _time, HAPITime& _timestamp ) {
  if ( !playback_file.good() ) {
    return false;
  }

  HAPIHapticsDevice::DeviceValues dv_tmp;
  HAPITime time_tmp= -1;
  HAPITime timestamp_tmp= -1;
  readFieldsValuesBinary ( dv_tmp, time_tmp, timestamp_tmp );

  if ( playback_file.good() ) {
    _time= time_tmp;
    _timestamp= timestamp_tmp;
    _dv= dv_tmp;
    return true;
  } else {
    return false;
  }
}


bool PlaybackHapticsDevice::getPlaybackValuesAtTime ( HAPI::HAPIHapticsDevice::DeviceValues &dv, HAPI::HAPITime _time ) {
  if ( _time > playback_time || !got_first_value ) {
    got_first_value = true;
    DeviceValues dv_tmp= playback_device_values;

    HAPITime timestamp;
    bool success= getPlaybackValuesNext ( dv_tmp, playback_time, timestamp );
  
    while ( success && playback_time < _time && playback_time > 0 ) {
      success= getPlaybackValuesNext ( dv_tmp, playback_time, timestamp );
    }

    if ( use_field_names.empty() ) {
      playback_device_values= dv_tmp;
    } else {
      for ( StringList::iterator i= use_field_names.begin(); i != use_field_names.end(); ++i ) {
        std::string field_name= *i;
        
        if ( field_name == "BUTTONS" ) {
          playback_device_values.button_status= dv_tmp.button_status;
        }
        else if ( field_name == "RAW_POSITION" ) {
          playback_device_values.position= dv_tmp.position;
        }
        else if ( field_name == "RAW_ORIENTATION" ) {
          playback_device_values.orientation= dv_tmp.orientation;
        }
        else if ( field_name == "RAW_VELOCITY" ) {
          playback_device_values.velocity= dv_tmp.velocity;
        }
      }
    }

    dv= playback_device_values;
    return success;
  } else {
    dv= playback_device_values;
    return true;
  }
}

void PlaybackHapticsDevice::readFieldsValuesBinary ( HAPIHapticsDevice::DeviceValues& _dv, HAPITime& _time, HAPITime& _timestamp ) { 
  for ( StringList::iterator i= field_names.begin(); i != field_names.end(); ++i ) {
    std::string field_name= *i;

    if ( field_name == "TIME" ) {
      H3DUtil::readH3DType ( playback_file, _time );
    }
    else if ( field_name == "POSITION" ) {
      HAPI::Vec3 p;
      H3DUtil::readH3DType ( playback_file, p );
    }
    else if ( field_name == "ORIENTATION" ) {
      HAPI::Rotation r;
      H3DUtil::readH3DType ( playback_file, r );
    }
    else if ( field_name == "VELOCITY" ) {
      HAPI::Vec3 v;
      H3DUtil::readH3DType ( playback_file, v );
    }
    else if ( field_name == "FORCE" ) {
      HAPI::Vec3 f;
      H3DUtil::readH3DType ( playback_file, f );
    }
    else if ( field_name == "TORQUE" ) {
      HAPI::Vec3 t;
      H3DUtil::readH3DType ( playback_file, t );
    }
    else if ( field_name == "BUTTONS" ) {
      H3DUtil::readH3DType ( playback_file, _dv.button_status );
    }
    else if ( field_name == "RAW_POSITION" ) {
      H3DUtil::readH3DType ( playback_file, _dv.position );
    }
    else if ( field_name == "RAW_ORIENTATION" ) {
      H3DUtil::readH3DType ( playback_file, _dv.orientation );
    }
    else if ( field_name == "RAW_VELOCITY" ) {
      H3DUtil::readH3DType ( playback_file, _dv.velocity );
    }
    else if ( field_name == "RAW_FORCE" ) {
      H3DUtil::readH3DType ( playback_file, _dv.force );
    }
    else if ( field_name == "RAW_TORQUE" ) {
      HAPI::Vec3 t;
      H3DUtil::readH3DType ( playback_file, t );
    }
    else if ( field_name == "TIMESTAMP" ) {
      H3DUtil::readH3DType ( playback_file, _timestamp );
    }
  }
}

void PlaybackHapticsDevice::readFieldsValuesText ( HAPIHapticsDevice::DeviceValues& _dv, HAPITime& _time, HAPITime& _timestamp ) {
  for ( StringList::iterator i= field_names.begin(); i != field_names.end(); ++i ) {
    std::string field_name= *i;

    if ( field_name == "TIME" ) {
      playback_file >> _time;
    }
    else if ( field_name == "POSITION" ) {
      HAPIFloat f;
      playback_file >> f; playback_file >> f; playback_file >> f;
    }
    else if ( field_name == "ORIENTATION" ) {
      HAPIFloat f;
      playback_file >> f; playback_file >> f; playback_file >> f; playback_file >> f;
    }
    else if ( field_name == "VELOCITY" ) {
      HAPIFloat f;
      playback_file >> f; playback_file >> f; playback_file >> f;
    }
    else if ( field_name == "FORCE" ) {
      HAPIFloat f;
      playback_file >> f; playback_file >> f; playback_file >> f;
    }
    else if ( field_name == "TORQUE" ) {
      HAPIFloat f;
      playback_file >> f; playback_file >> f; playback_file >> f;
    }
    else if ( field_name == "BUTTONS" ) {
      playback_file >> _dv.button_status;
    }
    else if ( field_name == "RAW_POSITION" ) {
      playback_file >> _dv.position.x; playback_file >> _dv.position.y; playback_file >> _dv.position.z;
    }
    else if ( field_name == "RAW_ORIENTATION" ) {
      playback_file >> _dv.orientation.axis.x; playback_file >> _dv.orientation.axis.y; playback_file >> _dv.orientation.axis.z; playback_file >> _dv.orientation.angle;
    }
    else if ( field_name == "RAW_VELOCITY" ) {
      playback_file >> _dv.velocity.x; playback_file >> _dv.velocity.y; playback_file >> _dv.velocity.z;
    }
    else if ( field_name == "RAW_FORCE" ) {
      HAPIFloat f;
      playback_file >> f; playback_file >> f; playback_file >> f;
    }
    else if ( field_name == "RAW_TORQUE" ) {
      HAPIFloat f;
      playback_file >> f; playback_file >> f; playback_file >> f;
    }
    else if ( field_name == "TIMESTAMP" ) {
      playback_file >> _timestamp;
    }
  }
}
