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
/// \file HAPI/src/DeviceLog.cpp
/// \brief CPP file for DeviceLog class.
//
//
//////////////////////////////////////////////////////////////////////////////
#include <HAPI/DeviceLog.h>
#include <HAPI/HAPIHapticsDevice.h>
#include <H3DUtil/ReadWriteH3DTypes.h>
#include <H3DUtil/Console.h>

using namespace HAPI;

DeviceLog::DeviceLog( const std::string &_log_file, 
                      const DeviceLog::LogTypeVector &_log_type,
                      int _freq, bool _binary, bool _use_ref_count_lock ) :
  HAPIForceEffect( _use_ref_count_lock ), log_type( _log_type ), binary( _binary ) {

  std::ios_base::openmode mode = std::ios::out | std::ios::trunc;
  if( binary )
    mode = mode | std::ios::binary;

  log_file.open( _log_file.c_str(), mode );
  if( !log_file.is_open() )
    H3DUtil::Console(H3DUtil::LogLevel::Warning) << "The _log_file argument to HAPI::DeviceLog "
                        << "does not point "
                        << "to a valid path, the file could not be created.";
  last_time = -1;
  time_diff = 1.0/_freq;
}

HAPI::HAPIForceEffect::EffectOutput DeviceLog::calculateForces(
  const HAPI::HAPIForceEffect::EffectInput &input ) {
  if( log_file.is_open() ) {
    if( last_time < 0 ) {
      start_time = TimeStamp::now();
      last_time = start_time;
      writeHeader( input );
      writeLog( input, 0 );
    } else {
      HAPITime this_time = TimeStamp::now();
      if( this_time - last_time >= time_diff ) {
        last_time = this_time;
        writeLog( input, this_time - start_time );
      }
    }
  }
  return EffectOutput();
}

void DeviceLog::close () {
  H3DUtil::HapticThread::synchronousHapticCB ( closeCallback, this );
}

const HAPIHapticsDevice::DeviceValues& HAPI::DeviceLog::getLastLoggedDeviceValues() {
  return last_logged_device_values;
}

const HAPIHapticsDevice::DeviceValues& HAPI::DeviceLog::getLastLoggedRawDeviceValues() {
  return last_logged_raw_device_values;
}

void DeviceLog::writeLog( const EffectInput &input, HAPITime log_time ) {
  writeLogRow ( input, log_time );

  if( !binary ) {
    log_file << std::endl;
  }
}

void DeviceLog::writeHeader( const EffectInput &input ) {
  writeHeaderRow ( input );

  if( binary ) {
    char null_char = '\0';
    log_file.write( &null_char, sizeof(char) );
  } else {
    log_file << std::endl;
  }
}

void DeviceLog::writeLogRow ( const EffectInput &input, HAPITime log_time ) {
  HAPIHapticsDevice::DeviceValues cdv = input.hd->getDeviceValues();
  HAPIHapticsDevice::DeviceValues crdv = input.hd->getRawDeviceValues();
  last_logged_device_values = cdv;
  last_logged_raw_device_values = crdv;
  if( binary ) {
    for( LogTypeVector::iterator i = log_type.begin();
         i != log_type.end(); ++i ) {
      switch( *i ) {
        case TIME: {
          H3DUtil::writeH3DType( log_file, log_time );
          break;
        }
        case POSITION: {
          H3DUtil::writeH3DType( log_file, cdv.position );
          break;
        }
        case ORIENTATION: {
          H3DUtil::writeH3DType( log_file, cdv.orientation );
          break;
        }
        case VELOCITY: {
          H3DUtil::writeH3DType( log_file, cdv.velocity );
          break;
        }
        case FORCE: {
          H3DUtil::writeH3DType( log_file, cdv.force );
          break;
        }
        case TORQUE: {
          H3DUtil::writeH3DType( log_file, cdv.torque );
          break;
        }
        case BUTTONS: {
          H3DUtil::writeH3DType( log_file, cdv.button_status );
          break;
        }
        case RAW_POSITION: {
          H3DUtil::writeH3DType( log_file, crdv.position );
          break;
        }
        case RAW_ORIENTATION: {
          H3DUtil::writeH3DType( log_file, crdv.orientation );
          break;
        }
        case RAW_VELOCITY: {
          H3DUtil::writeH3DType( log_file, crdv.velocity );
          break;
        }
        case RAW_FORCE: {
          H3DUtil::writeH3DType( log_file, crdv.force );
          break;
        }
        case RAW_TORQUE: {
          H3DUtil::writeH3DType( log_file, crdv.torque );
          break;
        }
        case TIMESTAMP: {
          H3DUtil::writeH3DType( log_file, TimeStamp() );
          break;
        }
        default: {}
      }
    }
  } else {
    for( LogTypeVector::iterator i = log_type.begin();
         i != log_type.end(); ++i ) {
      if( i != log_type.begin() )
        log_file << " ";
      switch( *i ) {
        case TIME: {
          log_file << log_time;
          break;
        }
        case POSITION: {
          log_file << cdv.position;
          break;
        }
        case ORIENTATION: {
          log_file << cdv.orientation;
          break;
        }
        case VELOCITY: {
          log_file << cdv.velocity;
          break;
        }
        case FORCE: {
          log_file << cdv.force;
          break;
        }
        case TORQUE: {
          log_file << cdv.torque;
          break;
        }
        case BUTTONS: {
          log_file << cdv.button_status;
          break;
        }
        case RAW_POSITION: {
          log_file << crdv.position;
          break;
        }
        case RAW_ORIENTATION: {
          log_file << crdv.orientation;
          break;
        }
        case RAW_VELOCITY: {
          log_file << crdv.velocity;
          break;
        }
        case RAW_FORCE: {
          log_file << crdv.force;
          break;
        }
        case RAW_TORQUE: {
          log_file << crdv.torque;
          break;
        }
        case TIMESTAMP: {
          log_file << TimeStamp();
          break;
        }
        default: {}
      }
    }
  }
}

void DeviceLog::writeHeaderRow ( const EffectInput & /*input*/ ) {
  if( binary ) {
    for( LogTypeVector::iterator i = log_type.begin();
         i != log_type.end(); ++i ) {
      switch( *i ) {
        case TIME: {
          H3DUtil::writeH3DType( log_file, "TIME" );
          break;
        }
        case POSITION: {
          H3DUtil::writeH3DType( log_file, "POSITION" );
          break;
        }
        case ORIENTATION: {
          H3DUtil::writeH3DType( log_file, "ORIENTATION" );
          break;
        }
        case VELOCITY: {
          H3DUtil::writeH3DType( log_file, "VELOCITY" );
          break;
        }
        case FORCE: {
          H3DUtil::writeH3DType( log_file, "FORCE" );
          break;
        }
        case TORQUE: {
          H3DUtil::writeH3DType( log_file, "TORQUE" );
          break;
        }
        case BUTTONS: {
          H3DUtil::writeH3DType( log_file, "BUTTONS" );
          break;
        }
        case RAW_POSITION: {
          H3DUtil::writeH3DType( log_file, "RAW_POSITION" );
          break;
        }
        case RAW_ORIENTATION: {
          H3DUtil::writeH3DType( log_file, "RAW_ORIENTATION" );
          break;
        }
        case RAW_VELOCITY: {
          H3DUtil::writeH3DType( log_file, "RAW_VELOCITY" );
          break;
        }
        case RAW_FORCE: {
          H3DUtil::writeH3DType( log_file, "RAW_FORCE" );
          break;
        }
        case RAW_TORQUE: {
          H3DUtil::writeH3DType( log_file, "RAW_TORQUE" );
          break;
        }
        case TIMESTAMP: {
          H3DUtil::writeH3DType( log_file, "TIMESTAMP" );
          break;
        }
        default: {}
      }
    }
  } else {
    log_file.precision ( 5 );
    log_file << std::fixed;
    for( LogTypeVector::iterator i = log_type.begin();
         i != log_type.end(); ++i ) {
      if( i != log_type.begin() )
        log_file << " ";
      switch( *i ) {
        case TIME: {
          log_file << "TIME";
          break;
        }
        case POSITION: {
          log_file << "POSITION";
          break;
        }
        case ORIENTATION: {
          log_file << "ORIENTATION";
          break;
        }
        case VELOCITY: {
          log_file << "VELOCITY";
          break;
        }
        case FORCE: {
          log_file << "FORCE";
          break;
        }
        case TORQUE: {
          log_file << "TORQUE";
          break;
        }
        case BUTTONS: {
          log_file << "BUTTONS";
          break;
        }
        case RAW_POSITION: {
          log_file << "RAW_POSITION";
          break;
        }
        case RAW_ORIENTATION: {
          log_file << "RAW_ORIENTATION";
          break;
        }
        case RAW_VELOCITY: {
          log_file << "RAW_VELOCITY";
          break;
        }
        case RAW_FORCE: {
          log_file << "RAW_FORCE";
          break;
        }
        case RAW_TORQUE: {
          log_file << "RAW_TORQUE";
          break;
        }
        case TIMESTAMP: {
          log_file << "TIMESTAMP";
          break;
        }
        default: {}
      }
    }
  }
}

H3DUtil::PeriodicThread::CallbackCode DeviceLog::closeCallback ( void* data ) {
  DeviceLog* o= static_cast < DeviceLog* > ( data );

  if( o->log_file.is_open() ) {
    o->log_file.close();
  }

  return H3DUtil::PeriodicThread::CALLBACK_DONE;
}