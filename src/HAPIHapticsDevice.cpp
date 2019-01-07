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
/// \file HAPIHapticsDevice.cpp
/// \brief Cpp file for HAPIHapticsDevice.
///
//
//////////////////////////////////////////////////////////////////////////////

#include <HAPI/HAPIHapticsDevice.h>
#include <HAPI/GodObjectRenderer.h>
#include <H3DUtil/H3DTimer.h>

using namespace HAPI;

HAPIHapticsDevice::local_auto_ptr< 
    std::list< HAPIHapticsDevice::HapticsDeviceRegistration > 
  > HAPIHapticsDevice::registered_devices( NULL );
 
bool HAPIHapticsDevice::initialized = false; 


H3DUtil::PeriodicThread::CallbackCode
  HAPIHapticsDevice::transferObjectsCallback( 
  void *data ) {
#ifdef HAVE_PROFILER
    H3DUtil::H3DTimer::begin("haptic_profile_transferObject");
    H3DUtil::H3DTimer::stepBegin("transferObject_begin");
#endif
  HAPIHapticsDevice *hd = 
    static_cast< HAPIHapticsDevice * >( data );

  // update last transform for all shapes.
  for( unsigned int layer = 0; layer < hd->tmp_shapes.size(); ++layer ) {
    H3DTIMER_BEGIN("TransferObjectPerLayer");
    for( HapticShapeVector::const_iterator s = hd->tmp_shapes[layer].begin(); 
         s != hd->tmp_shapes[layer].end(); ++s ) {
      H3DTIMER_BEGIN("TransferObjectPerShape");
      HAPIHapticShape *shape = NULL;
      if( hd->current_shapes.size() > layer ) {
        for( HapticShapeVector::const_iterator current_s =
               hd->current_shapes[layer].begin();
             current_s != hd->current_shapes[layer].end(); ++current_s ) {
          if( (*s)->getShapeId() == (*current_s)->getShapeId() ) {
            shape = (*current_s);
            break;
          }
        }
      }
      // if we have a new shape we need to initialize it.
      if( (*s) != shape ) 
        (*s)->initializeTransfer( shape );
      H3DTIMER_END("TransferObjectPerShape");
    }
    H3DTIMER_END("TransferObjectPerLayer");
  }

  // shapes
  hd->current_shapes = hd->tmp_shapes;

  // force effects. A bit more confusing due to the fact that they
  // should be added with interpolation in some cases.
  bool already_transferred = false;
  // If set effects was called.
  H3DTIMER_BEGIN("HANDLE_SWITCHING_EFFECTS");
  if( hd->tmp_switching_effects ) {
    if( hd->tmp_switch_effects_duration > Constants::epsilon ) {
      hd->last_add_rem_effect_map.clear();
      hd->last_add_rem_effect_map.swap( hd->current_add_rem_effect_map );
      hd->last_force_effects.swap( hd->current_force_effects );
      hd->switch_effects_duration = hd->tmp_switch_effects_duration;
      hd->last_force_effect_change = TimeStamp();
      hd->switching_effects = true;
    } else {
      hd->last_force_effects.clear();
      hd->current_add_rem_effect_map.clear();
      hd->last_add_rem_effect_map.clear();
      hd->switching_effects = false;
    }
    hd->tmp_switching_effects = false;
    hd->current_force_effects = hd->tmp_current_force_effects;
    already_transferred = true;
  }
  H3DTIMER_END("HANDLE_SWITCHING_EFFECTS");
  H3DTIMER_BEGIN("HANDLE_ADDEFFECT");
  // If addEffect was called
  if( !hd->added_effects_indices.empty() ) {
    
    for( std::vector< std::pair< unsigned int, HAPITime > >::iterator i =
         hd->added_effects_indices.begin();
         i != hd->added_effects_indices.end(); ++i ) {
      unsigned int index = (*i).first;
      if( !already_transferred ) {
        // The current_force_effects vector has not been updated.
        hd->current_force_effects.push_back(
          hd->tmp_current_force_effects[ index ] );
        index = (unsigned int) hd->current_force_effects.size() - 1;
      }
      if( (*i).second > Constants::epsilon ) {
        hd->current_add_rem_effect_map[ index ] =
          PhaseInOut( (*i).second, TimeStamp() );
      }
    }
    hd->added_effects_indices.clear();
  }
  H3DTIMER_END("HANDLE_ADDEFFECT");
  H3DTIMER_BEGIN("HANDLE_REMOVEEFFECT");
  // If removeEffect was called.
  if( !hd->force_effects_to_remove.empty() ) {
    for( std::vector< std::pair< HAPIForceEffect *, HAPITime > >::iterator i =
         hd->force_effects_to_remove.begin();
         i != hd->force_effects_to_remove.end(); ++i ) {

      // Find index of HAPIForceEffect if it is rendered.
      unsigned int j = 0;
      for( HapticEffectVector::const_iterator k =
           hd->current_force_effects.begin();
           k != hd->current_force_effects.end(); ++k, ++j ) {
        if( (*i).first == (*k) )
        break;
      }

      if( j < hd->current_force_effects.size() ) {
        // If found
        if( (*i).second > Constants::epsilon ) {
          // If the time to phase out is over 0.
          PhaseInOut temp_phase_in_out( (*i).second, TimeStamp(), false );
          
          IndexTimeMap::iterator found =
            hd->current_add_rem_effect_map.find( j );
          if( found != hd->current_add_rem_effect_map.end() ) {
            // If the force effect still is being phased in.
            // Old time is taken to not suddenly phase out from
            // maximum to 0, but only from "phase in fraction" to zero.
            temp_phase_in_out.setMaxFraction(
              (*found).second.getMaxFraction() );
            hd->current_add_rem_effect_map.erase( found );
            // Update index of other effects that are phased in.
            // They should be lowered by 1. The reason for not
            // iterating from the beginning is because they are added
            // in increasing order of index.
            for( unsigned int k = j + 1;
                 k < hd->current_force_effects.size(); k++ ) {
              found = hd->current_add_rem_effect_map.find( k );
              if( found != hd->current_add_rem_effect_map.end() ) {
                hd->current_add_rem_effect_map[ k - 1 ] =
                  hd->current_add_rem_effect_map[ k ];
                hd->current_add_rem_effect_map.erase( k );
              }
            }
          }
          // Updated last_add_rem_effect_map and last_force_effects;
          hd->last_add_rem_effect_map[ (unsigned int) hd->last_force_effects.size() ] =
            temp_phase_in_out;
          hd->last_force_effects.push_back( (*i).first );
        }
        if( !already_transferred ) {
          hd->current_force_effects.erase( (*i).first );
        }
      }
    }
    hd->force_effects_to_remove.clear();
  }
  H3DTIMER_END("HANDLE_REMOVEEFFECT");
#ifdef HAVE_PROFILER
  H3DUtil::H3DTimer::stepEnd("transferObject_begin");
  std::stringstream temp_profiledResult;
  //sofa::helper::AdvancedTimer::end("haptic_profile");
  H3DUtil::H3DTimer::end("haptic_profile_transferObject",temp_profiledResult);

  hd->profiled_result_lock.lock();
  hd->profiled_result_haptic[0] = "timer_for_objectTransfer: \n"+temp_profiledResult.str();
  hd->profiled_result_lock.unlock();
  //std::cout<<"hatpic profile result:"<<temp<<std::endl;
#endif
  
  // Signal that transfer obejcts callback is done
  // Wake up threads that are waiting to write to the shared data
  hd->transfer_objects_cond.lock ();
  hd->transfer_objects_active= false;
  hd->transfer_objects_cond.signal ();
  hd->transfer_objects_cond.unlock ();

  return H3DUtil::PeriodicThread::CALLBACK_DONE;
}
#ifdef HAVE_PROFILER
// Callback function to enable timer
H3DUtil::PeriodicThread::CallbackCode
  HAPIHapticsDevice::enableTimerCallback(void *data){
    H3DUtil::H3DTimer::setEnabled("haptic_profile",true);
    H3DUtil::H3DTimer::setInterval("haptic_profile",1);
    H3DUtil::H3DTimer::setEnabled("haptic_profile_transferObject",true);
    H3DUtil::H3DTimer::setInterval("haptic_profile_transferObject",1);
    

    return H3DUtil::PeriodicThread::CALLBACK_DONE;
}
#endif
// Callback function for rendering forces on the 
// HAPIHapticsDevice.  
H3DUtil::PeriodicThread::CallbackCode
  HAPIHapticsDevice::hapticRenderingCallback( void *data ) {
#ifdef HAVE_PROFILER
  H3DUtil::H3DTimer::begin("haptic_profile");
  H3DUtil::H3DTimer::stepBegin("haptic_render");
#endif
  HAPIHapticsDevice *hd = 
    static_cast< HAPIHapticsDevice * >( data );
  if( hd->nr_haptics_loops > 100 ) {
    TimeStamp now = TimeStamp();
    unsigned int nr_loops = hd->nr_haptics_loops;
    hd->nr_haptics_loops = 0;
    TimeStamp dt = now - hd->last_hr_update;
    hd->haptics_rate = (unsigned int)( nr_loops / dt );
    hd->last_hr_update = now;
  }
  ++(hd->nr_haptics_loops);
  
  TimeStamp start_time = TimeStamp();
  TimeStamp dt = start_time - hd->last_loop_time;
  hd->last_loop_time = start_time;

  hd->updateDeviceValues( dt );
  
  HAPIForceEffect::EffectOutput _output;
  
  bool last_add_interpolate = !hd->last_add_rem_effect_map.empty();
  bool current_add_interpolate = !hd->current_add_rem_effect_map.empty();
  HAPIForceEffect::EffectInput input( hd, dt );
  if( hd->switching_effects || current_add_interpolate ||
      last_add_interpolate ) {
    // If there is any kind of interpolation needed to be done, it is taken
    // care of here.

    // If any force effect should be removed from last_force_effect it is
    // stored here. Only used if last_add_interpolate is true.
    std::list< HAPIForceEffect * > to_remove;
    TimeStamp now_time;

    // calculate the forces generated by the last active force effects.
    if( last_add_interpolate ) {
      int j = 0;
      int shift_index = 0;
      // If any of the forces should be phased in or out then
      // check it, if it should be removed remove it and update index
      // in the map last_add_rem_effect_map.
      for( HapticEffectVector::const_iterator i =
             hd->last_force_effects.begin();
           i != hd->last_force_effects.end();
           ++i, ++j ) {
        IndexTimeMap::iterator found = hd->last_add_rem_effect_map.find( j );
        if( found != hd->last_add_rem_effect_map.end() ) {
          HAPIFloat temp_fraction = (*found).second.getFraction( now_time );
          if( temp_fraction > 1 ) temp_fraction = 1;
          if( temp_fraction < 0 ) {
            hd->last_add_rem_effect_map.erase( found );
            to_remove.push_back( *i );
            ++shift_index;
          } else {
            if ( shift_index > 0 ) {
              hd->last_add_rem_effect_map[ j - shift_index ] =
                hd->last_add_rem_effect_map[ j ];
              hd->last_add_rem_effect_map.erase( j );
            }
            _output = _output + (*i)->calculateForces( input ) * temp_fraction;
          }
        } else {
          _output = _output + (*i)->calculateForces( input );
        }
      }
    } else {
      // calculate forces normally.
      for( HapticEffectVector::const_iterator i =
           hd->last_force_effects.begin();
         i != hd->last_force_effects.end();
         ++i ) {
        _output = _output + (*i)->calculateForces( input );
      }
    }

    // fraction used for interpolation
    HAPIFloat force_interpolation_fraction = 1;

    if( hd->switching_effects ) {
      force_interpolation_fraction =
        ( now_time - hd->last_force_effect_change ) /
        hd->switch_effects_duration;
      if( force_interpolation_fraction > 1 ) {
        force_interpolation_fraction = 1;
        hd->last_force_effects.clear();
        hd->last_add_rem_effect_map.clear();
        hd->switching_effects = false;
      }
      _output = _output * ( 1 - force_interpolation_fraction );
    }

    if( !hd->last_force_effects.empty() ) {
      // remove HAPIForceEffects from last_force_effects if they have been
      // phased out.
      for( std::list< HAPIForceEffect * >::iterator i = to_remove.begin();
           i != to_remove.end();
           ++i ) {
        hd->last_force_effects.erase( *i );
      }
    }

    // calculate the forces generated by the active force effects
    if( current_add_interpolate ) {
      // If there are force effects that should be phased in
      // a fraction will be calculated.
      int j = 0;
      for( HapticEffectVector::const_iterator i =
             hd->current_force_effects.begin();
           i != hd->current_force_effects.end();
           ++i, ++j ) {
        IndexTimeMap::iterator found =
          hd->current_add_rem_effect_map.find( j );
        if( found != hd->current_add_rem_effect_map.end() ) {
          HAPIFloat temp_fraction =
            hd->current_add_rem_effect_map[j].getFraction( now_time );
          if( temp_fraction > 1 ) {
            temp_fraction = 1;
            hd->current_add_rem_effect_map.erase( found );
          }
          _output = _output + (*i)->calculateForces( input ) *
                            temp_fraction *
                            force_interpolation_fraction;
        } else
          _output = _output + (*i)->calculateForces( input ) *
                            force_interpolation_fraction;
      }
    } else {
      // Only fraction calculations from setEffects and swapEffects.
      for( HapticEffectVector::const_iterator i =
           hd->current_force_effects.begin();
         i != hd->current_force_effects.end();
         ++i ) {
        _output = _output + (*i)->calculateForces( input )
          * force_interpolation_fraction;
      }
    }
  } else {
    // No interpolation what so ever, just calculate and sum the forces
    // from the haptic effects.
    for( HapticEffectVector::const_iterator i =
           hd->current_force_effects.begin();
         i != hd->current_force_effects.end();
         ++i ) {
      _output = _output + (*i)->calculateForces( input );
    }
  }

  hd->renderer_change_lock.lock();

  for( unsigned int s = 0; s < hd->haptics_renderers.size(); ++s ) {
    if( hd->haptics_renderers[s] ) {
      // Resize current_shapes, this is done since we want renderHapticsOneStep
      // to be called even when there are no current_shapes in the scene in
      // order for the renderer to properly update internal data such as
      // proxy position (if the renderer is proxy based).
      if( s + 1 > hd->current_shapes.size() )
        hd->current_shapes.resize( s + 1 );

      _output = _output + 
        hd->haptics_renderers[s]->
        renderHapticsOneStep( hd, hd->current_shapes[s], dt );
    }
  }

  hd->renderer_change_lock.unlock();

  // scale output force
  _output.force = _output.force * hd->ts_force_scale;

  // clamp to limits
  HAPIFloat max_force = hd->getForceLimit();
  HAPIFloat max_torque = hd->getTorqueLimit();

  if( max_force >= 0 ) {
    HAPIFloat length_sqr = _output.force.lengthSqr();
    if( length_sqr > max_force * max_force ) {
      _output.force = _output.force * (max_force / H3DUtil::H3DSqrt( length_sqr ) );
    }
  }

  if( max_torque >= 0 ) {
    HAPIFloat length_sqr = _output.torque.lengthSqr();
    if( length_sqr > max_torque * max_torque ) {
      _output.torque = _output.torque * (max_torque / H3DUtil::H3DSqrt( length_sqr ) );
    }
  }

  // add the resulting force and torque to the rendered force.
  hd->sendForce( _output.force );
  hd->sendTorque( _output.torque );
  hd->sendRawDof7Force( _output.dof7_force );

  hd->sendOutput( dt );

  // move shapes according to velocity, etc
  for( unsigned int r = 0; r < hd->haptics_renderers.size(); ++r ) {
    if( r < hd->current_shapes.size() ) {
      for( unsigned int s = 0; s < hd->current_shapes[r].size(); ++s ) {
        hd->current_shapes[r][s]->moveTimestep( dt );
      } 
    }
  }

  hd->time_in_last_loop = TimeStamp() - start_time;
#ifdef HAVE_PROFILER
  H3DUtil::H3DTimer::stepEnd("haptic_render");
  //string temp = H3DUtil::H3DTimer::end("haptic_profile");
  //sofa::helper::AdvancedTimer::end("haptic_profile");
  //std::string temp ;
  std::stringstream temp;
  
  H3DUtil::H3DTimer::end("haptic_profile",temp);
  hd->profiled_result_lock.lock();
  hd->profiled_result_haptic[1] ="timer_for_haptic_rendering: \n" + temp.str();
  hd->profiled_result_lock.unlock();
#endif
  return H3DUtil::PeriodicThread::CALLBACK_CONTINUE;
}

void HAPIHapticsDevice::transferObjects() {
  
  if( thread ) {

    // Wait for previous callback to complete before starting next
    transfer_objects_cond.lock ();
    
    while ( transfer_objects_active ) {
      transfer_objects_cond.wait();
    }

    if ( !haveObjectsChanged() ) {
      transfer_objects_cond.unlock ();
      return;
    }
    
    transfer_objects_active= true;
    transfer_objects_cond.unlock ();

    for( unsigned int s = 0; s < haptics_renderers.size(); ++s ) {
      H3DTIMER_BEGIN("TRANSFEROBJECT_preprocessShape");
      if( haptics_renderers[s] && s < tmp_shapes.size() ) {
         haptics_renderers[s]->preProcessShapes( this, tmp_shapes[s] );
      }
      H3DTIMER_END("TRANSFEROBJECT_preprocessShape");
    }

    H3DTIMER_BEGIN("transferObjectCallback");
    thread->asynchronousCallback( transferObjectsCallback, this);
    H3DTIMER_END("transferObjectCallback");
  }
}

HAPIHapticsDevice::ErrorCode HAPIHapticsDevice::initDevice(
                               int _thread_frequency ) {
  if( device_state == UNINITIALIZED ) {
    last_device_values = current_device_values = DeviceValues();
    last_raw_device_values = current_raw_device_values = DeviceValues();
    if( !initHapticsDevice( _thread_frequency ) ) {
      return FAIL;
    }
    device_state = INITIALIZED;
    for( unsigned int i = 0; i < haptics_renderers.size(); ++i ) {
      if( haptics_renderers[i] ) {
        haptics_renderers[i]->initRenderer( this );
      }
    }
    
    device_state = INITIALIZED;
    if( !thread ) {
      // create a new thread to run the haptics in
      thread = new H3DUtil::HapticThread( H3DUtil::HapticThread::HIGH_PRIORITY,
        _thread_frequency );
      thread->setThreadName( "HAPI Haptics Thread" );
      delete_thread = true;
    }
#ifdef HAVE_PROFILER
    if(thread)
    {
      // synchronous callback to enable timer
      thread->synchronousCallback(enableTimerCallback,this);
    }
#endif
    if( setup_haptic_rendering_callback ) {
      haptic_rendering_cb_handle =
        thread->asynchronousCallback( hapticRenderingCallback,
                                      this );
    }
    
  }
  return SUCCESS;
}

void HAPIHapticsDevice::updateDeviceValues( HAPITime dt ) {

  if( device_state == ENABLED ) {
    DeviceValues dv;
    updateDeviceValues( dv, dt );
    
    device_values_lock.lock();
    ts_force_scale = force_scale;
    last_device_values = current_device_values;
    last_raw_device_values = current_raw_device_values;
    current_raw_device_values = dv;

    current_device_values.position = position_calibration * dv.position;
    
    current_device_values.velocity = 
      position_calibration.getScaleRotationPart() * dv.velocity;
    current_device_values.orientation = 
      orientation_calibration * dv.orientation;
    current_device_values.force = 
      position_calibration.getScaleRotationPart() * dv.force;
    current_device_values.torque = 
      position_calibration.getScaleRotationPart() * dv.torque;
    current_device_values.button_status = dv.button_status;
    current_device_values.angular_velocity = 
      orientation_calibration * dv.angular_velocity;
    current_device_values.dof7_angle = dv.dof7_angle;
    current_device_values.dof7_force = dv.dof7_force;
    device_values_lock.unlock();
  } else {
    if( device_state == INITIALIZED ) {
      device_values_lock.lock();
      ts_force_scale = force_scale;
      last_device_values = current_device_values;
      last_raw_device_values = current_raw_device_values;
      device_values_lock.unlock();
    }
  }
}

bool HAPIHapticsDevice::haveObjectsChanged () {
  if ( always_transfer_objects ) {
    return true;
  }

  // Compare force effects
  if ( tmp_current_force_effects.size() == current_force_effects.size() ) {
    for ( size_t i= 0; i < tmp_current_force_effects.size(); ++i ) {
      if ( tmp_current_force_effects[i] != current_force_effects[i] ) {
        // Force effects have changed
        return true;
      }
    }

    // Compare shapes
    if ( tmp_shapes.size() == current_shapes.size() ) {
      for ( size_t i= 0; i < tmp_shapes.size(); ++i ) {
        if ( tmp_shapes[i].size() == current_shapes[i].size() ) {
          for ( size_t j= 0; j < tmp_shapes[i].size(); ++j ) {
            if ( tmp_shapes[i][j] != current_shapes[i][j] ) {
              // Shapes have changed
              return true;
            }
          }
        }
      }
    }

    // Every thing is the same
    return false;
  } else {
    // Force effects have changed
    return true;
  }
}