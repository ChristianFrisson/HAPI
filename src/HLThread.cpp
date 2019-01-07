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
/// \file HLThread.cpp
/// \brief cpp file for HLThread.
///
//
//////////////////////////////////////////////////////////////////////////////
#include <HAPI/HLThread.h>

#ifdef HAVE_OPENHAPTICS
#include <HD/hd.h>
#endif

#include <algorithm>
#include <string>

using namespace HAPI;

std::auto_ptr< HLThread > HLThread::singleton( NULL );

/// Get the singleton instance of HLThread.
HLThread *HLThread::getInstance() {
  if( !singleton.get() ) singleton.reset( new HLThread );
  return singleton.get();
}

void HLThread::synchronousCallback( CallbackFunc func, void *data ) {
  callback_lock.lock();
  if( is_active ) {
    callbacks_added_lock.lock(); // Have to get added_lock in order to call genCallbackId in a thread safe way
    // add the new callback, the reason for not calling here is simply that we want
    // the callback to be executed in the correct thread.
    callbacks.push_back( std::make_pair( genCallbackId(), std::make_pair( func, data ) ) );
    // wait for the callback to be done.
    callbacks_added_lock.unlock();
    callback_lock.wait();
  } else {
    func(data);
  }
  callback_lock.unlock();
}

int HLThread::asynchronousCallback( CallbackFunc func, void *data ) {
  callbacks_added_lock.lock();
  int cb_id = genCallbackId();
  // add the new callback
  callbacks_added.push_back( std::make_pair( cb_id, std::make_pair( func, data ) ) );
  callbacks_added_lock.unlock();
  return cb_id;
}

bool HLThread::removeAsynchronousCallback( int callback_handle ) {
  callback_lock.lock();
  callbacks_added_lock.lock();
  // For threads with a low frequency it could be that the callback is
  // in callbacks_added, therefore lock both locks and then go through
  // both lists.
  for( CallbackList::iterator i = callbacks_added.begin();
       i != callbacks_added.end(); ++i ) {
  if( (*i).first == callback_handle ) {
      // Add callback_handle integer to the free_ids list in order
      // to reuse id later.
      free_ids.push_back( callback_handle );
      callbacks_added.erase( i );
      callbacks_added_lock.unlock();
      callback_lock.unlock();
      return true;
    }
  }

  for( CallbackList::iterator i = callbacks.begin();
       i != callbacks.end(); ++i ) {
    if( (*i).first == callback_handle ) {
      // Add callback_handle integer to the free_ids list in order
      // to reuse id later.
      free_ids.push_back( callback_handle );
      callbacks.erase( i );
      callbacks_added_lock.unlock();
      callback_lock.unlock();
      return true;
    }
  }
  callbacks_added_lock.unlock();
  callback_lock.unlock();
  return false;
}

H3DUtil::PeriodicThread::CallbackCode HLThread::setThreadId( void * data ) {
  HLThread *thread = static_cast< HLThread * >( data );
  thread->thread_id = H3DUtil::PeriodicThread::getCurrentThreadId();
  thread->setThreadName( std::string("OpenHaptics HD Scheduler Thread") );
  return H3DUtil::PeriodicThread::CALLBACK_DONE;
}

#ifdef HAVE_OPENHAPTICS
HDCallbackCode HDCALLBACK mainCallback( void *data ) {
  HLThread * thread = static_cast< HLThread * >(data);
  thread->threadFunction();
  return HD_CALLBACK_CONTINUE;
}
#endif

void HLThread::setActive( bool _active ) {
  callback_lock.lock();
  if( _active && !is_active ) {
    is_active = _active;
    callback_lock.unlock();
    sg_lock.lock(); 
    threads.push_back( this );
    sg_lock.unlock();
#ifdef HAVE_OPENHAPTICS
    HDSchedulerHandle hd_callback_handle = 
      hdScheduleAsynchronous( mainCallback,
                              this,
                              HD_DEFAULT_SCHEDULER_PRIORITY );
#endif
    asynchronousCallback( setThreadId, this );
  } else if( !_active && is_active ) {
    is_active = _active;
    callback_lock.unlock();
    sg_lock.lock();
    std::vector< H3DUtil::HapticThreadBase *>::iterator i = 
      std::find( threads.begin(), 
                 threads.end(), 
                 this );
    if( i != threads.end() ) {
      threads.erase( i );
    }
    sg_lock.unlock();
  } else
    callback_lock.unlock();
}

void HLThread::threadFunction() {
  callbacks_added_lock.lock();
  callbacks.insert( callbacks.end(), callbacks_added.begin(), callbacks_added.end() );
  callbacks_added.clear();
  callbacks_added_lock.unlock();

  std::vector< CallbackList::iterator > to_remove;
  callback_lock.lock();
  for( CallbackList::iterator i = callbacks.begin();
       i != callbacks.end(); ++i ) {
      CallbackCode c = ( (*i).second ).first(
        ( (*i).second ).second );
      if( c == CALLBACK_DONE ) {
        to_remove.push_back( i );
      }
  }

  callbacks_added_lock.lock();
  // remove all callbacks that returned CALLBACK_DONE.
  for( std::vector< CallbackList::iterator >::iterator i = 
       to_remove.begin();
       i != to_remove.end(); ++i ) {
      free_ids.push_back( (*(*i) ).first );
      callbacks.erase( *i );
  }
  callbacks_added_lock.unlock();
  callback_lock.signal();
  callback_lock.unlock();
}
