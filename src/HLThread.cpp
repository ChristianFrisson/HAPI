//////////////////////////////////////////////////////////////////////////////
//    Copyright 2004-2007, SenseGraphics AB
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

namespace HLThreadInternals {
  H3DUtil::MutexLock callback_handles_lock;
#ifdef HAVE_OPENHAPTICS
  typedef std::list< std::pair< int, std::pair< void *, HDSchedulerHandle > > >
    CallbackHandleList;
  CallbackHandleList callback_handles;
#endif
}


/// Get the singleton instance of HLThread.
HLThread *HLThread::getInstance() {
  if( !singleton.get() ) singleton.reset( new HLThread );
  return singleton.get();
}

#ifdef HAVE_OPENHAPTICS
HDCallbackCode HDCALLBACK hdCallbackSynchronous( void *_data ) {
  void * * data = static_cast< void * * >( _data );
  HLThread::CallbackFunc* func = static_cast< HLThread::CallbackFunc* >( _data );
  //HLThread::CallbackFunc func = static_cast< HLThread::CallbackFunc >( data[0] );
  HLThread::CallbackCode c = (*func)( data[1] );
  if( c == HLThread::CALLBACK_DONE ) return HD_CALLBACK_DONE;
  else if( c == HLThread::CALLBACK_CONTINUE ) return HD_CALLBACK_CONTINUE;
  else return c;
}

HDCallbackCode HDCALLBACK hdCallbackAsynchronous( void *_data ) {
  void * * data = static_cast< void * * >( _data );
  HLThread::CallbackFunc* func = static_cast< HLThread::CallbackFunc* >( _data );
  //HLThread::CallbackFunc func = static_cast< HLThread::CallbackFunc >( data[0] );
  HLThread::CallbackCode c = (*func)( data[1] );
  if( c == HLThread::CALLBACK_DONE ) {
    HLThreadInternals::callback_handles_lock.lock();
    for( HLThreadInternals::CallbackHandleList::iterator i =
         HLThreadInternals::callback_handles.begin();
         i != HLThreadInternals::callback_handles.end(); i++ ) {
      if( (*i).second.first == _data ) {
        HLThread * hl_thread = static_cast< HLThread * >(data[2]);
        hl_thread->addFreeId( (*i).first );
        HLThreadInternals::callback_handles.erase( i );
        break;
      }
    }
    HLThreadInternals::callback_handles_lock.unlock();
    delete [] data;
    return HD_CALLBACK_DONE;
  }
  else if( c == HLThread::CALLBACK_CONTINUE ) return HD_CALLBACK_CONTINUE;
  else return c;
}
#endif

void HLThread::synchronousCallback( CallbackFunc func, void *data ) {
#ifdef HAVE_OPENHAPTICS
  void * param[] = { (void*)func, data };
  hdScheduleSynchronous( hdCallbackSynchronous,
                         param,
                         HD_DEFAULT_SCHEDULER_PRIORITY );
#endif
}

int HLThread::asynchronousCallback( CallbackFunc func, void *data ) {
  int cb_handle = -1;
#ifdef HAVE_OPENHAPTICS
  void * * param = new void * [3];
  param[0] = (void*)func;
  param[1] = data;
  param[2] = (void*)this;
  HDSchedulerHandle hd_callback_handle = 
    hdScheduleAsynchronous( hdCallbackAsynchronous,
                            param,
                            HD_DEFAULT_SCHEDULER_PRIORITY );
  HLThreadInternals::callback_handles_lock.lock();
  cb_handle = genCallbackId();
  HLThreadInternals::callback_handles.push_back(
    std::make_pair( cb_handle, std::make_pair( param, hd_callback_handle ) ) );
  HLThreadInternals::callback_handles_lock.unlock();
#endif
  return cb_handle;
}

bool HLThread::removeAsynchronousCallback( int callback_handle )  {
#ifdef HAVE_OPENHAPTICS
  HLThreadInternals::callback_handles_lock.lock();
  for( HLThreadInternals::CallbackHandleList::iterator i =
       HLThreadInternals::callback_handles.begin();
       i != HLThreadInternals::callback_handles.end(); i++ ) {
    if( (*i).first == callback_handle ) {
      free_ids.push_back( callback_handle );
      hdUnschedule( (*i).second.second );
      void * data = (*i).second.first;
      HLThreadInternals::callback_handles.erase( i );
      // Delete param that was allocated in asyncronous callback.
      delete [] data;
      HLThreadInternals::callback_handles_lock.unlock();
      return true;
    }
  }
  HLThreadInternals::callback_handles_lock.unlock();
#endif
  return false;
}

H3DUtil::PeriodicThread::CallbackCode HLThread::setThreadId( void * data ) {
  HLThread *thread = static_cast< HLThread * >( data );
  thread->thread_id = H3DUtil::PeriodicThread::getCurrentThreadId();
  thread->setThreadName( std::string("OpenHaptics HD Scheduler Thread") );
  return H3DUtil::PeriodicThread::CALLBACK_DONE;
}

void HLThread::setActive( bool _active ) { 
  if( _active && !is_active ) {
    sg_lock.lock(); 
    threads.push_back( this );
    sg_lock.unlock();
    asynchronousCallback( setThreadId, this );
  } else if( !_active && is_active ) {
    sg_lock.lock();
    std::vector< H3DUtil::HapticThreadBase *>::iterator i = 
      std::find( threads.begin(), 
                 threads.end(), 
                 this );
    if( i != threads.end() ) {
      threads.erase( i );
    }
    sg_lock.unlock();
  }
  is_active = _active; 
}
