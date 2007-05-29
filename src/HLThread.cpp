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
/// \file Threads.cpp
/// \brief cpp file for thread handling functions.
///
//
//////////////////////////////////////////////////////////////////////////////
#include <HLThread.h>

#ifdef HAVE_OPENHAPTICS
#include <HD/hd.h>
#endif

#include <algorithm>

using namespace HAPI;

HLThread *HLThread::singleton = new HLThread;

#ifdef HAVE_OPENHAPTICS
HDCallbackCode HDCALLBACK hdCallback( void *_data ) {
  void * * data = static_cast< void * * >( _data );
  HLThread::CallbackFunc* func = static_cast< HLThread::CallbackFunc* >( _data );
  //HLThread::CallbackFunc func = static_cast< HLThread::CallbackFunc >( data[0] );
  HLThread::CallbackCode c = (*func)( data[1] );
  if( c == HLThread::CALLBACK_DONE ) return HD_CALLBACK_DONE;
  else if( c == HLThread::CALLBACK_CONTINUE ) return HD_CALLBACK_CONTINUE;
  else return c;
}
#endif

void HLThread::synchronousCallback( CallbackFunc func, void *data ) {
#ifdef HAVE_OPENHAPTICS
  void * param[] = { (void*)func, data };
  hdScheduleSynchronous( hdCallback,
                         param,
                         HD_DEFAULT_SCHEDULER_PRIORITY );
#endif
}

void HLThread::asynchronousCallback( CallbackFunc func, void *data ) {
#ifdef HAVE_OPENHAPTICS
  void * * param = new void * [2];
  param[0] = (void*)func;
  param[1] = data;
  hdScheduleAsynchronous( hdCallback,
                          param,
                          HD_DEFAULT_SCHEDULER_PRIORITY );
#endif
}

H3DUtil::PeriodicThread::CallbackCode HLThread::setThreadId( void * data ) {
  HLThread *thread = static_cast< HLThread * >( data );
  thread->thread_id = H3DUtil::PeriodicThread::getCurrentThreadId();
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
