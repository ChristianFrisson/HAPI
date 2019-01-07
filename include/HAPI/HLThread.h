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
/// \file HLThread.h
/// \brief Header file for HLThread.
///
//
//////////////////////////////////////////////////////////////////////////////

#ifndef __HLTHREAD_H__
#define __HLTHREAD_H__

#include <HAPI/HAPI.h>
#include <H3DUtil/Threads.h>

#include <memory>

namespace HAPI {
  /// \ingroup Others
  /// \class HLThread
  /// \brief A singleton class providing an interface to the
  /// scheduler and thread running when using OpenHaptics and HD API.
  ///
  /// It is used by the HLHapticsDevice and uses its own thread handling.
  /// Since only one instance of the HD API scheduler exists it is a
  /// singleton class.
  class HAPI_API HLThread : public H3DUtil::HapticThreadBase,
                            public H3DUtil::PeriodicThreadBase {

  private:
    HLThread():
      is_active( false ) {
      // the hl thread should not be added unless the hd scheduler
      // has started.
      sg_lock.lock();
      threads.pop_back();
      sg_lock.unlock();
    }
  public:
    /// Get the singleton instance of HLThread.
    static HLThread *getInstance();

    /// If the hd scheduler has been started true is returned.
    /// Note that the returned value can possible have changed if isActive
    /// is not called in the same thread as setActive.
    inline bool isActive() { return is_active; }

    /// Set the flag indicating if the hd scheduler has been
    /// started or not.
    void setActive( bool _active );

    /// Add a callback function to be executed in this thread. The calling
    /// thread will wait until the callback function has returned before 
    /// continuing. 
    virtual void synchronousCallback( CallbackFunc func, void *data );

    /// Add a callback function to be executed in this thread. The calling
    /// thread will continue executing after adding the callback and will 
    /// not wait for the callback function to execute.
    /// \returns A handle to the callback that can be used to remove
    /// the callback.
    virtual int asynchronousCallback( CallbackFunc func, void *data );

    /// Attempts to remove a callback. returns true if succeded. returns
    /// false if the callback does not exist. This function should be handled
    /// with care. It can remove the wrong callback if the callback that
    /// returned the callback_handle id is removed and a new callback is added.
    /// Callbacks are removed if they return CALLBACK_DONE or a call to this
    /// function is made.
    virtual bool removeAsynchronousCallback( int callback_handle );

    /// Function used for callbacks to add ids to free_ids vector. It is
    /// assumed to be called between lock/unlock pairs of
    /// HLThreadInternals::callback_handles_lock, which means that it is for
    /// internal use in HLThread.cpp only.
    inline void addFreeId( int _id ) {
      free_ids.push_back( _id );
    }

  protected:
    static H3DUtil::PeriodicThread::CallbackCode setThreadId( void * _data );
    static std::auto_ptr< HLThread > singleton;
    bool is_active;

    typedef std::list< std::pair< int, std::pair< CallbackFunc, void * > > >
      CallbackList;
    // A list of the callback functions to run.
    CallbackList callbacks;
    
    // A lock for synchronizing changes to the callbacks member.
    H3DUtil::ConditionLock callback_lock;

    // A list of the callback functions to add to the callbacks variable
    // when the callback_lock is released. 
    CallbackList callbacks_added;

    // A lock used when modifying callbacks_added variable.
    H3DUtil::MutexLock callbacks_added_lock;

  public:
    /// The main function that calls all callbacks and transfers asynchronous callbacks.
    /// Public in order to still be able to include HD/hd.h in HlThread.cpp to not get
    /// dependent symbols in classes that include HLThread.h.
    /// ANY OTHER USE OF THIS FUNCTION RESULTS IN UNDEFINED BEHAVIOUR.
    void threadFunction();
  };

}

#endif



