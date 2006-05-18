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
/// \file Threads.h
/// \brief Header file for thread handling functions.
///
//
//////////////////////////////////////////////////////////////////////////////

#ifndef __THREADS_H__
#define __THREADS_H__

#include <HAPI.h>
#include <list>
#include <vector>
#include <pthread.h>

#ifdef WIN32
#ifdef _MSC_VER
#pragma comment( lib, "pthreadVC2.lib" )
#endif
#define DEFAULT_THREAD_PRIORITY THREAD_PRIORITY_NORMAL
#else
#define DEFAULT_THREAD_PRIORITY 0
#endif

namespace H3D {
  /// Mutual exclusion lock class for synchronisation between threads. 
  /// Most common usage is to make sure that only one thread enters a 
  /// "critical section" at a time. E.g. if both threads uses the same 
  /// variable we must put a lock around the access to make sure that
  /// two threads does not access it at once.
  class HAPI_API MutexLock {
  public:
    /// Constructor.
    MutexLock();
    
    /// Destructor.
    ~MutexLock();

    /// Locks the mutex. If already locked, waits until it is unlocked and
    /// then locks it.
    void lock();

    /// Unlocks the mutex.
    void unlock();

    /// Try to lock the mutex, if the lock is not available false is returned.
    bool tryLock();
        
  protected:
    pthread_mutex_t mutex;
  };


  /// The ConditionLock is a little more advanced version of MutexLock in that
  /// it can wait for an arbitrary action in the other thread.
  /// 
  /// Thread 1        
  /// l.lock()
  /// l.wait()  - lock will be released and the thread will wait for a signal
  ///             from another thread before continuing. When it continues the
  ///             lock is aquired again.
  /// l.unlock()
  ///
  /// Thread 2
  /// l.lock()
  /// do some stuff
  /// l.signal() - wake the waiting thread  
  /// l.unlock()
  class HAPI_API ConditionLock: public MutexLock {
  public:
    /// Constructor.
    ConditionLock();
    
    /// Destructor.
    ~ConditionLock();

    /// Wait for the conditional to get a signal. The lock will be released
    /// while waiting and reaquired when a signal is received.
    void wait();

    /// Wait for the conditional to get a signal, but only wait a
    /// certain time. If the time exceeds the specified time false
    /// is returned. If signal is received true is returned.
    bool timedWait( unsigned int ms );

    /// Wakes up at least one thread blocked on this condition lock.
    void signal();

    /// This wakes up all of the threads blocked on the condition lock.
    void broadcast();
        
  protected:
    pthread_cond_t cond; 
  };

 
  /// The abstract base class for threads.
  class HAPI_API ThreadBase {
  public:
    /// Destructor.
    virtual ~ThreadBase() {}

    typedef pthread_t ThreadId;

    /// Returns the id of the thread this function is called in.
    static ThreadId getCurrentThreadId();

    /// Returns the id of the main thread.
    static ThreadId getMainThreadId() {
      return main_thread_id;
    }
    
    /// Returns true if the call was made from the main thread.
    static bool inMainThread();

    /// Returns the thread id for this thread.
    inline ThreadId getThreadId() { return thread_id; }
  protected:
    /// the id of the thread.
    ThreadId thread_id;
    
    /// The id of the main thread.
    static ThreadId main_thread_id;
  };

  /// The abstract base class for threads that have a main loop and that allow
  /// you to add callback functions to be run in that loop.
  class HAPI_API PeriodicThreadBase: public ThreadBase {
  public:
    /// Return code for callback functions. 
    typedef enum {
      /// The callback is done and should not be called any more.
      CALLBACK_DONE,
      /// The callback should be rescheduled and called the next loop 
      /// again.
      CALLBACK_CONTINUE
    } CallbackCode;

    /// Callback function type.
    typedef CallbackCode (*CallbackFunc)(void *data); 

    /// Add a callback function to be executed in this thread. The calling
    /// thread will wait until the callback function has returned before 
    /// continuing. 
    virtual void synchronousCallback( CallbackFunc func, void *data ) = 0;

    /// Add a callback function to be executed in this thread. The calling
    /// thread will continue executing after adding the callback and will 
    /// not wait for the callback function to execute. 
    virtual void asynchronousCallback( CallbackFunc func, void *data ) = 0;
  };

  /// The interface base class for all threads that are used for haptics
  /// devices.
  class HAPI_API HapticThreadBase {
  public:
    /// Constructor.
    HapticThreadBase();
    
    /// Destructor.
    virtual ~HapticThreadBase();

    /// Add a callback function that is to be executed when all the haptic
    /// threads have been synchronised, so it will be a thread safe 
    /// callback between all haptic threads.
    static void synchronousHapticCB( PeriodicThreadBase::CallbackFunc func, 
                                     void *data );

    /// Returns true if the call was made from within a HapticThreadBase
    /// thread.
    static bool inHapticThread();
  protected:
    // Callback function for synchronising all haptic threads.
    static PeriodicThreadBase::CallbackCode sync_haptics( void * );

    // The haptic threads that have been created.
    static std::vector< HapticThreadBase * > threads;

    // Lock used to get the haptic threads to wait for the callback 
    // function to finish.
    static ConditionLock haptic_lock; 

    // Lock used to get the thread calling the synchronousHapticCB
    // function to wait for all the haptic threads to synchronize.
    static ConditionLock sg_lock; 

    // Counter used when synchronizing haptic threads. It tells how
    // many more haptic threads that are left to synchronize.
    static int haptic_threads_left;
  };

  /// The SimpleThread class creates a new thread to run a function. The
  /// thread is run until the function returns.
  class HAPI_API SimpleThread : public ThreadBase {
  public:
    /// Constructor.
    /// \param thread_priority The priority of the thread.
    /// \param func The function to run in the thread. 
    SimpleThread( void *(func) (void *),
                  void *args = NULL,
                  int thread_priority = DEFAULT_THREAD_PRIORITY );
    
    /// Destructor.
    virtual ~SimpleThread();
  }; 

  /// The PeriodicThread class is used to create new threads and provides an interface
  /// to add callback functions to be executed in the new thread that can be 
  /// used by other threads.
  class HAPI_API PeriodicThread : public PeriodicThreadBase {
  public:
    /// Constructor.
    /// \param thread_priority The priority of the thread.
    /// \param thread_frequency The frequence of the thread loop. -1 means
    /// run as fast as possible.
    PeriodicThread( int thread_priority = DEFAULT_THREAD_PRIORITY,
            int thread_frequency = -1 );
    
    /// Destructor.
    virtual ~PeriodicThread();
    
    /// Add a callback function to be executed in this thread. The calling
    /// thread will wait until the callback function has returned before 
    /// continuing. 
    virtual void synchronousCallback( CallbackFunc func, void *data );

    /// Add a callback function to be executed in this thread. The calling
    /// thread will continue executing after adding the callback and will 
    /// not wait for the callback function to execute. 
    virtual void asynchronousCallback( CallbackFunc func, void *data );
  protected:
    // The function that handles callbacks. Is also the main function that
    // is run in the thread.
    static void *thread_func( void * );

    typedef std::list< std::pair< CallbackFunc, void * > > CallbackList;
    // A list of the callback functions to run.
    CallbackList callbacks;
    // A lock for synchronizing changes to the callbacks member.
    ConditionLock callback_lock;
    
    /// The priority of the thread.
    int priority;

    /// Thre frequency of the thread. -1 means run as fast as possible.
    int frequency;

  };

  /// HapticThread is a thread class that should be used by haptics devices
  /// when creating threads. It is the same as PeriodicThread, but also inherits
  /// from HapticThreadBase to make it aware that it is a haptic thread/
  class HAPI_API HapticThread : public HapticThreadBase,
                                public PeriodicThread {
  public:
    /// Constructor.
    HapticThread( int thread_priority =  DEFAULT_THREAD_PRIORITY,
		int thread_frequency = -1 ):
      PeriodicThread( thread_priority, thread_frequency ) {
    }
  };

  /// HLThread is a singleton class providing an interface to the scheduler
  /// and thread running when using OpenHaptics and HD API. It is used
  /// by the HLHapticsDevice and uses its own thread handling. Since only
  /// one instance of the HD API scheduler exists it is a singleton class.
  class HAPI_API HLThread : public HapticThreadBase,
                            public PeriodicThreadBase {

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
    static HLThread *getInstance() {
      return singleton;
    }

    /// If the hd scheduler has been started true is returned.
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
    virtual void asynchronousCallback( CallbackFunc func, void *data );
  protected:
    static PeriodicThread::CallbackCode setThreadId( void * _data );
    static HLThread *singleton;
    bool is_active;
  };

}

#endif



