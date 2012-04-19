IF(WIN32)
  cmake_minimum_required(VERSION 2.6.0)
ENDIF(WIN32)
# This will set up three projects. These three projects are examples
# from the manual.
project (ThreadExamples)

SET( SimpleThreadPrint_SRCS "${ThreadExamples_SOURCE_DIR}/../SimpleThreadPrint/SimpleThreadPrint.cpp" )
SET( SimpleThreadPrintLock_SRCS "${ThreadExamples_SOURCE_DIR}/../SimpleThreadPrintLock/SimpleThreadPrintLock.cpp" )
SET( PeriodicThreadCallbacks_SRCS "${ThreadExamples_SOURCE_DIR}/../PeriodicThreadCallbacks/PeriodicThreadCallbacks.cpp" )

SET(requiredLibs)

SET(CMAKE_MODULE_PATH ${ThreadExamples_SOURCE_DIR}/../../../build/modules )

IF( H3D_USE_DEPENDECIES_ONLY )
  # The variables set here must be set by the CMakeLists.txt that sets H3D_USE_DEPENDECIES_ONLY to true.
  IF(PTHREAD_FOUND)
    INCLUDE_DIRECTORIES( ${PTHREAD_INCLUDE_DIR} ) 
  ENDIF(PTHREAD_FOUND)
  INCLUDE_DIRECTORIES( ${EXTERNAL_INCLUDE_DIR} ) 
  INCLUDE_DIRECTORIES( ${H3DUTIL_INCLUDE_DIR} ) 
  
  SET( requiredLibs ${requiredLibs} H3DUtil )
ELSE( H3D_USE_DEPENDECIES_ONLY )
  #H3DUtil
  FIND_PACKAGE(H3DUtil REQUIRED)

  IF(H3DUTIL_FOUND)
    INCLUDE_DIRECTORIES( ${H3DUTIL_INCLUDE_DIR} ) 
    SET(requiredLibs ${requiredLibs} ${H3DUTIL_LIBRARIES} )
  ENDIF(H3DUTIL_FOUND)

  FIND_PACKAGE(PTHREAD REQUIRED)
  IF(PTHREAD_FOUND)
    INCLUDE_DIRECTORIES( ${PTHREAD_INCLUDE_DIR} ) 
    SET(requiredLibs ${requiredLibs} ${PTHREAD_LIBRARIES} )
  ENDIF(PTHREAD_FOUND)
ENDIF( H3D_USE_DEPENDECIES_ONLY )

ADD_EXECUTABLE(SimpleThreadPrint ${SimpleThreadPrint_SRCS})
ADD_EXECUTABLE(SimpleThreadPrintLock ${SimpleThreadPrintLock_SRCS})
ADD_EXECUTABLE(PeriodicThreadCallbacks ${PeriodicThreadCallbacks_SRCS})

# make the name of debug libraries end in _d.
SET_TARGET_PROPERTIES( SimpleThreadPrint PROPERTIES DEBUG_POSTFIX "_d" )
SET_TARGET_PROPERTIES( SimpleThreadPrintLock PROPERTIES DEBUG_POSTFIX "_d" )
SET_TARGET_PROPERTIES( PeriodicThreadCallbacks PROPERTIES DEBUG_POSTFIX "_d" )

TARGET_LINK_LIBRARIES( SimpleThreadPrint ${requiredLibs})
TARGET_LINK_LIBRARIES( SimpleThreadPrintLock ${requiredLibs})
TARGET_LINK_LIBRARIES( PeriodicThreadCallbacks ${requiredLibs})

# set the install directory to the H3D directory on Windows
IF(WIN32) 
  IF( CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT AND NOT H3D_USE_DEPENDENCIES_ONLY)
    SET( CMAKE_INSTALL_PREFIX ${ThreadExamples_SOURCE_DIR}/../.. CACHE PATH "Install path prefix, prepended onto install directories." FORCE )
  ENDIF( CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT AND NOT H3D_USE_DEPENDENCIES_ONLY)
ENDIF(WIN32)

INSTALL( TARGETS SimpleThreadPrint
	 LIBRARY DESTINATION lib
	 RUNTIME DESTINATION bin )

INSTALL( TARGETS SimpleThreadPrintLock
	 LIBRARY DESTINATION lib
	 RUNTIME DESTINATION bin )

INSTALL( TARGETS PeriodicThreadCallbacks
	 LIBRARY DESTINATION lib
	 RUNTIME DESTINATION bin )

