cmake_minimum_required(VERSION 2.6.0)
# This will set up three projects. These three projects are examples
# from the manual.
project (ThreadExamples)

AUX_SOURCE_DIRECTORY(${ThreadExamples_SOURCE_DIR}/../SimpleThreadPrint SimpleThreadPrint_SRCS)
AUX_SOURCE_DIRECTORY(${ThreadExamples_SOURCE_DIR}/../SimpleThreadPrintLock SimpleThreadPrintLock_SRCS)
AUX_SOURCE_DIRECTORY(${ThreadExamples_SOURCE_DIR}/../PeriodicThreadCallbacks PeriodicThreadCallbacks_SRCS)

SET(requiredLibs)

SET(CMAKE_MODULE_PATH ${ThreadExamples_SOURCE_DIR}/../../../build/modules )

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
 SET( CMAKE_INSTALL_PREFIX ${ThreadExamples_SOURCE_DIR}/../..)
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

