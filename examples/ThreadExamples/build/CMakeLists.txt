# This will set up three projects. These three projects are examples
# from the manual.
project (ThreadExamples)

AUX_SOURCE_DIRECTORY(../SimpleThreadPrint SimpleThreadPrint_SRCS)
AUX_SOURCE_DIRECTORY(../SimpleThreadPrintLock SimpleThreadPrintLock_SRCS)
AUX_SOURCE_DIRECTORY(../PeriodicThreadCallbacks PeriodicThreadCallbacks_SRCS)

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

TARGET_LINK_LIBRARIES( SimpleThreadPrint ${requiredLibs})
TARGET_LINK_LIBRARIES( SimpleThreadPrintLock ${requiredLibs})
TARGET_LINK_LIBRARIES( PeriodicThreadCallbacks ${requiredLibs})
