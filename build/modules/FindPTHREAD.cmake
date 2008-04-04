# - Find pthread
# Find the native PTHREAD headers and libraries.
#
#  PTHREAD_INCLUDE_DIR -  where to find pthread.h, etc.
#  PTHREAD_LIBRARIES    - List of libraries when using pthread.
#  PTHREAD_FOUND        - True if pthread found.


# Look for the header file.
FIND_PATH(PTHREAD_INCLUDE_DIR NAMES pthread.h
                              PATHS $ENV{H3D_EXTERNAL_ROOT}/include  
                                    $ENV{H3D_EXTERNAL_ROOT}/include/pthread
                                    ../../External/include    
                                    ../../External/include/pthread)

MARK_AS_ADVANCED(PTHREAD_INCLUDE_DIR)

# Look for the library.

IF(WIN32)
  FIND_LIBRARY(PTHREAD_LIBRARY NAMES pthreadVC2 
                               PATHS $ENV{H3D_EXTERNAL_ROOT}/lib
                                     ../../External/lib    )
ELSE(WIN32)
  FIND_LIBRARY(PTHREAD_LIBRARY NAMES pthread)
ENDIF(WIN32)
MARK_AS_ADVANCED(PTHREAD_LIBRARY)

# Copy the results to the output variables.
IF(PTHREAD_INCLUDE_DIR AND PTHREAD_LIBRARY)
  SET(PTHREAD_FOUND 1)
  SET(PTHREAD_LIBRARIES ${PTHREAD_LIBRARY})
  SET(PTHREAD_INCLUDE_DIR ${PTHREAD_INCLUDE_DIR})
ELSE(PTHREAD_INCLUDE_DIR AND PTHREAD_LIBRARY)
  SET(PTHREAD_FOUND 0)
  SET(PTHREAD_LIBRARIES)
  SET(PTHREAD_INCLUDE_DIR)
ENDIF(PTHREAD_INCLUDE_DIR AND PTHREAD_LIBRARY)

# Report the results.
IF(NOT PTHREAD_FOUND)
  SET(PTHREAD_DIR_MESSAGE
    "PTHREAD was not found. Make sure PTHREAD_LIBRARY and PTHREAD_INCLUDE_DIR are set. Pthread is required to compile.")
  IF(NOT PTHREAD_FIND_QUIETLY)
    MESSAGE(STATUS "${PTHREAD_DIR_MESSAGE}")
  ELSE(NOT PTHREAD_FIND_QUIETLY)
    IF(PTHREAD_FIND_REQUIRED)
      MESSAGE(FATAL_ERROR "${PTHREAD_DIR_MESSAGE}")
    ENDIF(PTHREAD_FIND_REQUIRED)
  ENDIF(NOT PTHREAD_FIND_QUIETLY)
ENDIF(NOT PTHREAD_FOUND)
