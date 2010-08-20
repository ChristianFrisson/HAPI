# - Find pthread
# Find the native PTHREAD headers and libraries.
#
#  PTHREAD_INCLUDE_DIR -  where to find pthread.h, etc.
#  PTHREAD_LIBRARIES    - List of libraries when using pthread.
#  PTHREAD_FOUND        - True if pthread found.

GET_FILENAME_COMPONENT(module_file_path ${CMAKE_CURRENT_LIST_FILE} PATH )

# Look for the header file.
FIND_PATH(PTHREAD_INCLUDE_DIR NAMES pthread.h
                              PATHS $ENV{H3D_EXTERNAL_ROOT}/include  
                                    $ENV{H3D_EXTERNAL_ROOT}/include/pthread
                                    $ENV{H3D_ROOT}/../External/include  
                                    $ENV{H3D_ROOT}/../External/include/pthread
                                    ../../External/include
                                    ../../External/include/pthread
                                    ${module_file_path}/../../../External/include
                                    ${module_file_path}/../../../External/include/pthread
                              DOC "Path in which the file pthread.h is located." )

MARK_AS_ADVANCED(PTHREAD_INCLUDE_DIR)

# Look for the library.

IF( CMAKE_CL_64 )
  SET( LIB "lib64" )
ELSE( CMAKE_CL_64 )
  SET( LIB "lib32" )
ENDIF( CMAKE_CL_64 )

IF(WIN32)
  FIND_LIBRARY(PTHREAD_LIBRARY NAMES pthreadVC2 
                               PATHS $ENV{H3D_EXTERNAL_ROOT}/${LIB}
                                     $ENV{H3D_ROOT}/../External/${LIB}
                                     ../../External/${LIB}
                                     ${module_file_path}/../../../External/${LIB}
                               DOC "Path to pthreadVC2 library." )
ELSE(WIN32)
  FIND_LIBRARY( PTHREAD_LIBRARY NAMES pthread
                DOC "Path to pthread library." )
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
    "PTHREAD was not found. Make sure PTHREAD_LIBRARY and PTHREAD_INCLUDE_DIR are set.")
  IF(PTHREAD_FIND_REQUIRED)
    SET( PTHREAD_DIR_MESSAGE
         "${PTHREAD_DIR_MESSAGE} Pthread is required to build.")
    MESSAGE(FATAL_ERROR "${PTHREAD_DIR_MESSAGE}")
  ELSEIF(NOT PTHREAD_FIND_QUIETLY)
    SET( PTHREAD_DIR_MESSAGE
         "${PTHREAD_DIR_MESSAGE} Threading support will be disabled without PTHREAD.")
    MESSAGE(STATUS "${PTHREAD_DIR_MESSAGE}")
  ENDIF(PTHREAD_FIND_REQUIRED)
ENDIF(NOT PTHREAD_FOUND)
