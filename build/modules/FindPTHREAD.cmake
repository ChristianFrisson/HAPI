# - Find pthread
# Find the native PTHREAD headers and libraries.
#
#  PTHREAD_INCLUDE_DIR -  where to find pthread.h, etc.
#  PTHREAD_LIBRARIES    - List of libraries when using pthread.
#  PTHREAD_FOUND        - True if pthread found.

include( H3DExternalSearchPath )
GET_FILENAME_COMPONENT( module_file_path ${CMAKE_CURRENT_LIST_FILE} PATH )
get_external_search_paths_h3d( module_include_search_paths module_lib_search_paths ${module_file_path} "pthread" )

# Look for the header file.
FIND_PATH(PTHREAD_INCLUDE_DIR NAMES pthread.h
                              PATHS ${module_include_search_paths}
                              DOC "Path in which the file pthread.h is located." )
MARK_AS_ADVANCED(PTHREAD_INCLUDE_DIR)

# Look for the library.
IF(WIN32)
  FIND_LIBRARY(PTHREAD_LIBRARY NAMES pthreadVC2 
                               PATHS ${module_lib_search_paths}
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
  IF( NOT WIN32 AND UNIX )
    SET(PTHREAD_LIBRARIES ${PTHREAD_LIBRARIES} dl)
  ENDIF( NOT WIN32 AND UNIX )
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
