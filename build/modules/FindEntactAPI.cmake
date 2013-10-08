# - Find ENTACTAPI
# Find the native ENTACTAPI headers and libraries.
#
#  ENTACTAPI_INCLUDE_DIR -  where to find ENTACTAPI headers
#  ENTACTAPI_LIBRARIES    - List of libraries when using ENTACTAPI.
#  ENTACTAPI_FOUND        - True if ENTACTAPI found.

GET_FILENAME_COMPONENT(module_file_path ${CMAKE_CURRENT_LIST_FILE} PATH )

IF( CMAKE_CL_64 )
  SET( LIB "lib64" )
ELSE( CMAKE_CL_64 )
  SET( LIB "lib32" )
ENDIF( CMAKE_CL_64 )

SET( ENTACT_INCLUDE_SEARCH_PATHS "" )
SET( ENTACT_LIB_SEARCH_PATHS "" )
IF( CMAKE_SYSTEM_NAME STREQUAL "Windows" AND CMAKE_SYSTEM_VERSION VERSION_GREATER "5.9999" )
  SET( ENTACT_INCLUDE_SEARCH_PATHS $ENV{H3D_EXTERNAL_ROOT}/include
                                   $ENV{H3D_ROOT}/../External/include
                                   ../../External/include
                                   ${module_file_path}/../../../External/include )
  SET( ENTACT_LIB_SEARCH_PATHS $ENV{H3D_EXTERNAL_ROOT}/${LIB}
                               $ENV{H3D_ROOT}/../External/${LIB}
                               ../../External/${LIB}
                               ${module_file_path}/../../../External/${LIB} )
ENDIF( CMAKE_SYSTEM_NAME STREQUAL "Windows" AND CMAKE_SYSTEM_VERSION VERSION_GREATER "5.9999" )

# Look for the header file.
FIND_PATH(ENTACTAPI_INCLUDE_DIR NAMES EntactAPI.h 
                          PATHS ${ENTACT_INCLUDE_SEARCH_PATHS}
                          DOC "Path in which the file EntactAPI.h is located." )
MARK_AS_ADVANCED(ENTACTAPI_INCLUDE_DIR)


# Look for the library.
IF(WIN32)
  FIND_LIBRARY(ENTACTAPI_LIBRARY NAMES EntactAPI 
                           PATHS ${ENTACT_LIB_SEARCH_PATHS}
                           DOC "Path to EntactAPI.lib library." )
ELSE(WIN32)
  FIND_LIBRARY(ENTACTAPI_LIBRARY NAMES entact
                           PATHS ${ENTACT_LIB_SEARCH_PATHS}
                           DOC "Path to EntactAPI library." )

ENDIF(WIN32)
MARK_AS_ADVANCED(ENTACTAPI_LIBRARY)


# Copy the results to the output variables.
IF(ENTACTAPI_INCLUDE_DIR AND ENTACTAPI_LIBRARY)
  SET(ENTACTAPI_FOUND 1)
  SET(ENTACTAPI_LIBRARIES ${ENTACTAPI_LIBRARY})
  SET(ENTACTAPI_INCLUDE_DIR ${ENTACTAPI_INCLUDE_DIR})
ELSE(ENTACTAPI_INCLUDE_DIR AND ENTACTAPI_LIBRARY)
  SET(ENTACTAPI_FOUND 0)
  SET(ENTACTAPI_LIBRARIES)
  SET(ENTACTAPI_INCLUDE_DIR)
ENDIF(ENTACTAPI_INCLUDE_DIR  AND ENTACTAPI_LIBRARY)

# Report the results.
IF(NOT ENTACTAPI_FOUND)
  SET( ENTACTAPI_DIR_MESSAGE
       "Entact API was not found. Make sure to set ENTACTAPI_LIBRARY" )
  SET( ENTACTAPI_DIR_MESSAGE
       "${ENTACTAPI_DIR_MESSAGE} and ENTACTAPI_INCLUDE_DIR. If you do not have EntactAPI library you will not be able to use the Entact haptics device.")
  IF(ENTACTAPI_FIND_REQUIRED)
    MESSAGE(FATAL_ERROR "${ENTACTAPI_DIR_MESSAGE}")
  ELSEIF(NOT ENTACTAPI_FIND_QUIETLY)
    MESSAGE(STATUS "${ENTACTAPI_DIR_MESSAGE}")
  ENDIF(ENTACTAPI_FIND_REQUIRED)
ENDIF(NOT ENTACTAPI_FOUND)
