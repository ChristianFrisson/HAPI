# - Find VIRTUOSE
# Find the native VIRTUOSE headers and libraries.
#
#  VIRTUOSE_INCLUDE_DIR -  where to find VIRTUOSE headers
#  VIRTUOSE_LIBRARIES    - List of libraries when using VIRTUOSE.
#  VIRTUOSE_FOUND        - True if VIRTUOSE found.

include( H3DExternalSearchPath )
GET_FILENAME_COMPONENT( module_file_path ${CMAKE_CURRENT_LIST_FILE} PATH )

SET( VIRTUOSE_INCLUDE_SEARCH_PATHS "" )
SET( VIRTUOSE_LIB_SEARCH_PATHS "" )
IF( NOT MSVC14 )
  get_external_search_paths_h3d( VIRTUOSE_INCLUDE_SEARCH_PATHS VIRTUOSE_LIB_SEARCH_PATHS ${module_file_path} )
ENDIF()

# Look for the header file.
FIND_PATH(VIRTUOSE_INCLUDE_DIR NAMES VirtuoseAPI.h 
                          PATHS ${VIRTUOSE_INCLUDE_SEARCH_PATHS}
                          DOC "Path in which the file VirtuoseAPI.h is located." )
MARK_AS_ADVANCED(VIRTUOSE_INCLUDE_DIR)


# Look for the library.
IF(WIN32)
  FIND_LIBRARY(VIRTUOSE_LIBRARY NAMES virtuoseDLL
                           PATHS ${VIRTUOSE_LIB_SEARCH_PATHS}
                           DOC "Path to virtuoseDLL.lib library." )
ELSE(WIN32)
  FIND_LIBRARY(VIRTUOSE_LIBRARY NAMES virtuose
                           PATHS ${VIRTUOSE_LIB_SEARCH_PATHS}
                           DOC "Path to dhd library." )

ENDIF(WIN32)
MARK_AS_ADVANCED(VIRTUOSE_LIBRARY)


# Copy the results to the output variables.
IF(VIRTUOSE_INCLUDE_DIR AND VIRTUOSE_LIBRARY)
  SET(VIRTUOSE_FOUND 1)
  SET(VIRTUOSE_LIBRARIES ${VIRTUOSE_LIBRARY})
  SET(VIRTUOSE_INCLUDE_DIR ${VIRTUOSE_INCLUDE_DIR})
ELSE(VIRTUOSE_INCLUDE_DIR AND VIRTUOSE_LIBRARY)
  SET(VIRTUOSE_FOUND 0)
  SET(VIRTUOSE_LIBRARIES)
  SET(VIRTUOSE_INCLUDE_DIR)
ENDIF(VIRTUOSE_INCLUDE_DIR  AND VIRTUOSE_LIBRARY)

# Report the results.
IF(NOT VIRTUOSE_FOUND)
  SET( VIRTUOSE_DIR_MESSAGE
       "VIRTUOSE was not found. Make sure to set VIRTUOSE_LIBRARY" )
  SET( VIRTUOSE_DIR_MESSAGE
       "${VIRTUOSE_DIR_MESSAGE} and VIRTUOSE_INCLUDE_DIR. If you do
       not have VirtuouseAPI library you will not be able to use the
       Haption haptics device such as the Virtuose series.")
  IF(VIRTUOSE_FIND_REQUIRED)
    MESSAGE(FATAL_ERROR "${VIRTUOSE_DIR_MESSAGE}")
  ELSEIF(NOT VIRTUOSE_FIND_QUIETLY)
    MESSAGE(STATUS "${VIRTUOSE_DIR_MESSAGE}")
  ENDIF(VIRTUOSE_FIND_REQUIRED)
ENDIF(NOT VIRTUOSE_FOUND)
