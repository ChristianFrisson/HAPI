# - Find H3DUtil
# Find the native H3DUTIL headers and libraries.
#
#  H3DUTIL_INCLUDE_DIR -  where to find H3DUtil.h, etc.
#  H3DUTIL_LIBRARIES    - List of libraries when using H3DUtil.
#  H3DUTIL_FOUND        - True if H3DUtil found.

GET_FILENAME_COMPONENT(module_file_path ${CMAKE_CURRENT_LIST_FILE} PATH )

# Look for the header file.
FIND_PATH(H3DUTIL_INCLUDE_DIR NAMES H3DUtil/H3DUtil.h 
                              PATHS $ENV{H3D_ROOT}/../H3DUtil/include
                                    ../../H3DUtil/include
                                    ${module_file_path}/../../../H3DUtil/include
                              DOC "Path in which the file H3DUtil/H3DUtil.h is located." )
MARK_AS_ADVANCED(H3DUTIL_INCLUDE_DIR)

# Look for the library.
IF( MSVC )
  SET( H3D_MSVC_VERSION 6 )
  SET( TEMP_MSVC_VERSION 1299 )
  WHILE( ${MSVC_VERSION} GREATER ${TEMP_MSVC_VERSION} )
    MATH( EXPR H3D_MSVC_VERSION "${H3D_MSVC_VERSION} + 1" )
    MATH( EXPR TEMP_MSVC_VERSION "${TEMP_MSVC_VERSION} + 100" )
  ENDWHILE( ${MSVC_VERSION} GREATER ${TEMP_MSVC_VERSION} )
  SET( H3DUTIL_NAME "H3DUtil_vc${H3D_MSVC_VERSION}" )
ELSE(MSVC)
  SET( H3DUTIL_NAME h3dutil )
ENDIF( MSVC )

FIND_LIBRARY(H3DUTIL_LIBRARY NAMES ${H3DUTIL_NAME}
                             PATHS $ENV{H3D_ROOT}/../lib
                                   ../../lib
                                   ${module_file_path}/../../../lib
                             DOC "Path to ${H3DUTIL_NAME} library." )

FIND_LIBRARY(H3DUTIL_DEBUG_LIBRARY NAMES ${H3DUTIL_NAME}_d
                                   PATHS $ENV{H3D_ROOT}/../lib
                                         ../../lib
                                         ${module_file_path}/../../../lib
                                   DOC "Path to ${H3DUTIL_NAME}_d library." )

MARK_AS_ADVANCED(H3DUTIL_LIBRARY)
MARK_AS_ADVANCED(H3DUTIL_DEBUG_LIBRARY)

IF(H3DUTIL_LIBRARY OR H3DUTIL_DEBUG_LIBRARY)
  SET( HAVE_H3DUTIL_LIBRARY 1 )
ELSE(H3DUTIL_LIBRARY OR H3DUTIL_DEBUG_LIBRARY)
  SET( HAVE_H3DUTIL_LIBRARY 0 )
ENDIF(H3DUTIL_LIBRARY OR H3DUTIL_DEBUG_LIBRARY)

# Copy the results to the output variables.
IF(H3DUTIL_INCLUDE_DIR AND HAVE_H3DUTIL_LIBRARY )

  #pthread is required for using the H3DUtil library
  FIND_PACKAGE(PTHREAD REQUIRED)
  IF(PTHREAD_FOUND)
    SET( H3DUTIL_INCLUDE_DIR ${H3DUTIL_INCLUDE_DIR} ${PTHREAD_INCLUDE_DIR} )
    SET(H3DUTIL_LIBRARIES ${PTHREAD_LIBRARIES} )
  ENDIF(PTHREAD_FOUND)

  SET(H3DUTIL_FOUND 1)
  IF(H3DUTIL_LIBRARY)
    SET(H3DUTIL_LIBRARIES optimized ${H3DUTIL_LIBRARY} )
  ELSE(H3DUTIL_LIBRARY)
    SET(H3DUTIL_LIBRARIES optimized ${H3DUTIL_NAME} )
    MESSAGE( STATUS "H3DUtil release libraries not found. Release build might not work." )
  ENDIF(H3DUTIL_LIBRARY)

  IF(H3DUTIL_DEBUG_LIBRARY)
    SET(H3DUTIL_LIBRARIES ${H3DUTIL_LIBRARIES} debug ${H3DUTIL_DEBUG_LIBRARY} )
  ELSE(H3DUTIL_DEBUG_LIBRARY)
    SET(H3DUTIL_LIBRARIES ${H3DUTIL_LIBRARIES} debug ${H3DUTIL_NAME}_d )
    MESSAGE( STATUS "H3DUtil debug libraries not found. Debug build might not work." )
  ENDIF(H3DUTIL_DEBUG_LIBRARY)
  
  SET(H3DUTIL_INCLUDE_DIR ${H3DUTIL_INCLUDE_DIR} )
  SET(H3DUTIL_LIBRARIES ${H3DUTIL_LIBRARIES} ${PTHREAD_LIBRARIES} )

ELSE(H3DUTIL_INCLUDE_DIR AND HAVE_H3DUTIL_LIBRARY )
  SET(H3DUTIL_FOUND 0)
  SET(H3DUTIL_LIBRARIES)
  SET(H3DUTIL_INCLUDE_DIR)
ENDIF(H3DUTIL_INCLUDE_DIR AND HAVE_H3DUTIL_LIBRARY )

# Report the results.
IF(NOT H3DUTIL_FOUND)
  SET(H3DUTIL_DIR_MESSAGE
    "H3DUTIL was not found. Make sure H3DUTIL_LIBRARY ( and/or H3DUTIL_DEBUG_LIBRARY ) and H3DUTIL_INCLUDE_DIR are set.")
  IF(H3DUtil_FIND_REQUIRED)
    MESSAGE(FATAL_ERROR "${H3DUTIL_DIR_MESSAGE}")
  ELSEIF(NOT H3DUtil_FIND_QUIETLY)
    MESSAGE(STATUS "${H3DUTIL_DIR_MESSAGE}")
  ENDIF(H3DUtil_FIND_REQUIRED)
ENDIF(NOT H3DUTIL_FOUND)
