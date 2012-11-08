# - Find Chai3D
# Find the native CHAI3D headers and libraries.
#
#  CHAI3D_INCLUDE_DIR -  where to find Chai3D headers
#  CHAI3D_LIBRARIES    - List of libraries when using Chai3D.
#  CHAI3D_FOUND        - True if Chai3D found.

SET( SEARCH_FOR_CHAI3D 1 )

IF(MSVC)
  INCLUDE( TestIfVCExpress )
  TestIfVCExpress()
  IF( NOT CMake_HAVE_MFC )
    SET( SEARCH_FOR_CHAI3D 0 )
  ENDIF( NOT CMake_HAVE_MFC )
ENDIF(MSVC)

IF( CMAKE_CL_64 )
  SET( LIB "lib64" )
ELSE( CMAKE_CL_64 )
  SET( LIB "lib32" )
ENDIF( CMAKE_CL_64 )

IF( SEARCH_FOR_CHAI3D )
  GET_FILENAME_COMPONENT(module_file_path ${CMAKE_CURRENT_LIST_FILE} PATH )
  # Look for the header file.
  FIND_PATH(CHAI3D_INCLUDE_DIR NAMES chai3d.h cWorld.h
                               PATHS $ENV{H3D_EXTERNAL_ROOT}/include
                                     $ENV{H3D_EXTERNAL_ROOT}/include/chai3d/include
                                     $ENV{H3D_ROOT}/../External/include
                                     $ENV{H3D_ROOT}/../External/include/chai3d/include
                                     ../../External/include
                                     ../../External/include/chai3d/include
                                     ${module_file_path}/../../../External/include
                                     ${module_file_path}/../../../External/include/chai3d/include
                               DOC "Path in which the file chai3d.h ( cWorld.h for chai3d versions earlier than 2.0 ) is located." )

  # Look for the library.
  IF(WIN32)
    SET( CHAI3D_LIBRARY_NAME chai3d_complete )
    IF( MSVC70 OR MSVC71)
      SET( CHAI3D_LIBRARY_NAME chai3d_complete_vc7 )
    ELSEIF( MSVC80 )
      SET( CHAI3D_LIBRARY_NAME chai3d_complete_vc8 )
    ELSEIF( MSVC90 )
      SET( CHAI3D_LIBRARY_NAME chai3d_complete_vc9 )
    ELSEIF( MSVC10 )             
      SET( CHAI3D_LIBRARY_NAME chai3d_complete_vc10 )
    ENDIF( MSVC70 OR MSVC71)

    FIND_LIBRARY(CHAI3D_LIBRARY NAMES ${CHAI3D_LIBRARY_NAME} chai3d-release
                                PATHS $ENV{H3D_EXTERNAL_ROOT}/${LIB}/static
                                      $ENV{H3D_ROOT}/../External/${LIB}/static
                                      ../../External/${LIB}/static
                                      ${module_file_path}/../../../External/${LIB}/static
                                DOC "Path to ${CHAI3D_LIBRARY_NAME} library." )
    IF( MSVC80 OR MSVC90 OR MSVC10 )
      FIND_LIBRARY( CHAI3D_DEBUG_LIBRARY NAMES ${CHAI3D_LIBRARY_NAME}_d chai3d-debug
                                         PATHS $ENV{H3D_EXTERNAL_ROOT}/${LIB}/static
                                               $ENV{H3D_ROOT}/../External/${LIB}/static
                                               ../../External/${LIB}/static
                                               ${module_file_path}/../../../External/${LIB}/static
                                         DOC "Path to ${CHAI3D_LIBRARY_NAME}_d library." )
    ENDIF( MSVC80 OR MSVC90 OR MSVC10 )
    MARK_AS_ADVANCED(CHAI3D_DEBUG_LIBRARY)
  ELSE(WIN32)
    FIND_LIBRARY( CHAI3D_LIBRARY NAMES chai3d_linux chai3d
                  DOC "Path to chai3d_linux (or chai3d) library." )
  ENDIF(WIN32)
ELSE( SEARCH_FOR_CHAI3D )
  SET( CHAI3D_INCLUDE_DIR "" CACHE PATH "Path to include files for chai3d. The path is to where chai3d.h ( cWorld.h for chai3d versions earlier than 2.0 ) is located." )
  SET( CHAI3D_LIBRARY "" CACHE FILEPATH "Path to chai3d library file." )
ENDIF( SEARCH_FOR_CHAI3D )

MARK_AS_ADVANCED(CHAI3D_INCLUDE_DIR)
MARK_AS_ADVANCED(CHAI3D_LIBRARY)

SET( CHAI3D_LIBRARIES_FOUND 0 )
IF( MSVC80 OR MSVC90 OR MSVC10 )
  IF( CHAI3D_LIBRARY OR CHAI3D_DEBUG_LIBRARY )
    SET( CHAI3D_LIBRARIES_FOUND 1 )
  ENDIF( CHAI3D_LIBRARY OR CHAI3D_DEBUG_LIBRARY )
ELSEIF( CHAI3D_LIBRARY )
  SET( CHAI3D_LIBRARIES_FOUND 1 )
ENDIF( MSVC80 OR MSVC90 OR MSVC10 )

# Copy the results to the output variables.
IF(CHAI3D_INCLUDE_DIR AND CHAI3D_LIBRARIES_FOUND)
  
  # The way we get the version number might be unreliable since the version
  # number is not updated in every file for previous releases of chai3d.
  # Note that this might also break in the future if chai3d changes their
  # version handling, then FindChai3D.cmake needs to be updated.
  SET( CHAI3D_VERSION "1.0" )
  SET( CHAI3D_FILE ${CHAI3D_INCLUDE_DIR}/cWorld.h )
  IF( NOT EXISTS ${CHAI3D_FILE} )
    SET( CHAI3D_FILE ${CHAI3D_INCLUDE_DIR}/chai3d.h )
  ENDIF( NOT EXISTS ${CHAI3D_FILE} )
  IF( EXISTS ${CHAI3D_FILE} )
    FILE( STRINGS ${CHAI3D_FILE} chai3d_versions REGEX "\\version[ ]*[0-9][.][0-9][.]*[0-9]*" )
    FOREACH( line ${chai3d_versions} )
      # Only get the two first numbers in order to use it for comparasion in c++.
      STRING( REGEX MATCH "[0-9][.][0-9][.]*[0-9]*" CHAI3D_VERSION ${line} )
    ENDFOREACH( line )
  ENDIF( EXISTS ${CHAI3D_FILE} )
  SET(CHAI3D_FOUND 1)
  
  IF( MSVC80 OR MSVC90 OR MSVC10 )
    IF(CHAI3D_LIBRARY)
      SET(CHAI3D_LIBRARIES optimized ${CHAI3D_LIBRARY} )
    ELSE(CHAI3D_LIBRARY)
      SET(CHAI3D_LIBRARIES optimized ${CHAI3D_LIBRARY_NAME} )
      MESSAGE( STATUS "Chai3D release libraries not found. Release build might not work." )
    ENDIF(CHAI3D_LIBRARY)

    IF(CHAI3D_DEBUG_LIBRARY)
      SET(CHAI3D_LIBRARIES ${CHAI3D_LIBRARIES} debug ${CHAI3D_DEBUG_LIBRARY} )
    ELSE(CHAI3D_DEBUG_LIBRARY)
      SET(CHAI3D_LIBRARIES ${CHAI3D_LIBRARIES} debug ${CHAI3D_LIBRARY_NAME}_d )
      MESSAGE( STATUS "Chai3D debug libraries not found. Debug build might not work." )
    ENDIF(CHAI3D_DEBUG_LIBRARY)
  ELSE( MSVC80 OR MSVC90 OR MSVC10 )
    SET(CHAI3D_LIBRARIES ${CHAI3D_LIBRARY} )
  ENDIF( MSVC80 OR MSVC90 OR MSVC10 )
  
  IF( MSVC AND SEARCH_FOR_CHAI3D )
    SET(CHAI3D_LIBRARIES ${CHAI3D_LIBRARIES} optimized "atls.lib" debug "atlsd.lib" )
  ENDIF( MSVC AND SEARCH_FOR_CHAI3D )
  SET(CHAI3D_INCLUDE_DIR ${CHAI3D_INCLUDE_DIR})
ELSE(CHAI3D_INCLUDE_DIR AND CHAI3D_LIBRARIES_FOUND)
  SET(CHAI3D_FOUND 0)
  SET(CHAI3D_LIBRARIES)
  SET(CHAI3D_INCLUDE_DIR)
ENDIF(CHAI3D_INCLUDE_DIR  AND CHAI3D_LIBRARIES_FOUND)

# Report the results.
IF(NOT CHAI3D_FOUND)
  SET( CHAI3D_DIR_MESSAGE
       "CHAI3D was not found. Make sure to set CHAI3D_LIBRARY" )
  IF( MSVC80 OR MSVC90 OR MSVC10 )
    SET( CHAI3D_DIR_MESSAGE
         "${CHAI3D_DIR_MESSAGE}, CHAI3D_DEBUG_LIBRARY")
  ENDIF( MSVC80 OR MSVC90 OR MSVC10 )
  SET( CHAI3D_DIR_MESSAGE
       "${CHAI3D_DIR_MESSAGE} and CHAI3D_INCLUDE_DIR. If you do not have chai3d you will not be able to use the Chai3DRenderer.")
  IF(Chai3D_FIND_REQUIRED)
    MESSAGE(FATAL_ERROR "${CHAI3D_DIR_MESSAGE}")
  ELSEIF(NOT Chai3D_FIND_QUIETLY)
    MESSAGE(STATUS "${CHAI3D_DIR_MESSAGE}")
  ENDIF(Chai3D_FIND_REQUIRED)
ENDIF(NOT CHAI3D_FOUND)
