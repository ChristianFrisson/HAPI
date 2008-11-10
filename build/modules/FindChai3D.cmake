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

IF( SEARCH_FOR_CHAI3D )
  GET_FILENAME_COMPONENT(module_file_path ${CMAKE_CURRENT_LIST_FILE} PATH )
  # Look for the header file.
  FIND_PATH(CHAI3D_INCLUDE_DIR NAMES cWorld.h
                               PATHS $ENV{H3D_EXTERNAL_ROOT}/include
                                     $ENV{H3D_EXTERNAL_ROOT}/include/Chai3D/include
                                     $ENV{H3D_ROOT}/../External/include
                                     $ENV{H3D_ROOT}/../External/include/Chai3D/include
                                     ../../External/include
                                     ../../External/include/Chai3D/include
                                     ${module_file_path}/../../../External/include
                                     ${module_file_path}/../../../External/include/Chai3D/include)

  SET( CHAI3D_LIBRARY_NAME chai3d_complete )
  IF( MSVC80 )
    SET( CHAI3D_LIBRARY_NAME chai3d_complete_vc8 )
  ELSEIF( MSVC90 )
    SET( CHAI3D_LIBRARY_NAME chai3d_complete_vc9 )
  ENDIF( MSVC80 )

  # Look for the library.
  IF(WIN32)
    FIND_LIBRARY(CHAI3D_LIBRARY NAMES ${CHAI3D_LIBRARY_NAME}  
                                PATHS $ENV{H3D_EXTERNAL_ROOT}/lib
                                      $ENV{H3D_ROOT}/../External/lib
                                      ../../External/lib
                                      ${module_file_path}/../../../External/lib )
    IF( MSVC80 OR MSVC90 )
      FIND_LIBRARY( CHAI3D_DEBUG_LIBRARY NAMES ${CHAI3D_LIBRARY_NAME}_d
                                         PATHS $ENV{H3D_EXTERNAL_ROOT}/lib
                                               $ENV{H3D_ROOT}/../External/lib
                                               ../../External/lib
                                               ${module_file_path}/../../../External/lib )
    ENDIF( MSVC80 OR MSVC90 )
    MARK_AS_ADVANCED(CHAI3D_DEBUG_LIBRARY)
  ELSE(WIN32)
    FIND_LIBRARY(CHAI3D_LIBRARY NAMES chai3d_linux)
  ENDIF(WIN32)
ELSE( SEARCH_FOR_CHAI3D )
  SET( CHAI3D_INCLUDE_DIR "" CACHE PATH "Path to include files for Chai3d." )
  SET( CHAI3D_LIBRARY "" CACHE FILEPATH "Path to Chai3d library file." )
ENDIF( SEARCH_FOR_CHAI3D )

MARK_AS_ADVANCED(CHAI3D_INCLUDE_DIR)
MARK_AS_ADVANCED(CHAI3D_LIBRARY)

SET( CHAI3D_LIBRARIES_FOUND 0 )
IF( MSVC80 OR MSVC90 )
  IF( CHAI3D_LIBRARY OR CHAI3D_DEBUG_LIBRARY )
    SET( CHAI3D_LIBRARIES_FOUND 1 )
  ENDIF( CHAI3D_LIBRARY OR CHAI3D_DEBUG_LIBRARY )
ELSEIF( CHAI3D_LIBRARY )
  SET( CHAI3D_LIBRARIES_FOUND 1 )
ENDIF( MSVC80 OR MSVC90 )

# Copy the results to the output variables.
IF(CHAI3D_INCLUDE_DIR AND CHAI3D_LIBRARIES_FOUND)
  SET(CHAI3D_FOUND 1)
  
  IF( MSVC80 OR MSVC90 )
    IF(CHAI3D_LIBRARY)
      SET(CHAI3D_LIBRARIES optimized ${CHAI3D_LIBRARY} )
    ELSE(CHAI3D_LIBRARY)
      SET(CHAI3D_LIBRARIES optimized ${CHAI3D_LIBRARY_NAME} )
      MESSAGE( STATUS, "Chai3D release libraries not found. Release build might not work." )
    ENDIF(CHAI3D_LIBRARY)

    IF(CHAI3D_DEBUG_LIBRARY)
      SET(CHAI3D_LIBRARIES ${CHAI3D_LIBRARIES} debug ${CHAI3D_DEBUG_LIBRARY} )
    ELSE(CHAI3D_DEBUG_LIBRARY)
      SET(CHAI3D_LIBRARIES ${CHAI3D_LIBRARIES} debug ${CHAI3D_LIBRARY_NAME}_d )
      MESSAGE( STATUS, "Chai3D debug libraries not found. Debug build might not work." )
    ENDIF(CHAI3D_DEBUG_LIBRARY)
  ELSE( MSVC80 OR MSVC90 )
    SET(CHAI3D_LIBRARIES ${CHAI3D_LIBRARY} )
  ENDIF( MSVC80 OR MSVC90 )
  
  IF( MSVC AND SEARCH_FOR_CHAI3D )
    SET(CHAI3D_LIBRARIES ${CHAI3D_LIBRARIES} "atls.lib" )
  ENDIF( MSVC AND SEARCH_FOR_CHAI3D )
  SET(CHAI3D_INCLUDE_DIR ${CHAI3D_INCLUDE_DIR})
ELSE(CHAI3D_INCLUDE_DIR AND CHAI3D_LIBRARIES_FOUND)
  SET(CHAI3D_FOUND 0)
  SET(CHAI3D_LIBRARIES)
  SET(CHAI3D_INCLUDE_DIR)
ENDIF(CHAI3D_INCLUDE_DIR  AND CHAI3D_LIBRARIES_FOUND)

# Report the results.
IF(NOT CHAI3D_FOUND)
  SET(CHAI3D_DIR_MESSAGE
    "CHAI3D was not found. Make sure to set CHAI3D_LIBRARY and CHAI3D_INCLUDE_DIR to the location of the library. If you do not have it you will not be able to use the Chai3DRenderer.")
  IF(Chai3D_FIND_REQUIRED)
    MESSAGE(FATAL_ERROR "${CHAI3D_DIR_MESSAGE}")
  ELSEIF(NOT Chai3D_FIND_QUIETLY)
    MESSAGE(STATUS "${CHAI3D_DIR_MESSAGE}")
  ENDIF(Chai3D_FIND_REQUIRED)
ENDIF(NOT CHAI3D_FOUND)
