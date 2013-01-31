# - Find FalconAPI
# Find the native FALCONAPI headers and libraries.
#
#  FALCONAPI_INCLUDE_DIR -  where to find hdl.h, etc.
#  FALCONAPI_LIBRARIES    - List of libraries when using FalconAPI.
#  FALCONAPI_FOUND        - True if FalconAPI found.

SET( program_files_path "" )
IF( CMAKE_CL_64 )
  SET( program_files_path "$ENV{ProgramW6432}" )
ELSE( CMAKE_CL_64 )
  SET( program_files_path "$ENV{ProgramFiles}" )
ENDIF( CMAKE_CL_64 )

SET( FALCON_INCLUDE_SEARCH_PATH "" )
SET( FALCON_LIB_SEARCH_PATH "" )
IF( WIN32 )
  IF( NOT CMAKE_CL_64 )
    SET( FALCON_INCLUDE_SEARCH_PATH $ENV{NOVINT_FALCON_SUPPORT}/include
                                    ${program_files_path}/Novint/Falcon/HDAL/include
                                    ${program_files_path}/Novint/HDAL_SDK_2.1.3/include
                                    $ENV{NOVINT_DEVICE_SUPPORT}/include )
    SET( FALCON_LIB_SEARCH_PATH $ENV{NOVINT_FALCON_SUPPORT}/lib
                                ${program_files_path}/Novint/Falcon/HDAL/lib
                                ${program_files_path}/Novint/HDAL_SDK_2.1.3/lib
                                $ENV{NOVINT_DEVICE_SUPPORT}/lib )
  ENDIF( NOT CMAKE_CL_64 )
ENDIF( WIN32 )

# Look for the header file.
FIND_PATH(FALCONAPI_INCLUDE_DIR NAMES hdl/hdl.h 
                                PATHS ${FALCON_INCLUDE_SEARCH_PATH}
                                DOC "Path in which the file hdl/hdl.h is located. File is part of HDAL SDK." )

MARK_AS_ADVANCED(FALCONAPI_INCLUDE_DIR)

# Look for the library.
FIND_LIBRARY(FALCONAPI_HDL_LIBRARY NAMES hdl 
                        PATHS ${FALCON_LIB_SEARCH_PATH}
                        DOC "Path to hdl library. Library is part of HDAL SDK." )
MARK_AS_ADVANCED(FALCONAPI_HDL_LIBRARY)

# Copy the results to the output variables.
IF(FALCONAPI_INCLUDE_DIR AND FALCONAPI_HDL_LIBRARY)
  SET(FALCONAPI_FOUND 1)
  SET(FALCONAPI_LIBRARIES ${FALCONAPI_HDL_LIBRARY} )
  SET(FALCONAPI_INCLUDE_DIR ${FALCONAPI_INCLUDE_DIR})
ELSE(FALCONAPI_INCLUDE_DIR AND FALCONAPI_HDL_LIBRARY)
  SET(FALCONAPI_FOUND 0)
  SET(FALCONAPI_LIBRARIES)
  SET(FALCONAPI_INCLUDE_DIR)
ENDIF(FALCONAPI_INCLUDE_DIR  AND FALCONAPI_HDL_LIBRARY)

# Report the results.
IF(NOT FALCONAPI_FOUND)
  SET(FALCONAPI_DIR_MESSAGE
    "The Novint Falcon API(HDAL SDK) was not found. Make sure to set FALCONAPI_HDL_LIBRARY and FALCONAPI_INCLUDE_DIR. If you do not have it you will not be able to use the Novint Falcon Haptics device.")
  IF( CMAKE_CL_64 )
    SET( FALCONAPI_DIR_MESSAGE "${FALCONAPI_DIR_MESSAGE} NOVINT HAS NOT RELEASED A 64 BIT VERSION OF HDAL SDK YET." )
  ENDIF( CMAKE_CL_64 )
  IF(FalconAPI_FIND_REQUIRED)
    MESSAGE(FATAL_ERROR "${FALCONAPI_DIR_MESSAGE}")
  ELSEIF(NOT FalconAPI_FIND_QUIETLY)
    MESSAGE(STATUS "${FALCONAPI_DIR_MESSAGE}")
  ENDIF(FalconAPI_FIND_REQUIRED)
ENDIF(NOT FALCONAPI_FOUND)
