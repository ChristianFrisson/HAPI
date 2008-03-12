# - Find FalconAPI
# Find the native FALCONAPI headers and libraries.
#
#  FALCONAPI_INCLUDE_DIR -  where to find hdl.h, etc.
#  FALCONAPI_LIBRARIES    - List of libraries when using FalconAPI.
#  FALCONAPI_FOUND        - True if FalconAPI found.


# Look for the header file.
FIND_PATH(FALCONAPI_INCLUDE_DIR NAMES hdl/hdl.h 
                                PATHS $ENV{NOVINT_FALCON_SUPPORT}/include
                                      "/Program Files/Novint/Falcon/HDAL/include")
MARK_AS_ADVANCED(FALCONAPI_INCLUDE_DIR)

# Look for the library.
FIND_LIBRARY(HDL_LIBRARY NAMES hdl 
                        PATHS $ENV{NOVINT_FALCON_SUPPORT}/lib
                              "/Program Files/Novint/Falcon/HDAL/lib")
MARK_AS_ADVANCED(HDL_LIBRARY)

# Copy the results to the output variables.
IF(FALCONAPI_INCLUDE_DIR AND HDL_LIBRARY)
  SET(FALCONAPI_FOUND 1)
  SET(FALCONAPI_LIBRARIES ${HD_LIBRARY} ${HL_LIBRARY} ${HDU_LIBRARY})
  SET(FALCONAPI_INCLUDE_DIRS ${FALCONAPI_INCLUDE_DIR})
ELSE(FALCONAPI_INCLUDE_DIR AND HDL_LIBRARY)
  SET(FALCONAPI_FOUND 0)
  SET(FALCONAPI_LIBRARIES)
  SET(FALCONAPI_INCLUDE_DIRS)
ENDIF(FALCONAPI_INCLUDE_DIR  AND HDL_LIBRARY)

# Report the results.
IF(NOT FALCONAPI_FOUND)
  SET(FALCONAPI_DIR_MESSAGE
    "The Novint Falcon API(HDAL) was not found. Make sure to set FALCONAPI_LIBRARY and FALCONAPI_INCLUDE_DIR to the location of the library. If you do not have it you will not be able to use the Novint Falcon Haptics device.")
  IF(NOT FALCONAPI_FIND_QUIETLY)
    MESSAGE(STATUS "${FALCONAPI_DIR_MESSAGE}")
  ELSE(NOT FALCONAPI_FIND_QUIETLY)
    IF(FALCONAPI_FIND_REQUIRED)
      MESSAGE(FATAL_ERROR "${FALCONAPI_DIR_MESSAGE}")
    ENDIF(FALCONAPI_FIND_REQUIRED)
  ENDIF(NOT FALCONAPI_FIND_QUIETLY)
ENDIF(NOT FALCONAPI_FOUND)
