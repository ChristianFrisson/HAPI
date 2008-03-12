# - Find Chai3D
# Find the native CHAI3D headers and libraries.
#
#  CHAI3D_INCLUDE_DIR -  where to find Chai3D headers
#  CHAI3D_LIBRARIES    - List of libraries when using Chai3D.
#  CHAI3D_FOUND        - True if Chai3D found.


# Look for the header file.
FIND_PATH(CHAI3D_INCLUDE_DIR NAMES cWorld.h
                             PATHS $ENV{H3D_EXTERNAL_ROOT}/include
                                   $ENV{H3D_EXTERNAL_ROOT}/include/Chai3D/include
                                   ../../External/include
                                   ../../External/include/Chai3D/include)
MARK_AS_ADVANCED(CHAI3D_INCLUDE_DIR)


# Look for the library.
IF(WIN32)
FIND_LIBRARY(CHAI3D_LIBRARY NAMES chai3d_complete  
                            PATHS $ENV{H3D_EXTERNAL_ROOT}/lib
                                  ../../External/lib )
ELSE(WIN32)
FIND_LIBRARY(CHAI3D_LIBRARY NAMES chai3d_linux)
ENDIF(WIN32)
MARK_AS_ADVANCED(CHAI3D_LIBRARY)

# Copy the results to the output variables.
IF(CHAI3D_INCLUDE_DIR AND CHAI3D_LIBRARY)
  SET(CHAI3D_FOUND 1)
  SET(CHAI3D_LIBRARIES ${CHAI3D_LIBRARY} )
  SET(CHAI3D_INCLUDE_DIRS ${CHAI3D_INCLUDE_DIR})
ELSE(CHAI3D_INCLUDE_DIR AND CHAI3D_LIBRARY)
  SET(CHAI3D_FOUND 0)
  SET(CHAI3D_LIBRARIES)
  SET(CHAI3D_INCLUDE_DIRS)
ENDIF(CHAI3D_INCLUDE_DIR  AND CHAI3D_LIBRARY)

# Report the results.
IF(NOT CHAI3D_FOUND)
  SET(CHAI3D_DIR_MESSAGE
    "CHAI3D was not found. Make sure to set CHAI3D_LIBRARY and CHAI3D_INCLUDE_DIR to the location of the library. If you do not have it you will not be able to use the Chai3DRenderer.")
  IF(NOT CHAI3D_FIND_QUIETLY)
    MESSAGE(STATUS "${CHAI3D_DIR_MESSAGE}")
  ELSE(NOT CHAI3D_FIND_QUIETLY)
    IF(CHAI3D_FIND_REQUIRED)
      MESSAGE(FATAL_ERROR "${CHAI3D_DIR_MESSAGE}")
    ENDIF(CHAI3D_FIND_REQUIRED)
  ENDIF(NOT CHAI3D_FIND_QUIETLY)
ENDIF(NOT CHAI3D_FOUND)
