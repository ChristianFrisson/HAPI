# - Find DirectX on windows
#
#  DirectX_INCLUDE_DIR -  where to find DirectX headers
#  DirectX_LIBRARIES    - List of libraries when using DirectX.
#  DirectX_FOUND        - True if DirectX found.


# Look for the header file.
FIND_PATH( DirectX_INCLUDE_DIR NAMES d3d9.h
           PATHS $ENV{DXSDK_DIR}/Include
           DOC "Path in which the file d3d9.h is located." )
MARK_AS_ADVANCED(DirectX_INCLUDE_DIR)

# Look for the library.
FIND_LIBRARY( DirectX_d3d9_LIBRARY NAMES d3d9
              PATHS $ENV{DXSDK_DIR}/Lib/$ENV{PROCESSOR_ARCHITECTURE}
              DOC "Path to d3d9 library." )

FIND_LIBRARY( DirectX_d3dx9_LIBRARY NAMES d3dx9
              PATHS $ENV{DXSDK_DIR}/Lib/$ENV{PROCESSOR_ARCHITECTURE}
              DOC "Path to d3dx9 library." )

MARK_AS_ADVANCED(DirectX_d3d9_LIBRARY)
MARK_AS_ADVANCED(DirectX_d3dx9_LIBRARY)

# Copy the results to the output variables.
IF(DirectX_INCLUDE_DIR AND DirectX_d3d9_LIBRARY AND DirectX_d3dx9_LIBRARY )
  SET(DirectX_FOUND 1)
  SET(DirectX_LIBRARIES ${DirectX_d3d9_LIBRARY} ${DirectX_d3dx9_LIBRARY} )
  SET(DirectX_INCLUDE_DIR ${DirectX_INCLUDE_DIR})
ELSE(DirectX_INCLUDE_DIR AND DirectX_d3d9_LIBRARY AND DirectX_d3dx9_LIBRARY )
  SET(DirectX_FOUND 0)
  SET(DirectX_LIBRARIES)
  SET(DirectX_INCLUDE_DIR)
ENDIF(DirectX_INCLUDE_DIR AND DirectX_d3d9_LIBRARY AND DirectX_d3dx9_LIBRARY )

# Report the results.
IF(NOT DirectX_FOUND)
  SET(DirectX_DIR_MESSAGE
    "DirectX was not found. Make sure to set DirectX_d3d9_LIBRARY and DirectX_d3dx9_LIBRARY and DirectX_INCLUDE_DIR to the location of the library and include files. If you do not have it you will not be able to build the DirectXExample of HAPI.")
  IF(DirectX_FIND_REQUIRED)
    MESSAGE(FATAL_ERROR "${DirectX_DIR_MESSAGE}")
  ELSEIF(NOT DirectX_FIND_QUIETLY)
    MESSAGE(STATUS "${DirectX_DIR_MESSAGE}")
  ENDIF(DirectX_FIND_REQUIRED)
ENDIF(NOT DirectX_FOUND)
