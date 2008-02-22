# - Find DHD
# Find the native DHD headers and libraries.
#
#  DHD_INCLUDE_DIR -  where to find DHD headers
#  DHD_LIBRARIES    - List of libraries when using DHD.
#  DHD_FOUND        - True if DHD found.


# Look for the header file.
FIND_PATH(DHD_INCLUDE_DIR NAMES dhdc.h)
MARK_AS_ADVANCED(DHD_INCLUDE_DIR)

# Look for the library.
FIND_LIBRARY(DHD_LIBRARY NAMES dhd)
MARK_AS_ADVANCED(DHD_LIBRARY)


# Copy the results to the output variables.
IF(DHD_INCLUDE_DIR AND DHD_LIBRARY)
  SET(DHD_FOUND 1)
  SET(DHD_LIBRARIES ${DHD_LIBRARY} )
  SET(DHD_INCLUDE_DIRS ${DHD_INCLUDE_DIR})
ELSE(DHD_INCLUDE_DIR AND DHD_LIBRARY)
  SET(DHD_FOUND 0)
  SET(DHD_LIBRARIES)
  SET(DHD_INCLUDE_DIRS)
ENDIF(DHD_INCLUDE_DIR  AND DHD_LIBRARY)

# Report the results.
IF(NOT DHD_FOUND)
  SET(DHD_DIR_MESSAGE
    "DHD was not found. Make sure to set DHD_LIBRARY and DHD_INCLUDE_DIR to the location of the library. If you do not have it you will not be able to use the Omega or Delta haptics devices from ForceDimension.")
  IF(NOT DHD_FIND_QUIETLY)
    MESSAGE(STATUS "${DHD_DIR_MESSAGE}")
  ELSE(NOT DHD_FIND_QUIETLY)
    IF(DHD_FIND_REQUIRED)
      MESSAGE(FATAL_ERROR "${DHD_DIR_MESSAGE}")
    ENDIF(DHD_FIND_REQUIRED)
  ENDIF(NOT DHD_FIND_QUIETLY)
ENDIF(NOT DHD_FOUND)
