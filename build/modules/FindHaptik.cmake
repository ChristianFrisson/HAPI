# - Find Haptik
# Find the native HAPTIK headers and libraries.
#
#  HAPTIK_INCLUDE_DIR -  where to find Haptik headers
#  HAPTIK_LIBRARIES    - List of libraries when using Haptik.
#  HAPTIK_FOUND        - True if Haptik found.


# Look for the header file.
FIND_PATH( HAPTIK_INCLUDE_DIR NAMES RSLib/Haptik.hpp
                              DOC "Path in which the file RSLib/Haptik.hpp is located." )
MARK_AS_ADVANCED(HAPTIK_INCLUDE_DIR)

# Look for the library.
FIND_LIBRARY( HAPTIK_LIBRARY NAMES Haptik.Library
                             DOC "Path to Haptik.Library library." )
MARK_AS_ADVANCED(HAPTIK_LIBRARY)

# Copy the results to the output variables.
IF(HAPTIK_INCLUDE_DIR AND HAPTIK_LIBRARY)
  SET(HAPTIK_FOUND 1)
  SET(HAPTIK_LIBRARIES ${HAPTIK_LIBRARY} )
  SET(HAPTIK_INCLUDE_DIR ${HAPTIK_INCLUDE_DIR})
ELSE(HAPTIK_INCLUDE_DIR AND HAPTIK_LIBRARY)
  SET(HAPTIK_FOUND 0)
  SET(HAPTIK_LIBRARIES)
  SET(HAPTIK_INCLUDE_DIR)
ENDIF(HAPTIK_INCLUDE_DIR  AND HAPTIK_LIBRARY)

# Report the results.
IF(NOT HAPTIK_FOUND)
  SET(HAPTIK_DIR_MESSAGE
    "The HAPTIK API was not found. Make sure to set HAPTIK_LIBRARY and HAPTIK_INCLUDE_DIR to the location of the library. If you do not have it you will not be able to use the haptik device.")
  IF(Haptik_FIND_REQUIRED)
    MESSAGE(FATAL_ERROR "${HAPTIK_DIR_MESSAGE}")
  ELSEIF(NOT Haptik_FIND_QUIETLY)
    MESSAGE(STATUS "${HAPTIK_DIR_MESSAGE}")
  ENDIF(Haptik_FIND_REQUIRED)
ENDIF(NOT HAPTIK_FOUND)
