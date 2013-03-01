# - Find MLHI
# Find the MLHI library
#
#  MLHI_INCLUDE_DIR -  where to find MLHI/ml_api.h
#  MLHI_LIBRARIES    - List of libraries when using MLHI
#  MLHI_FOUND        - True if MLHI found

# Look for the header file.
FIND_PATH(MLHI_INCLUDE_DIR NAMES MLHI/ml_api.h
                                PATHS /usr/local/include/
                                DOC "Path in which the file MLHI/ml_api.h is located." )
MARK_AS_ADVANCED(MLHI_INCLUDE_DIR)

# Look for the library.
FIND_LIBRARY(MLHI_LIBRARY NAMES mlhi_api_linux
                        PATHS /usr/local/lib/
                        DOC "Path to MLHI library." )
MARK_AS_ADVANCED(MLHI_LIBRARY)

# Copy the results to the output variables.
IF(MLHI_INCLUDE_DIR AND MLHI_LIBRARY)
  SET(MLHI_FOUND 1)
  SET(MLHI_LIBRARIES ${MLHI_LIBRARY} )
  SET(MLHI_INCLUDE_DIR ${MLHI_INCLUDE_DIR})
ELSE(MLHI_INCLUDE_DIR AND MLHI_LIBRARY)
  SET(MLHI_FOUND 0)
  SET(MLHI_LIBRARIES)
  SET(MLHI_INCLUDE_DIR)
ENDIF(MLHI_INCLUDE_DIR  AND MLHI_LIBRARY)

# Report the results.
IF(NOT MLHI_FOUND)
  SET(MLHI_DIR_MESSAGE
    "The MLHI API was not found. Make sure to set MLHI_LIBRARY and MLHI_INCLUDE_DIR. If you do not have the MLHI library you will not be able to use the MLHI devices.")
  IF(MLHI_FIND_REQUIRED)
    MESSAGE(FATAL_ERROR "${MLHI_DIR_MESSAGE}")
  ELSEIF(NOT MLHI_FIND_QUIETLY)
    MESSAGE(STATUS "${MLHI_DIR_MESSAGE}")
  ENDIF(MLHI_FIND_REQUIRED)
ENDIF(NOT MLHI_FOUND)
