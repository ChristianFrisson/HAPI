# - Find SimballMedical
# Find the native SimballMedical headers and libraries.
#
#  SIMBALLMEDICAL_INCLUDE_DIR -  where to find SimballMedicalHID.h, etc.
#  SIMBALLMEDICAL_LIBRARIES    - List of libraries when using SimballMedical.
#  SIMBALLMEDICAL_FOUND        - True if SimballMedical found.

include( H3DExternalSearchPath )
GET_FILENAME_COMPONENT( module_file_path ${CMAKE_CURRENT_LIST_FILE} PATH )
get_external_search_paths_h3d( module_include_search_paths module_lib_search_paths ${module_file_path} )

# Look for the header file.
FIND_PATH(SIMBALLMEDICAL_INCLUDE_DIR NAMES Simball/SimballMedicalHID.h
                                PATHS ${module_include_search_paths}
                                DOC "Path in which the file Simball/SimballMedicalHID.h is located." )
MARK_AS_ADVANCED(SIMBALLMEDICAL_INCLUDE_DIR)

# Look for the library.
FIND_LIBRARY(SIMBALLMEDICAL_LIBRARY NAMES SimballMedicalHID 
                        PATHS ${module_lib_search_paths}
                        DOC "Path to SimballMedicalHID library." )
MARK_AS_ADVANCED(SIMBALLMEDICAL_LIBRARY)

# Copy the results to the output variables.
IF(SIMBALLMEDICAL_INCLUDE_DIR AND SIMBALLMEDICAL_LIBRARY)
  SET(SIMBALLMEDICAL_FOUND 1)
  SET(SIMBALLMEDICAL_LIBRARIES ${SIMBALLMEDICAL_LIBRARY} )
  SET(SIMBALLMEDICAL_INCLUDE_DIR ${SIMBALLMEDICAL_INCLUDE_DIR})
ELSE(SIMBALLMEDICAL_INCLUDE_DIR AND SIMBALLMEDICAL_LIBRARY)
  SET(SIMBALLMEDICAL_FOUND 0)
  SET(SIMBALLMEDICAL_LIBRARIES)
  SET(SIMBALLMEDICAL_INCLUDE_DIR)
ENDIF(SIMBALLMEDICAL_INCLUDE_DIR  AND SIMBALLMEDICAL_LIBRARY)

# Report the results.
IF(NOT SIMBALLMEDICAL_FOUND)
  SET(SIMBALLMEDICAL_DIR_MESSAGE
    "The SimballMedical API was not found. Make sure to set SIMBALLMEDICAL_LIBRARY and SIMBALLMEDICAL_INCLUDE_DIR. If you do not have the SimballMedicalHID library you will not be able to use the Simball device.")
  IF(SimballMedical_FIND_REQUIRED)
    MESSAGE(FATAL_ERROR "${SIMBALLMEDICAL_DIR_MESSAGE}")
  ELSEIF(NOT SimballMedical_FIND_QUIETLY)
    MESSAGE(STATUS "${SIMBALLMEDICAL_DIR_MESSAGE}")
  ENDIF(SimballMedical_FIND_REQUIRED)
ENDIF(NOT SIMBALLMEDICAL_FOUND)
