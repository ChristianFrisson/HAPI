# - Find SimballMedical
# Find the native SimballMedical headers and libraries.
#
#  SIMBALLMEDICAL_INCLUDE_DIR -  where to find SimballMedicalHID.h, etc.
#  SIMBALLMEDICAL_LIBRARIES    - List of libraries when using SimballMedical.
#  SIMBALLMEDICAL_FOUND        - True if SimballMedical found.


# Look for the header file.
FIND_PATH(SIMBALLMEDICAL_INCLUDE_DIR NAMES Simball/SimballMedicalHID.h
                                PATHS $ENV{H3D_EXTERNAL_ROOT}/include
                                      $ENV{H3D_ROOT}/../External/include
                                      ../../External/include
                                      ${CMAKE_MODULE_PATH}/../../../External/include )
MARK_AS_ADVANCED(SIMBALLMEDICAL_INCLUDE_DIR)

# Look for the library.
FIND_LIBRARY(SIMBALLMEDICAL_LIBRARY NAMES SimballMedicalHID 
                        PATHS $ENV{H3D_EXTERNAL_ROOT}/lib
                              $ENV{H3D_ROOT}/../External/lib
                              ../../External/lib
                              ${CMAKE_MODULE_PATH}/../../../External/lib)
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
    "The SimballMedical API was not found. Make sure to set SIMBALLMEDICAL_LIBRARY and SIMBALLMEDICAL_INCLUDE_DIR to the location of the library. If you do not have it you will not be able to use the Simball device.")
  IF(SIMBALLMEDICAL_FIND_REQUIRED)
    MESSAGE(FATAL_ERROR "${SIMBALLMEDICAL_DIR_MESSAGE}")
  ELSEIF(NOT SIMBALLMEDICAL_FIND_QUIETLY)
    MESSAGE(STATUS "${SIMBALLMEDICAL_DIR_MESSAGE}")
  ENDIF(SIMBALLMEDICAL_FIND_REQUIRED)
ENDIF(NOT SIMBALLMEDICAL_FOUND)
