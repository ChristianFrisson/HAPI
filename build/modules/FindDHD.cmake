# - Find DHD
# Find the native DHD headers and libraries.
#
#  DHD_INCLUDE_DIR -  where to find DHD headers
#  DHD_LIBRARIES    - List of libraries when using DHD.
#  DHD_FOUND        - True if DHD found.


# Look for the header file.
FIND_PATH(DHD_INCLUDE_DIR NAMES dhdc.h 
                          PATHS $ENV{H3D_EXTERNAL_ROOT}/include
                                $ENV{H3D_EXTERNAL_ROOT}/include/DHD-API
                                $ENV{H3D_ROOT}/../External/include
                                $ENV{H3D_ROOT}/../External/include/DHD-API
                                ../../External/include
                                ../../External/include/DHD-API 
                                ${CMAKE_MODULE_PATH}/../../../External/include
                                ${CMAKE_MODULE_PATH}/../../../External/include/DHD-API )
MARK_AS_ADVANCED(DHD_INCLUDE_DIR)


# Look for the library.
IF(WIN32)
  FIND_LIBRARY(DHD_LIBRARY NAMES dhdms
                           PATHS $ENV{H3D_EXTERNAL_ROOT}/lib
                                 $ENV{H3D_ROOT}/../External/lib
                                 ../../External/lib
                                 ${CMAKE_MODULE_PATH}/../../../External/lib)
ELSE(WIN32)
  FIND_LIBRARY(DHD_LIBRARY NAMES dhd
                           PATHS $ENV{H3D_EXTERNAL_ROOT}/lib
                                 $ENV{H3D_ROOT}/../External/lib
                                 ../../External/lib
                                 ${CMAKE_MODULE_PATH}/../../../External/lib)

  IF(UNIX)
    FIND_LIBRARY( USB_LIBRARY NAMES usb )
    FIND_LIBRARY( PCISCAN_LIBRARY NAMES pciscan )
    MARK_AS_ADVANCED(USB_LIBRARY)
    MARK_AS_ADVANCED(PCISCAN_LIBRARY)
  ENDIF(UNIX)
ENDIF(WIN32)
MARK_AS_ADVANCED(DHD_LIBRARY)


# Copy the results to the output variables.
IF(DHD_INCLUDE_DIR AND DHD_LIBRARY)
  SET(DHD_FOUND 1)
  SET(DHD_LIBRARIES ${DHD_LIBRARY})
  SET(DHD_INCLUDE_DIR ${DHD_INCLUDE_DIR})
  IF(UNIX)
    IF(USB_LIBRARY AND PCISCAN_LIBRARY)
      SET(DHD_LIBRARIES ${DHD_LIBRARIES} ${USB_LIBRARY} ${PCISCAN_LIBRARY})
    ELSE(USB_LIBRARY AND PCISCAN_LIBRARY)
      SET(DHD_FOUND 0)
      SET(DHD_LIBRARIES)
      SET(DHD_INCLUDE_DIR)
    ENDIF(USB_LIBRARY AND PCISCAN_LIBRARY)
  ENDIF(UNIX)
ELSE(DHD_INCLUDE_DIR AND DHD_LIBRARY)
  SET(DHD_FOUND 0)
  SET(DHD_LIBRARIES)
  SET(DHD_INCLUDE_DIR)
ENDIF(DHD_INCLUDE_DIR  AND DHD_LIBRARY)

# Report the results.
IF(NOT DHD_FOUND)
  SET(DHD_DIR_MESSAGE
    "DHD was not found. Make sure to set DHD_LIBRARY and DHD_INCLUDE_DIR to the location of the library. If you do not have it you will not be able to use the Omega or Delta haptics devices from ForceDimension.")
  IF(DHD_FIND_REQUIRED)
    MESSAGE(FATAL_ERROR "${DHD_DIR_MESSAGE}")
  ELSEIF(NOT DHD_FIND_QUIETLY)
    MESSAGE(STATUS "${DHD_DIR_MESSAGE}")
  ENDIF(DHD_FIND_REQUIRED)
ENDIF(NOT DHD_FOUND)
