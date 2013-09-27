# - Find fparser
# Find the native fparser headers and libraries.
#
#  fparser_INCLUDE_DIRS -  where to find fparser headers
#  fparser_LIBRARIES    - List of libraries when using fparser.
#  fparser_FOUND        - True if fparser found.
GET_FILENAME_COMPONENT(module_file_path ${CMAKE_CURRENT_LIST_FILE} PATH )

IF( CMAKE_CL_64 )
  SET( LIB "lib64" )
ELSE( CMAKE_CL_64 )
  SET( LIB "lib32" )
ENDIF( CMAKE_CL_64 )

# Look for the header file.
FIND_PATH(fparser_INCLUDE_DIR NAMES fparser.hh
                          PATHS $ENV{H3D_EXTERNAL_ROOT}/include/fparser
                                $ENV{H3D_ROOT}/../External/include/fparser
                                ../../External/include/fparser
                                ${module_file_path}/../../../External/include/fparser
                          DOC "Path in which the file fparser.hh is located." )
MARK_AS_ADVANCED(fparser_INCLUDE_DIR)


# Look for the library.
FIND_LIBRARY(fparser_LIBRARY NAMES fparser 
                             PATHS $ENV{H3D_EXTERNAL_ROOT}/${LIB}
                                   $ENV{H3D_ROOT}/../External/${LIB}
                                   ../../External/${LIB}
                                   ${module_file_path}/../../../External/${LIB}
                             DOC "Path to fparser.lib library." )
MARK_AS_ADVANCED(fparser_LIBRARY)

FIND_LIBRARY(fparser_DEBUG_LIBRARY NAMES fparser_d
                                   PATHS $ENV{H3D_EXTERNAL_ROOT}/${LIB}
                                   $ENV{H3D_ROOT}/../External/${LIB}
                                   ../../External/${LIB}
                                   ${module_file_path}/../../../External/${LIB}
                                   DOC "Path to fparser.lib library." )
MARK_AS_ADVANCED(fparser_DEBUG_LIBRARY)


# Copy the results to the output variables.
IF(fparser_INCLUDE_DIR AND fparser_LIBRARY)
  SET(fparser_FOUND 1)
  IF( fparser_DEBUG_LIBRARY )
    SET(fparser_LIBRARIES optimized ${fparser_LIBRARY} debug ${fparser_DEBUG_LIBRARY} )
  ELSE( fparser_DEBUG_LIBRARY )
    SET(fparser_LIBRARIES ${fparser_LIBRARY})
    IF( MSVC )
      MESSAGE( WARNING "fparser debug library not found. Debug build might not work properly." )
    ENDIF( MSVC )
  ENDIF( fparser_DEBUG_LIBRARY )
  
  SET(fparser_INCLUDE_DIRS ${fparser_INCLUDE_DIR})
ELSE(fparser_INCLUDE_DIR AND fparser_LIBRARY)
  SET(fparser_FOUND 0)
  SET(fparser_LIBRARIES)
  SET(fparser_INCLUDE_DIRS)
ENDIF(fparser_INCLUDE_DIR  AND fparser_LIBRARY)

# Report the results.
IF(NOT fparser_FOUND)
  SET( fparser_DIR_MESSAGE
       "fparser was not found. Make sure to set fparser_LIBRARY" )
  SET( fparser_DIR_MESSAGE
       "${fparser_DIR_MESSAGE} and fparser_INCLUDE_DIR. If you do not have fparser library you will not be able to use function parser nodes.")
  IF(fparser_FIND_REQUIRED)
    MESSAGE(FATAL_ERROR "${fparser_DIR_MESSAGE}")
  ELSEIF(NOT fparser_FIND_QUIETLY)
    MESSAGE(STATUS "${fparser_DIR_MESSAGE}")
  ENDIF(fparser_FIND_REQUIRED)
ENDIF(NOT fparser_FOUND)
