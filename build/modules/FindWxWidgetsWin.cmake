# - Find wxWidgets
# Find the native wxWidgets headers and libraries.
#
#  wxWidgets_INCLUDE_DIR -  where to find WxWidgets headers
#  wxWidgets_LIBRARIES    - List of libraries when using WxWidgets.
#  wxWidgets_FOUND        - True if WxWidgets found.

GET_FILENAME_COMPONENT(module_file_path ${CMAKE_CURRENT_LIST_FILE} PATH )

IF( CMAKE_CL_64 )
  SET( LIB "lib64" )
ELSE( CMAKE_CL_64 )
  SET( LIB "lib32" )
ENDIF( CMAKE_CL_64 )


SET( wxWidgets_LIBRARY_SEARCH_PATHS "" )
SET( wxWidgets_INCLUDE_SEARCH_PATHS "" )
IF( MSVC10 )
  SET( wxWidgets_INCLUDE_SEARCH_PATHS $ENV{H3D_EXTERNAL_ROOT}/include
                                      $ENV{H3D_ROOT}/../External/include
                                      ../../External/include
                                      ${module_file_path}/../../../External/include )
  SET( wxWidgets_LIBRARY_SEARCH_PATHS $ENV{H3D_EXTERNAL_ROOT}/${LIB}
                                      $ENV{H3D_ROOT}/../External/${LIB}
                                      ../../External/${LIB}
                                      ${module_file_path}/../../../External/${LIB} )                                      
ENDIF( MSVC10 )
IF( MSVC11 )
  SET( wxWidgets_INCLUDE_SEARCH_PATHS $ENV{H3D_EXTERNAL_ROOT}/include
                                      $ENV{H3D_ROOT}/../External/include
                                      ../../External/include
                                      ${module_file_path}/../../../External/include )
  SET( wxWidgets_LIBRARY_SEARCH_PATHS $ENV{H3D_EXTERNAL_ROOT}/vc11/${LIB}
                                      $ENV{H3D_ROOT}/../External/vc11/${LIB}
                                      ../../External/vc11/${LIB}
                                      ${module_file_path}/../../../External/vc11/${LIB} )              
ENDIF( MSVC11 )



# Look for the header file.
FIND_PATH(wxWidgets_INCLUDE_DIR NAMES wx/wx.h 
                                PATHS ${wxWidgets_INCLUDE_SEARCH_PATHS}
                                DOC "Path in which the file wx/wx.h is located." )
MARK_AS_ADVANCED(wxWidgets_INCLUDE_DIR)

IF(wxWidgets_USE_LIBS)
  SET(wxlibs ${wxWidgets_USE_LIBS})
ELSE(wxWidgets_USE_LIBS)
  SET( wxlibs core adv aui html media xrc gl qa richtext )
ENDIF(wxWidgets_USE_LIBS)

SET( wxWidgets_Win_DEBUG_LIBS "YES" CACHE BOOL "If set to YES debug libraries will be included." )
MARK_AS_ADVANCED( wxWidgets_Win_DEBUG_LIBS )
IF( wxWidgets_Win_DEBUG_LIBS )
  SET( _DBG "d" )
ELSE( wxWidgets_Win_DEBUG_LIBS )
  SET( _DBG "" )
ENDIF( wxWidgets_Win_DEBUG_LIBS )
FOREACH(WXLIB ${wxlibs} )
      FIND_LIBRARY(wxWidgets_${WXLIB}_LIBRARY
        NAMES
        wxmsw30u_${WXLIB}
        wxbase30u_${WXLIB}
        wxmsw29u_${WXLIB}
        wxbase29u_${WXLIB}
        wxmsw28${_UCD}_${WXLIB}
        wx${WXLIB}
        PATHS ${wxWidgets_LIBRARY_SEARCH_PATHS}
        DOC "Path to wx ${WXLIB} library."
        )
        MARK_AS_ADVANCED(wxWidgets_${WXLIB}_LIBRARY)
  IF( wxWidgets_Win_DEBUG_LIBS )
      # The _DBG variable is not used for 2.8 since the libraries in External works for both debug and release.
      FIND_LIBRARY(wxWidgets_${WXLIB}${_DBG}_LIBRARY
      NAMES
      wxmsw30u${_DBG}_${WXLIB}
      wxbase30u${_DBG}_${WXLIB}
      wxmsw29u${_DBG}_${WXLIB}
      wxbase29u${_DBG}_${WXLIB}
      wxmsw28${_UCD}_${WXLIB}
      wx${WXLIB}${_DBG}
      PATHS ${wxWidgets_LIBRARY_SEARCH_PATHS}
      DOC "Path to wx ${WXLIB}d library."
      )
      MARK_AS_ADVANCED(wxWidgets_${WXLIB}${_DBG}_LIBRARY)
  ENDIF( wxWidgets_Win_DEBUG_LIBS )
ENDFOREACH( WXLIB )


FIND_LIBRARY(wxWidgets_base_LIBRARY NAMES wxbase30u wxbase29u wxbase28 
                                      PATHS ${wxWidgets_LIBRARY_SEARCH_PATHS}
                                      DOC "Path to wx base library." )
MARK_AS_ADVANCED(wxWidgets_base_LIBRARY)

IF( wxWidgets_Win_DEBUG_LIBS )
  # The _DBG variable is not used for 2.8 since the libraries in External works for both debug and release.
  FIND_LIBRARY(wxWidgets_base${_DBG}_LIBRARY NAMES wxbase30u${_DBG} wxbase29u${_DBG} wxbase28
                      PATHS ${wxWidgets_LIBRARY_SEARCH_PATHS}
                      DOC "Path to wx base library." )
  MARK_AS_ADVANCED(wxWidgets_base${_DBG}_LIBRARY)
ENDIF( wxWidgets_Win_DEBUG_LIBS )

IF( wxWidgets_base_LIBRARY )
  SET( wxWidgets_FOUND 1 )
ELSE( wxWidgets_base_LIBRARY )
  SET( wxWidgets_FOUND 0 )
ENDIF( wxWidgets_base_LIBRARY )

FOREACH(WXLIB ${wxlibs} )
  IF( NOT wxWidgets_${WXLIB}_LIBRARY )
    SET( wxWidgets_FOUND 0 )
  ENDIF( NOT wxWidgets_${WXLIB}_LIBRARY )
IF( wxWidgets_Win_DEBUG_LIBS )
  IF( NOT wxWidgets_${WXLIB}${_DBG}_LIBRARY )
    SET( wxWidgets_FOUND 0 )
  ENDIF( NOT wxWidgets_${WXLIB}${_DBG}_LIBRARY )
ENDIF( wxWidgets_Win_DEBUG_LIBS )
ENDFOREACH( WXLIB )

# Copy the results to the output variables.
IF(wxWidgets_INCLUDE_DIR AND wxWidgets_base_LIBRARY AND wxWidgets_FOUND )
  SET( wxWidgets_FOUND 1)
  SET( wxWidgets_LIBRARIES comctl32 Rpcrt4 optimized  ${wxWidgets_base_LIBRARY} debug  ${wxWidgets_base${_DBG}_LIBRARY}  )
  FOREACH( WXLIB ${wxlibs} )
    SET( wxWidgets_LIBRARIES ${wxWidgets_LIBRARIES} optimized ${wxWidgets_${WXLIB}_LIBRARY} debug ${wxWidgets_${WXLIB}${_DBG}_LIBRARY} )
  ENDFOREACH( WXLIB ${wxlibs} )
  SET( wxWidgets_INCLUDE_DIR ${wxWidgets_INCLUDE_DIR})
ELSE(wxWidgets_INCLUDE_DIR AND wxWidgets_base_LIBRARY AND wxWidgets_FOUND )
  SET(wxWidgets_FOUND 0)
  SET(wxWidgets_LIBRARIES)
  SET(wxWidgets_INCLUDE_DIR)
ENDIF(wxWidgets_INCLUDE_DIR AND wxWidgets_base_LIBRARY AND wxWidgets_FOUND )

# Report the results.
IF(NOT wxWidgets_FOUND)
  SET(wxWidgets_DIR_MESSAGE
    "WxWidgets was not found. Make sure wxWidgets_core_LIBRARY, wxWidgets_base_LIBRARY")
   SET( wxWidgets_DIR_MESSAGE "${wxWidgets_DIR_MESSAGE} and wxWidgets_INCLUDE_DIR are set and other requested libs are set.")
  IF(wxWidgets_FIND_REQUIRED)
      MESSAGE(FATAL_ERROR "${wxWidgets_DIR_MESSAGE}")
  ELSEIF(NOT wxWidgets_FIND_QUIETLY)
    MESSAGE(STATUS "${wxWidgets_DIR_MESSAGE}")
  ENDIF(wxWidgets_FIND_REQUIRED)
ENDIF(NOT wxWidgets_FOUND)
