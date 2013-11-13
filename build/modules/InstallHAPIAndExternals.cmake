# This file detects HAPI libraries/include/binaries and the
# external libraries/includes/binaries a particular
# HAPI source is built with and installs that.
# It requires that HAPI_INCLUDE_DIR is set to HAPI/include of
# the build that should be checked.
# HAPI_CMAKE_INSTALL_PREFIX must be set to the CMAKE_INSTALL_PREFIX
# used when installing HAPI (basically the directory above the directory
# in which HAPI binaries and libraries are installed.
# EXTERNAL_ROOT should be set to the External directory that comes with
# H3D.
# FEATURES_TO_INSTALL should be set to a list of pairs.  The first
# item in each pair should be a specific word. The second item in the
# pair should be the directory to install into. The predefined words are:
# "include" - Include directories should be installed.
# "lib" - Release libraries should be installed.
# "bin" - Debug binaries should be installed.
# Output variables are:
# HAPI_INCLUDE_FILES_INSTALL - Contains a list of include files that
# was used when the checked HAPI version was built.
# HAPI_INCLUDE_DIRECTORIES_INSTALL - Contains a list of directories that
# contains files used when the checked HAPI version was built.
# HAPI_LIBRARIES_INSTALL - Contains a list of libraries that
# was used when the checked HAPI version was built.
# HAPI_BINARIES_INSTALL - Contains a list of binaries that
# the built HAPI version needs.
# TODO, IMPLEMENT FOR OTHER THAN WINDOWS IF IT MAKES SENSE TO DO THAT.
# IMPLEMENT for other than MSVC10.
# GET INCLUDE_DIR AND LIBS FROM FIND_MODULES used by HAPI?
# IMPLEMENT to HANDLE debug libs/bins and configure to include them or not.

INCLUDE( InstallH3DUtilAndExternals )
IF( NOT h3d_release_only_warning )
  SET( h3d_release_only_warning TRUE )
  MESSAGE( "NOTE: Packing will only be done properly in Release build, not RelWithDebInfo, MinSizeRel or Debug" )
ENDIF( NOT h3d_release_only_warning )

IF( NOT DEFINED HAPI_INCLUDE_DIR )
  SET( HAPI_INCLUDE_DIR "" CACHE BOOL "Path to HAPI/include." )
ENDIF( NOT DEFINED HAPI_INCLUDE_DIR )

IF( WIN32 )
  IF( NOT DEFINED EXTERNAL_ROOT )
    SET( EXTERNAL_ROOT_DEFAULT "" )
    IF( NOT "$ENV{H3D_EXTERNAL_ROOT}" STREQUAL  "" )
      SET( EXTERNAL_ROOT_DEFAULT "$ENV{H3D_EXTERNAL_ROOT}" )
    ENDIF( NOT "$ENV{H3D_EXTERNAL_ROOT}" STREQUAL  "" )
    SET( EXTERNAL_ROOT "${EXTERNAL_ROOT_DEFAULT}" CACHE PATH "Path to External directory that comes with H3D." )
  ENDIF( NOT DEFINED EXTERNAL_ROOT )
ENDIF( WIN32 )

IF( NOT DEFINED HAPI_CMAKE_INSTALL_PREFIX )
  SET( HAPI_CMAKE_INSTALL_PREFIX_DEFAULT "" )
  IF( TARGET HAPI )
    SET( HAPI_CMAKE_INSTALL_PREFIX_DEFAULT ${CMAKE_INSTALL_PREFIX} )
  ELSEIF( NOT "${H3D_ROOT_CMAKE_PATH}" STREQUAL  "" )
    SET( HAPI_CMAKE_INSTALL_PREFIX_DEFAULT "${H3D_ROOT_CMAKE_PATH}/.." )
  ENDIF( TARGET HAPI )
  SET( HAPI_CMAKE_INSTALL_PREFIX ${HAPI_CMAKE_INSTALL_PREFIX_DEFAULT} CACHE PATH "Set this to the CMAKE_INSTALL_PREFIX directory used when installing HAPI. It is assumed that HAPI is installed in bin32/bin64 and lib32/lib64." )
  MARK_AS_ADVANCED(HAPI_CMAKE_INSTALL_PREFIX)
ENDIF( NOT DEFINED HAPI_CMAKE_INSTALL_PREFIX )

SET( HAPI_INCLUDE_FILES_INSTALL "" CACHE INTERNAL "List of External include files used by this compiled version of HAPI." )
SET( HAPI_INCLUDE_DIRECTORIES_INSTALL "" CACHE INTERNAL "List of External include directories used by this compiled version of HAPI." )
SET( HAPI_LIBRARIES_INSTALL "" CACHE INTERNAL "List of External libraries used by this compiled version of HAPI." )
SET( HAPI_BINARIES_INSTALL "" CACHE INTERNAL "List of External binaries used by this compiled version of HAPI." )

SET( HAPI_bin "bin32" )
SET( HAPI_lib "lib32" )
IF( CMAKE_SIZEOF_VOID_P EQUAL 8 )
  SET( HAPI_bin "bin64" )
  SET( HAPI_lib "lib64" )
ENDIF( CMAKE_SIZEOF_VOID_P EQUAL 8 )
SET( HAPI_EXTERNAL_BIN "${EXTERNAL_ROOT}/${HAPI_bin}" )
SET( HAPI_EXTERNAL_LIB "${EXTERNAL_ROOT}/${HAPI_lib}" )

IF( HAPI_INCLUDE_DIR AND EXTERNAL_ROOT)
  set( externals_to_look_for "" )
  IF( MSVC )
    SET( H3D_MSVC_VERSION 6 )
    SET( TEMP_MSVC_VERSION 1299 )
    WHILE( ${MSVC_VERSION} GREATER ${TEMP_MSVC_VERSION} )
      MATH( EXPR H3D_MSVC_VERSION "${H3D_MSVC_VERSION} + 1" )
      MATH( EXPR TEMP_MSVC_VERSION "${TEMP_MSVC_VERSION} + 100" )
    ENDWHILE( ${MSVC_VERSION} GREATER ${TEMP_MSVC_VERSION} )
  
    ## TODO: Somehow check if TEEM is compiled against BZPI2, PNG and/or ZLIB, probably have to use find modules.
    # When the first item for an external entry is only "#define" then it will always be included.
    set( externals_to_look_for "#define HAVE_OPENHAPTICS" )
    IF( NOT TARGET HAPI )
      set( externals_to_look_for ${externals_to_look_for}
                               "include" "../OpenHapticsRenderer/include/HAPI" )
    ENDIF( NOT TARGET HAPI )
    set( externals_to_look_for ${externals_to_look_for}
                               "lib" "OpenHapticsRenderer"
                               "bin" "OpenHapticsRenderer"
                               "warning" "NOTE: HAPI compiled with OpenHaptics support. Test that application starts on system without OpenHaptics before distributing package if you do not distribute it yourself."
                               
                               "#define HAVE_CHAI3D" )
    IF( NOT TARGET HAPI )
      set( externals_to_look_for ${externals_to_look_for}
                               "include" "../Chai3DRenderer/include/HAPI" )
    ENDIF( NOT TARGET HAPI )
    set( externals_to_look_for ${externals_to_look_for}
                               "include" "chai3d"
                               "lib" "chai3d_complete_vc${H3D_MSVC_VERSION}" "Chai3DRenderer"
                               "bin" "Chai3DRenderer"
                                                              
                               "#define HAVE_ENTACTAPI"
                               "include" "EntactAPI"
                               "lib" "EntactAPI"
                               "bin" "EntactAPI"
                               
                               "#define HAVE_DHDAPI"
                               "include" "DHD-API"
                               "lib" "dhdms"

                               "#define HAVE_VIRTUOSEAPI"
                               "include" "virtuoseAPI"
                               "lib" "virtuoseDLL"

                               "#define HAVE_FALCONAPI"
                               "warning" "NOTE: HAPI compiled with Novint Falcon support. Test that application starts on system without Novint Falcon dlls before distributing package if you do not distribute it yourself."
                               
                               "#define HAVE_NIFALCONAPI"
                               "warning" "NOTE: HAPI compiled with NiFalcon api support. Test that application starts on system without NiFalcon dlls (if there are any) before distributing package if you do not distribute it yourself."
                               
                               "#define NIFALCONAPI_LIBUSB"
                               "warning" "NOTE: HAPI compiled with NiFalcon api LIBUSB support. Test that application starts on system without NiFalcon dlls (if there are any) before distributing package if you do not distribute it yourself."
                               
                               "#define NIFALCONAPI_LIBFTD2XX"
                               "warning" "NOTE: HAPI compiled with NiFalcon api LIBFTD2XX api support. Test that application starts on system without NiFalcon dlls (if there are any) before distributing package if you do not distribute it yourself."
                               
                               "#define NIFALCONAPI_LIBFTDI"
                               "warning" "NOTE: HAPI compiled with NiFalcon api LIBFTDI support. Test that application starts on system without NiFalcon dlls (if there are any) before distributing package if you do not distribute it yourself."
                               
                               "#define HAVE_FPARSER"
                               "include" "fparser"
                               "lib" "fparser"
                               "bin" "fparser"
                               
                               "#define HAVE_HAPTIK_LIBRARY"
                               "warning" "NOTE: HAPI compiled with HAPTIK library support. Test that application starts on system without HAPTIK dlls (if there are any) before distributing package if you do not distribute it yourself."

                               "#define HAVE_SIMBALLMEDICAL_API"
                               "include" "Simball"
                               "lib" "SimballMedicalHID"
                               "bin" "SimballMedicalHID"
                               
                               "#define HAVE_HAPTIC_MASTER_API"
                               "bin" "HapticAPI" "HapticMasterDriver"
                               
                               "#define HAVE_QUANSERAPI"
                               
                               "#define HAVE_MLHI"
                               
                               "#define HAVE_OPENGL"
                               "include" "GL/freeglut" "GL/freeglut" "GL/freeglut_ext" "GL/freeglut_std" "GL/glew" "GL/glext" "GL/glut" "GL/wglew"
                               "lib" "glew32" "freeglut"
                               "bin" "glew32" "freeglut"
                               
                               "#define"
                               "include" "HAPI"
                               "lib" "HAPI"
                               "bin" "HAPI" )
  ENDIF( MSVC )
  
  list( LENGTH FEATURES_TO_INSTALL FEATURES_TO_INSTALL_LENGTH )
  math( EXPR FEATURES_TO_INSTALL_LENGTH "${FEATURES_TO_INSTALL_LENGTH} - 1" )
  set( FEATURES_TO_INSTALL_TRUNCATED "")
  foreach( loop_var RANGE 0 ${FEATURES_TO_INSTALL_LENGTH} 2 )
    list( GET FEATURES_TO_INSTALL ${loop_var} one_feature )
    list( APPEND FEATURES_TO_INSTALL_TRUNCATED ${one_feature} )
  endforeach(loop_var)
  
  SET( INCLUDE_DIRS_TO_CHECK ${EXTERNAL_ROOT}/include ${HAPI_INCLUDE_DIR} )
  SET( ADD_EXTERNAL FALSE )
  SET( CURRENT_CHECKED_FEATURE "" )
  foreach( HAPI_INCLUDE_DIR_TMP ${HAPI_INCLUDE_DIR} )
    IF( EXISTS ${HAPI_INCLUDE_DIR_TMP}/HAPI/HAPI.h )
      foreach( feature_to_look_for ${externals_to_look_for} )
        IF( feature_to_look_for MATCHES "#define" )
          SET( ADD_EXTERNAL FALSE )
          SET( regex_to_find ${feature_to_look_for} )
          FILE( STRINGS ${HAPI_INCLUDE_DIR_TMP}/HAPI/HAPI.h list_of_defines REGEX ${regex_to_find} )
          LIST( LENGTH list_of_defines list_of_defines_length )
          IF( list_of_defines_length )
            SET( ADD_EXTERNAL TRUE )
          ENDIF( list_of_defines_length )
        ELSEIF( ADD_EXTERNAL )
          IF( feature_to_look_for STREQUAL "include" OR
              feature_to_look_for STREQUAL "lib" OR
              feature_to_look_for STREQUAL "bin" )
            LIST( FIND FEATURES_TO_INSTALL_TRUNCATED ${feature_to_look_for} feature_found )
            IF( ${feature_found} EQUAL -1 )
              SET( CURRENT_CHECKED_FEATURE "feature_not_desired" )
            ELSE( ${feature_found} EQUAL -1 )
              SET( CURRENT_CHECKED_FEATURE ${feature_to_look_for} )
            ENDIF( ${feature_found} EQUAL -1 )
          ELSEIF( feature_to_look_for STREQUAL "warning" )
            SET( CURRENT_CHECKED_FEATURE ${feature_to_look_for} )
          ELSEIF( CURRENT_CHECKED_FEATURE STREQUAL "include" )
            SET( FOUND_INCLUDE_PATH FALSE )
            foreach( include_dir_to_check ${INCLUDE_DIRS_TO_CHECK} )
              IF( EXISTS ${include_dir_to_check}/${feature_to_look_for} )
                SET( FOUND_INCLUDE_PATH TRUE )
                SET( HAPI_INCLUDE_DIRECTORIES_INSTALL ${HAPI_INCLUDE_DIRECTORIES_INSTALL} ${include_dir_to_check}/${feature_to_look_for} )
              ELSEIF( EXISTS ${include_dir_to_check}/${feature_to_look_for}.h )
                SET( FOUND_INCLUDE_PATH TRUE )
                SET( HAPI_INCLUDE_FILES_INSTALL ${HAPI_INCLUDE_FILES_INSTALL} ${include_dir_to_check}/${feature_to_look_for}.h )
              ENDIF( EXISTS ${include_dir_to_check}/${feature_to_look_for} )
            endforeach( include_dir_to_check )
            IF( NOT FOUND_INCLUDE_PATH )
              MESSAGE( "Include directory or file ${feature_to_look_for} not found. Searched using CMake variable EXTERNAL_ROOT and HAPI_INCLUDE_DIR." )
            ENDIF( NOT FOUND_INCLUDE_PATH )
          ELSEIF( CURRENT_CHECKED_FEATURE STREQUAL "lib" )
            IF( EXISTS ${HAPI_EXTERNAL_LIB}/${feature_to_look_for}.lib )
              SET( HAPI_LIBRARIES_INSTALL ${HAPI_LIBRARIES_INSTALL} ${HAPI_EXTERNAL_LIB}/${feature_to_look_for}.lib )
            ELSEIF( EXISTS ${HAPI_EXTERNAL_LIB}/static/${feature_to_look_for}.lib )
              SET( HAPI_LIBRARIES_INSTALL ${HAPI_LIBRARIES_INSTALL} ${HAPI_EXTERNAL_LIB}/static/${feature_to_look_for}.lib )
            ELSEIF( TARGET ${feature_to_look_for} )
              get_target_property( hapi_release_filename_path ${feature_to_look_for} LOCATION_RELEASE )
              get_filename_component( hapi_release_filename_path ${hapi_release_filename_path} PATH )
              SET( HAPI_LIBRARIES_INSTALL ${HAPI_LIBRARIES_INSTALL} ${hapi_release_filename_path}/${feature_to_look_for}_vc${H3D_MSVC_VERSION}.lib )
            ELSEIF( HAPI_CMAKE_INSTALL_PREFIX )
              set( dirs_to_test ${HAPI_CMAKE_INSTALL_PREFIX}/${HAPI_lib}
                                ${HAPI_CMAKE_INSTALL_PREFIX}/lib )
              IF( dirs_to_test )
                foreach( dir_to_test ${dirs_to_test} )
                  IF( EXISTS ${dir_to_test}/${feature_to_look_for}_vc${H3D_MSVC_VERSION}.lib )
                    SET( HAPI_LIBRARIES_INSTALL ${HAPI_LIBRARIES_INSTALL} ${dir_to_test}/${feature_to_look_for}_vc${H3D_MSVC_VERSION}.lib )
                    break()
                  ENDIF( EXISTS ${dir_to_test}/${feature_to_look_for}_vc${H3D_MSVC_VERSION}.lib )
                endforeach( dir_to_test )
              ENDIF( dirs_to_test )
            ELSE( EXISTS ${HAPI_EXTERNAL_LIB}/${feature_to_look_for}.lib )
              MESSAGE( "Library file ${feature_to_look_for}.lib not found. Searched using CMake variable EXTERNAL_ROOT and HAPI_CMAKE_INSTALL_PREFIX." )
            ENDIF( EXISTS ${HAPI_EXTERNAL_LIB}/${feature_to_look_for}.lib )
          ELSEIF( CURRENT_CHECKED_FEATURE STREQUAL "bin" )
            IF( EXISTS ${HAPI_EXTERNAL_BIN}/${feature_to_look_for}.dll )
              SET( HAPI_BINARIES_INSTALL ${HAPI_BINARIES_INSTALL} ${HAPI_EXTERNAL_BIN}/${feature_to_look_for}.dll )
            ELSEIF( TARGET ${feature_to_look_for} )
              get_target_property( hapi_release_filename ${feature_to_look_for} LOCATION_RELEASE )
              SET( HAPI_BINARIES_INSTALL ${HAPI_BINARIES_INSTALL} ${hapi_release_filename} )
            ELSEIF( HAPI_CMAKE_INSTALL_PREFIX )
              set( dirs_to_test ${HAPI_CMAKE_INSTALL_PREFIX}/${HAPI_bin}
                                ${HAPI_CMAKE_INSTALL_PREFIX}/bin )
              foreach( dir_to_test ${dirs_to_test} )
                IF( EXISTS ${dir_to_test}/${feature_to_look_for}_vc${H3D_MSVC_VERSION}.dll )
                  SET( HAPI_BINARIES_INSTALL ${HAPI_BINARIES_INSTALL} ${dir_to_test}/${feature_to_look_for}_vc${H3D_MSVC_VERSION}.dll )
                  break()
                ENDIF( EXISTS ${dir_to_test}/${feature_to_look_for}_vc${H3D_MSVC_VERSION}.dll )
              endforeach( dir_to_test )
            ELSE( EXISTS ${HAPI_EXTERNAL_BIN}/${feature_to_look_for}.dll )
              MESSAGE( "Binary file ${feature_to_look_for}.dll not found. Searched using CMake variable EXTERNAL_ROOT and HAPI_CMAKE_INSTALL_PREFIX." )
            ENDIF( EXISTS ${HAPI_EXTERNAL_BIN}/${feature_to_look_for}.dll )
          ELSEIF( CURRENT_CHECKED_FEATURE STREQUAL "warning" )
            MESSAGE( ${feature_to_look_for} )
          ENDIF() 
        ENDIF( feature_to_look_for MATCHES "#define" )
      endforeach( feature_to_look_for)
    ENDIF( EXISTS ${HAPI_INCLUDE_DIR_TMP}/HAPI/HAPI.h )
  endforeach( HAPI_INCLUDE_DIR_TMP )

  SET( INSTALL_INCLUDE FALSE )
  SET( INSTALL_LIB FALSE )
  SET( INSTALL_BIN FALSE )
  foreach( feature_to_install ${FEATURES_TO_INSTALL} )
    IF( NOT INSTALL_INCLUDE AND feature_to_install STREQUAL "include" )
      SET( INSTALL_INCLUDE TRUE )
    ELSEIF( NOT INSTALL_LIB AND feature_to_install STREQUAL "lib" )
      SET( INSTALL_LIB TRUE )
    ELSEIF( NOT INSTALL_BIN AND feature_to_install STREQUAL "bin" )
      SET( INSTALL_BIN TRUE )
    ELSEIF( INSTALL_INCLUDE )
      SET( INSTALL_INCLUDE FALSE )
      IF( HAPI_INCLUDE_DIRECTORIES_INSTALL )
        foreach( ext_dir ${HAPI_INCLUDE_DIRECTORIES_INSTALL} )
          SET( include_file_path_last_part "" )
          foreach( include_dir_to_check ${INCLUDE_DIRS_TO_CHECK} )
            get_filename_component( include_file_path ${ext_dir} PATH )
            string( LENGTH ${include_file_path} include_length )
            string( LENGTH ${include_dir_to_check} include_dir_to_check_length )
            IF( ${include_length} GREATER ${include_dir_to_check_length} )
              string( SUBSTRING ${include_file_path} 0 ${include_dir_to_check_length} include_file_path_first_part )
              IF( ${include_file_path_first_part} STREQUAL ${include_dir_to_check} )
                string( SUBSTRING ${include_file_path} ${include_dir_to_check_length} -1 include_file_path_last_part )
                break()
              ENDIF( ${include_file_path_first_part} STREQUAL ${include_dir_to_check} )
            ENDIF( ${include_length} GREATER ${include_dir_to_check_length} )
          endforeach( include_dir_to_check )
          INSTALL( DIRECTORY ${ext_dir}
                   DESTINATION ${feature_to_install}${include_file_path_last_part}
                   REGEX "(/.svn)|(/CVS)" EXCLUDE )
        endforeach( ext_dir )
      ENDIF( HAPI_INCLUDE_DIRECTORIES_INSTALL )

      IF( HAPI_INCLUDE_FILES_INSTALL )
        foreach( ext_include_file ${HAPI_INCLUDE_FILES_INSTALL} )
          SET( include_file_path_last_part "" )
          foreach( include_dir_to_check ${INCLUDE_DIRS_TO_CHECK} )
            get_filename_component( include_file_path ${ext_include_file} PATH )
            string( LENGTH ${include_file_path} include_length )
            string( LENGTH ${include_dir_to_check} include_dir_to_check_length )
            IF( ${include_length} GREATER ${include_dir_to_check_length} )
              string( SUBSTRING ${include_file_path} 0 ${include_dir_to_check_length} include_file_path_first_part )
              IF( ${include_file_path_first_part} STREQUAL ${include_dir_to_check} )
                string( SUBSTRING ${include_file_path} ${include_dir_to_check_length} -1 include_file_path_last_part )
                break()
              ENDIF( ${include_file_path_first_part} STREQUAL ${include_dir_to_check} )
            ENDIF( ${include_length} GREATER ${include_dir_to_check_length} )
          endforeach( include_dir_to_check )
          INSTALL( FILES ${ext_include_file}
                   DESTINATION ${feature_to_install}${include_file_path_last_part} )
        endforeach( ext_include_file )
      ENDIF( HAPI_INCLUDE_FILES_INSTALL )
    ELSEIF( INSTALL_LIB )
      SET( INSTALL_LIB FALSE )
      IF( HAPI_LIBRARIES_INSTALL )
        foreach( ext_lib ${HAPI_LIBRARIES_INSTALL} )
          INSTALL( FILES ${ext_lib}
                   DESTINATION ${feature_to_install} )
        endforeach( ext_lib )
      ENDIF( HAPI_LIBRARIES_INSTALL )
    ELSEIF( INSTALL_BIN )
      SET( INSTALL_BIN FALSE )
      IF( HAPI_BINARIES_INSTALL )
        foreach( ext_bin ${HAPI_BINARIES_INSTALL} )
          INSTALL( FILES ${ext_bin}
                   DESTINATION ${feature_to_install} )
        endforeach( ext_bin )
      ENDIF( HAPI_BINARIES_INSTALL )
    ENDIF( NOT INSTALL_INCLUDE AND feature_to_install STREQUAL "include" )
  endforeach( feature_to_install )
ENDIF( HAPI_INCLUDE_DIR AND EXTERNAL_ROOT)