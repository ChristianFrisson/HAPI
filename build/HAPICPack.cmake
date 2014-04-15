IF( NOT TARGET HAPI )
  MESSAGE( FATAL_ERROR "Include file HAPICPack.cmake require the target HAPI to exist. Please add HAPI/build/CMakeLists.txt as subdirectory first." )
ENDIF( NOT TARGET HAPI )

# Add all sources, they are added to a variable called HAPI_SRCS defined
# in the included file. All header files are added to a variable called
# HAPI_HEADERS.
INCLUDE( ${HAPI_SOURCE_DIR}/HAPISourceFiles.txt )

# If cpack should be configured.
IF( GENERATE_CPACK_PROJECT )
  IF( WIN32 )
    # Add a cache variable which indicates where the Externals directory used for packaging
    # HAPI is located. If not set then FIND modules will be used instead.
    IF( NOT DEFINED H3DAPI_CPACK_EXTERNAL_ROOT )
      IF( NOT DEFINED HAPI_CPACK_EXTERNAL_ROOT )
        SET( HAPI_CPACK_EXTERNAL_ROOT_DEFAULT "" )
        IF( H3D_USE_DEPENDENCIES_ONLY )
          foreach( EXTERNAL_INCLUDE_DIR_TMP ${EXTERNAL_INCLUDE_DIR} )
            IF( EXISTS ${EXTERNAL_INCLUDE_DIR_TMP}/../include/pthread )
              SET( HAPI_CPACK_EXTERNAL_ROOT_DEFAULT "${EXTERNAL_INCLUDE_DIR_TMP}/.." )
            ENDIF( EXISTS ${EXTERNAL_INCLUDE_DIR_TMP}/../include/pthread )
          endforeach( EXTERNAL_INCLUDE_DIR_TMP ${EXTERNAL_INCLUDE_DIR} )
        ELSE( H3D_USE_DEPENDENCIES_ONLY )
          SET( HAPI_CPACK_EXTERNAL_ROOT_DEFAULT "$ENV{H3D_EXTERNAL_ROOT}" )
        ENDIF( H3D_USE_DEPENDENCIES_ONLY )
        SET( HAPI_CPACK_EXTERNAL_ROOT "${HAPI_CPACK_EXTERNAL_ROOT_DEFAULT}" CACHE PATH "Set to the External directory used with HAPI, needed to pack properly. If not set FIND_modules will be used instead." )
        MARK_AS_ADVANCED(HAPI_CPACK_EXTERNAL_ROOT)
      ENDIF( NOT DEFINED HAPI_CPACK_EXTERNAL_ROOT )
    ELSE( NOT DEFINED H3DAPI_CPACK_EXTERNAL_ROOT )
      SET( HAPI_CPACK_EXTERNAL_ROOT ${H3DAPI_CPACK_EXTERNAL_ROOT} )
    ENDIF( NOT DEFINED H3DAPI_CPACK_EXTERNAL_ROOT )
  ENDIF( WIN32 )
  include( ${HAPI_SOURCE_DIR}/../../H3DUtil/build/H3DUtilCPack.cmake )
  
  IF( NOT TARGET H3DAPI )
    set( CPACK_ALL_INSTALL_TYPES Full Developer )
    SET(CMAKE_MODULE_PATH ${HAPI_SOURCE_DIR}/modules )
    SET(CPACK_PACKAGE_DESCRIPTION_SUMMARY "HAPI. A cross platform, device independent haptics library.")
    SET(CPACK_PACKAGE_VENDOR "SenseGraphics AB")
    SET(CPACK_PACKAGE_CONTACT "support@sensegraphics.com" )
    SET(CPACK_PACKAGE_DESCRIPTION_FILE "${HAPI_SOURCE_DIR}/../ReadMe")
    SET(CPACK_RESOURCE_FILE_LICENSE "${HAPI_SOURCE_DIR}/../LICENSE")
    SET(CPACK_INSTALL_CMAKE_PROJECTS "${CMAKE_CURRENT_BINARY_DIR};HAPI;ALL;/" )
    SET(CPACK_PACKAGE_INSTALL_DIRECTORY "HAPI" )
    
    # File patterns to ignore, common for all operating systems.
    SET( HAPI_CPACK_IGNORE_PATTERNS /\\\\.svn/
                                    \\\\.obj$
                                    \\\\.ncb$
                                    \\\\.log$
                                    \\\\.suo$
                                    \\\\.zip$
                                    \\\\.dir/
                                    \\\\.user$
                                    \\\\.cv$
                                    "/Debug(.)*/"
                                    "/debug(.)*/"
                                    /Release
                                    /release
                                    /linux
                                    /build/win32/
                                    "/(build|examples)/vc(7|(8|9))"
                                    /osx/ )
    SET(CPACK_PACKAGE_VERSION_MAJOR ${HAPI_MAJOR_VERSION})
    SET(CPACK_PACKAGE_VERSION_MINOR ${HAPI_MINOR_VERSION})
    SET(CPACK_PACKAGE_VERSION_PATCH ${HAPI_BUILD_VERSION})
    
    IF( WIN32 AND NOT UNIX )
      SET(CPACK_NSIS_INSTALL_ROOT "C:" )
      SET(CPACK_NSIS_ENABLE_UNINSTALL_BEFORE_INSTALL "ON" )  
      SET( CPACK_PACKAGE_START_MENU_NAME "HAPI 1.1" )
      
      # Extra links to start menu if values are "ON"
      SET( CPACK_ADD_HAPIDOC_LINKS "ON" )
      
      IF( HAPI_EXAMPLE_PROJECTS )
        SET( CPACK_ADD_HAPIEXAMPLES_LINKS "ON" )
      ENDIF( HAPI_EXAMPLE_PROJECTS )

      # External binary directory to add to path.
      SET( CPACK_EXTERNAL_BIN "bin32" )
      SET( CPACK_H3D_64_BIT "FALSE" )
      IF( CMAKE_SIZEOF_VOID_P EQUAL 8 )
        SET( CPACK_EXTERNAL_BIN "bin64" )
        SET( CPACK_H3D_64_BIT "TRUE" )
      ENDIF( CMAKE_SIZEOF_VOID_P EQUAL 8 )

      # Extra install commands will be set to install vc8(9)_redists
      SET( CPACK_NSIS_EXTRA_INSTALL_COMMANDS "\\n" )
      
      SET( redist_versions 8 9 10 )
      foreach( redist_version ${redist_versions} )
        # Add cache variable vc${redist_version}_redist which should be set to the install file
        # for microsoft visual studio redistributables, they can be found in the
        # installation folder for each visual studio installation.
        IF( NOT DEFINED vc${redist_version}_redist )
          SET( vc${redist_version}_redist CACHE FILEPATH "Set this to the exe installing microsoft visual studio redistributable for visual studio ${redist_version}" )
          MARK_AS_ADVANCED(vc${redist_version})
        ENDIF( NOT DEFINED vc${redist_version}_redist )
        IF( vc${redist_version}_redist )
          STRING( REPLACE "/" "\\\\" Temp_vc${redist_version}_redist ${vc${redist_version}_redist} )
          GET_FILENAME_COMPONENT( VC${redist_version}_FILE_NAME ${vc${redist_version}_redist} NAME )
          SET( MS_REDIST_INSTALL_COMMAND_1 " Set output Path\\n  SetOutPath \\\"$INSTDIR\\\\vc${redist_version}\\\"\\n"
                                           " Code to install Visual studio redistributable\\n  File \\\"${Temp_vc${redist_version}_redist}\\\"\\n" )
          SET( CPACK_NSIS_EXTRA_INSTALL_COMMANDS ${CPACK_NSIS_EXTRA_INSTALL_COMMANDS}
                                                 ${MS_REDIST_INSTALL_COMMAND_1} )
          SET( CPACK_NSIS_EXTRA_UNINSTALL_COMMANDS ${CPACK_NSIS_EXTRA_UNINSTALL_COMMANDS}
                                                   " Check if uninstall vc redist \\n  MessageBox MB_YESNO \\\"Do you want to uninstall Visual studio ${redist_version} redistributable? It is recommended if no other applications use it.\\\" IDYES uninstall_vcredist_yes IDNO uninstall_vcredist_no\\n"
                                                   " A comment \\n  uninstall_vcredist_yes:\\n"
                                                   ${MS_REDIST_INSTALL_COMMAND_1} )
          IF( ${redist_version} LESS 9 )
            SET( CPACK_NSIS_EXTRA_INSTALL_COMMANDS ${CPACK_NSIS_EXTRA_INSTALL_COMMANDS}
                                                   " Execute silent and wait\\n  ExecWait '\\\"$INSTDIR\\\\vc${redist_version}\\\\${VC${redist_version}_FILE_NAME}\\\" /q:a /norestart /c:\\\"msiexec /i vcredist.msi /qn\\\"'\\n" )
            SET( CPACK_NSIS_EXTRA_UNINSTALL_COMMANDS ${CPACK_NSIS_EXTRA_UNINSTALL_COMMANDS}
                                                   " Execute silent and wait\\n  ExecWait '\\\"$INSTDIR\\\\vc${redist_version}\\\\${VC${redist_version}_FILE_NAME}\\\" /q:a /norestart /c:\\\"msiexec /x vcredist.msi /qn\\\"'\\n" )
          ELSE( )
            SET( CPACK_NSIS_EXTRA_INSTALL_COMMANDS ${CPACK_NSIS_EXTRA_INSTALL_COMMANDS}
                                                   " Execute silent and wait\\n  ExecWait '\\\"$INSTDIR\\\\vc${redist_version}\\\\${VC${redist_version}_FILE_NAME}\\\" /q /norestart \\\"'" )
            SET( CPACK_NSIS_EXTRA_UNINSTALL_COMMANDS ${CPACK_NSIS_EXTRA_UNINSTALL_COMMANDS}
                                                   " Execute silent and wait\\n  ExecWait '\\\"$INSTDIR\\\\vc${redist_version}\\\\${VC${redist_version}_FILE_NAME}\\\" /q /uninstall \\\"'" )
          ENDIF( ${redist_version} LESS 9 )
          SET( MS_REDIST_INSTALL_COMMAND_2 " Wait a bit for system to unlock file.\\n  Sleep 1000\\n"
                                           " Delete file\\n  Delete \\\"$INSTDIR\\\\vc${redist_version}\\\\${VC${redist_version}_FILE_NAME}\\\"\\n"
                                           " Reset output Path\\n  SetOutPath \\\"$INSTDIR\\\"\\n"
                                           " Remove folder\\n  RMDir /r \\\"$INSTDIR\\\\vc${redist_version}\\\"\\n\\n" )
          SET( CPACK_NSIS_EXTRA_INSTALL_COMMANDS ${CPACK_NSIS_EXTRA_INSTALL_COMMANDS}
                                                 ${MS_REDIST_INSTALL_COMMAND_2} )
          SET( CPACK_NSIS_EXTRA_UNINSTALL_COMMANDS ${CPACK_NSIS_EXTRA_UNINSTALL_COMMANDS}
                                                   ${MS_REDIST_INSTALL_COMMAND_2}
                                                   " A comment \\n  uninstall_vcredist_no:\\n\\n" )
        ENDIF( vc${redist_version}_redist )
      endforeach( redist_version ${redist_versions} )
      
      # Modify path since in the NSIS template.
      SET( CPACK_NSIS_MODIFY_PATH "ON" )
    ENDIF( WIN32 AND NOT UNIX )
  ENDIF( NOT TARGET H3DAPI )
  
  IF( WIN32 )
    # EXTERNAL_INCLUDES and EXTERNAL_INCLUDE_INSTALL_PATHS must be of equal lengths.
    # The reason for defining these variables here is in case we want to add functionality
    # to configure installation in some other way (using FIND-modules for example).
    SET( EXTERNAL_INCLUDES "" )
    SET( EXTERNAL_INCLUDE_INSTALL_PATHS "" )
    # The EXTERNAL_INCLUDES_FILES are installed directly in External/include
    SET( EXTERNAL_INCLUDES_FILES "" )
    SET( EXTERNAL_LIBRARIES "" )
    SET( EXTERNAL_STATIC_LIBRARIES "" )
    SET( EXTERNAL_BINARIES "" )
    
    SET( EXTERNAL_BIN_PATH "bin32" )
    SET( EXTERNAL_BIN_REPLACE_PATH "bin64" )
    IF( CMAKE_SIZEOF_VOID_P EQUAL 8 )
      SET( EXTERNAL_BIN_PATH "bin64" )
      SET( EXTERNAL_BIN_REPLACE_PATH "bin32" )
    ENDIF( CMAKE_SIZEOF_VOID_P EQUAL 8 )
    
    IF( EXISTS ${HAPI_CPACK_EXTERNAL_ROOT} )
      SET( EXTERNAL_INCLUDES ${HAPI_CPACK_EXTERNAL_ROOT}/include/GL/
                             ${HAPI_CPACK_EXTERNAL_ROOT}/include/DHD-API/
                             ${HAPI_CPACK_EXTERNAL_ROOT}/include/chai3d/
                             ${HAPI_CPACK_EXTERNAL_ROOT}/include/Simball/
                             ${HAPI_CPACK_EXTERNAL_ROOT}/include/wx/
                             ${HAPI_CPACK_EXTERNAL_ROOT}/include/fparser/ )
      SET( EXTERNAL_INCLUDE_INSTALL_PATHS External/include/GL
                                          External/include/DHD-API
                                          External/include/chai3d
                                          External/include/Simball
                                          External/include/wx
                                          External/include/fparser )
      SET( EXTERNAL_INCLUDE_IGNORE_PATTERN "((/.svn)|(/CVS))|(/old)"
                                           "(/.svn)|(/CVS)"
                                           "(/.svn)|(/CVS)"
                                           "(/.svn)|(/CVS)"
                                           "(/.svn)|(/CVS)"
                                           "(/.svn)|(/CVS)" )

      SET( EXTERNAL_INCLUDES_FILES ${HAPI_CPACK_EXTERNAL_ROOT}/include/VirtuoseAPI.h
                                   ${HAPI_CPACK_EXTERNAL_ROOT}/include/EntactAPI.h )

      SET( EXTERNAL_LIBRARIES ${HAPI_CPACK_EXTERNAL_ROOT}/lib32/freeglut.lib
                              ${HAPI_CPACK_EXTERNAL_ROOT}/lib32/virtuoseDLL.lib
                              ${HAPI_CPACK_EXTERNAL_ROOT}/lib32/EntactAPI.lib
                              ${HAPI_CPACK_EXTERNAL_ROOT}/lib32/SimballMedicalHID.lib
                              ${HAPI_CPACK_EXTERNAL_ROOT}/lib32/dhdms64.lib
                              ${HAPI_CPACK_EXTERNAL_ROOT}/lib32/dhdms.lib
                              ${HAPI_CPACK_EXTERNAL_ROOT}/lib32/fparser.lib
                              ${HAPI_CPACK_EXTERNAL_ROOT}/lib32/fparser_d.lib )
      
      SET( wxlibs core adv aui html media propgrid ribbon stc webview xrc gl qa richtext )
      FOREACH( library_name ${wxlibs} )
        SET( EXTERNAL_LIBRARIES ${EXTERNAL_LIBRARIES}
                                ${HAPI_CPACK_EXTERNAL_ROOT}/lib32/wxmsw30u_${library_name}.lib
                                ${HAPI_CPACK_EXTERNAL_ROOT}/lib32/wxmsw30ud_${library_name}.lib )
        #SET( EXTERNAL_STATIC_LIBRARIES ${EXTERNAL_STATIC_LIBRARIES}
        #                               ${HAPI_CPACK_EXTERNAL_ROOT}/lib32/static/wxmsw30u_${library_name}.lib
        #                               ${HAPI_CPACK_EXTERNAL_ROOT}/lib32/static/wxmsw30ud_${library_name}.lib )
        SET( EXTERNAL_BINARIES ${EXTERNAL_BINARIES}
                               ${HAPI_CPACK_EXTERNAL_ROOT}/${EXTERNAL_BIN_PATH}/wxmsw30u_${library_name}_vc_custom.dll
                               ${HAPI_CPACK_EXTERNAL_ROOT}/${EXTERNAL_BIN_PATH}/wxmsw30u_${library_name}_vc_x64_custom.dll
                               ${HAPI_CPACK_EXTERNAL_ROOT}/${EXTERNAL_BIN_PATH}/wxmsw30ud_${library_name}_vc_custom.dll
                               ${HAPI_CPACK_EXTERNAL_ROOT}/${EXTERNAL_BIN_PATH}/wxmsw30ud_${library_name}_vc_x64_custom.dll
                               )
      ENDFOREACH( library_name )
      SET( wxlibs "" _net _xml )
      # IN LISTS means that the empty argument is parsed
      FOREACH( library_name IN LISTS wxlibs )
        SET( EXTERNAL_LIBRARIES ${EXTERNAL_LIBRARIES}
                                ${HAPI_CPACK_EXTERNAL_ROOT}/lib32/wxbase30u${library_name}.lib
                                ${HAPI_CPACK_EXTERNAL_ROOT}/lib32/wxbase30ud${library_name}.lib )
        SET( EXTERNAL_BINARIES ${EXTERNAL_BINARIES}
                               ${HAPI_CPACK_EXTERNAL_ROOT}/${EXTERNAL_BIN_PATH}/wxbase30u${library_name}_vc_custom.dll
                               ${HAPI_CPACK_EXTERNAL_ROOT}/${EXTERNAL_BIN_PATH}/wxbase30u${library_name}_vc_x64_custom.dll
                               ${HAPI_CPACK_EXTERNAL_ROOT}/${EXTERNAL_BIN_PATH}/wxbase30ud${library_name}_vc_custom.dll
                               ${HAPI_CPACK_EXTERNAL_ROOT}/${EXTERNAL_BIN_PATH}/wxbase30ud${library_name}_vc_x64_custom.dll
                               )
      ENDFOREACH( library_name )
      
      SET( wxlibs expat jpeg png regexu scintilla tiff zlib )
      FOREACH( library_name ${wxlibs} )
        SET( EXTERNAL_LIBRARIES ${EXTERNAL_LIBRARIES}
                                ${HAPI_CPACK_EXTERNAL_ROOT}/lib32/wx${library_name}.lib
                                ${HAPI_CPACK_EXTERNAL_ROOT}/lib32/wx${library_name}d.lib )
        SET( EXTERNAL_BINARIES ${EXTERNAL_BINARIES}
                               )
      ENDFOREACH( library_name )

      SET( EXTERNAL_STATIC_LIBRARIES ${EXTERNAL_STATIC_LIBRARIES}
                                     ${HAPI_CPACK_EXTERNAL_ROOT}/lib32/static/chai3d_complete_vc10.lib
                                     ${HAPI_CPACK_EXTERNAL_ROOT}/lib32/static/chai3d_complete_vc10_d.lib )
     
      SET( EXTERNAL_BINARIES ${EXTERNAL_BINARIES}
                             ${HAPI_CPACK_EXTERNAL_ROOT}/${EXTERNAL_BIN_PATH}/freeglut.dll
                             ${HAPI_CPACK_EXTERNAL_ROOT}/${EXTERNAL_BIN_PATH}/virtuoseDLL.dll
                             ${HAPI_CPACK_EXTERNAL_ROOT}/${EXTERNAL_BIN_PATH}/EntactAPI.dll
                             ${HAPI_CPACK_EXTERNAL_ROOT}/${EXTERNAL_BIN_PATH}/SimballMedicalHID.dll
                             ${HAPI_CPACK_EXTERNAL_ROOT}/${EXTERNAL_BIN_PATH}/HapticAPI.dll
                             ${HAPI_CPACK_EXTERNAL_ROOT}/${EXTERNAL_BIN_PATH}/HapticMasterDriver.dll
                             ${HAPI_CPACK_EXTERNAL_ROOT}/${EXTERNAL_BIN_PATH}/fparser.dll
                             ${HAPI_CPACK_EXTERNAL_ROOT}/${EXTERNAL_BIN_PATH}/fparser_d.dll )

    ELSE( EXISTS ${HAPI_CPACK_EXTERNAL_ROOT} )
      MESSAGE( WARNING "HAPI_CPACK_EXTERNAL_ROOT must be set to the External directory used by HAPI in order to package properly." )
    ENDIF( EXISTS ${HAPI_CPACK_EXTERNAL_ROOT} )
    
    IF( EXTERNAL_INCLUDES )
      list( LENGTH EXTERNAL_INCLUDES EXTERNAL_INCLUDES_LENGTH )
      math( EXPR EXTERNAL_INCLUDES_LENGTH "${EXTERNAL_INCLUDES_LENGTH} - 1" )
      foreach( val RANGE ${EXTERNAL_INCLUDES_LENGTH} )
        list(GET EXTERNAL_INCLUDES ${val} val1)
        list(GET EXTERNAL_INCLUDE_INSTALL_PATHS ${val} val2)
        list(GET EXTERNAL_INCLUDE_IGNORE_PATTERN ${val} val3)
        INSTALL( DIRECTORY ${val1}
                 DESTINATION ${val2}
                 COMPONENT HAPI_cpack_external_source
                 REGEX ${val3} EXCLUDE )
      endforeach( val )
    ENDIF( EXTERNAL_INCLUDES )
    
    foreach( include_file ${EXTERNAL_INCLUDES_FILES} )
      IF( EXISTS ${include_file} )
        INSTALL( FILES ${include_file}
                 DESTINATION External/include
                 COMPONENT HAPI_cpack_external_source )
      ENDIF( EXISTS ${include_file} )
    endforeach( include_file )
    
    foreach( library ${EXTERNAL_LIBRARIES} )
      IF( EXISTS ${library} )
        INSTALL( FILES ${library}
                 DESTINATION External/lib32
                 COMPONENT HAPI_cpack_external_source )
      ENDIF( EXISTS ${library} )
      # Add the other library path as well
      STRING( REGEX REPLACE "(/lib32/)" "/lib64/" other_library ${library} )
      IF( EXISTS ${other_library} )
        INSTALL( FILES ${other_library}
                 DESTINATION External/lib64
                 COMPONENT HAPI_cpack_external_source )
      ENDIF( EXISTS ${other_library} )
    endforeach( library )
    
    foreach( library ${EXTERNAL_STATIC_LIBRARIES} )
      IF( EXISTS ${library} )
        INSTALL( FILES ${library}
                 DESTINATION External/lib32/static
                 COMPONENT HAPI_cpack_external_source )
      ENDIF( EXISTS ${library} )
      # Add the other library path as well
      STRING( REGEX REPLACE "(/lib32/)" "/lib64/" other_library ${library} )
      IF( EXISTS ${other_library} )
        INSTALL( FILES ${other_library}
                 DESTINATION External/lib64/static
                 COMPONENT HAPI_cpack_external_source )
      ENDIF( EXISTS ${other_library} )
    endforeach( library )
    
    foreach( binary ${EXTERNAL_BINARIES} )
      IF( EXISTS ${binary} )
        INSTALL( FILES ${binary}
                 DESTINATION External/${EXTERNAL_BIN_PATH}
                 COMPONENT HAPI_cpack_external_runtime )
      ENDIF( EXISTS ${binary} )

      STRING( REGEX REPLACE "(/${EXTERNAL_BIN_PATH}/)" "/${EXTERNAL_BIN_REPLACE_PATH}/" other_binary ${binary} )
      IF( EXISTS ${other_binary} )
        INSTALL( FILES ${other_binary}
                 DESTINATION External/${EXTERNAL_BIN_REPLACE_PATH}
                 COMPONENT HAPI_cpack_external_runtime )
      ENDIF( EXISTS ${other_binary} )
    endforeach( binary )    
    
    # setting names and dependencies between components and also grouping them.
    set(CPACK_COMPONENT_HAPI_CPACK_EXTERNAL_RUNTIME_DISPLAY_NAME "External runtime")
    set(CPACK_COMPONENT_HAPI_CPACK_EXTERNAL_RUNTIME_DESCRIPTION "External runtime binaries needed by HAPI.")
    set(CPACK_COMPONENT_HAPI_CPACK_EXTERNAL_RUNTIME_DEPENDS H3DUtil_cpack_external_runtime )
    set(CPACK_COMPONENT_HAPI_CPACK_EXTERNAL_RUNTIME_GROUP "HAPI_cpack_group")
    set(CPACK_COMPONENT_HAPI_CPACK_EXTERNAL_RUNTIME_INSTALL_TYPES Developer Full)
    
    set(CPACK_COMPONENT_HAPI_CPACK_EXTERNAL_SOURCE_DISPLAY_NAME "External header/libraries")
    set(CPACK_COMPONENT_HAPI_CPACK_EXTERNAL_SOURCE_DESCRIPTION "External headers and libraries needed by HAPI.")
    set(CPACK_COMPONENT_HAPI_CPACK_EXTERNAL_SOURCE_DEPENDS H3DUtil_cpack_external_source HAPI_cpack_external_runtime )
    set(CPACK_COMPONENT_HAPI_CPACK_EXTERNAL_SOURCE_GROUP "HAPI_cpack_group")
    set(CPACK_COMPONENT_HAPI_CPACK_EXTERNAL_SOURCE_INSTALL_TYPES Developer Full)
  ENDIF( WIN32 )

  # Our project depends on these debian packages for Linux.
  SET(DEBIAN_PACKAGE_DEPENDS "libgl-dev, h3dutil(>=1.0.0)" )
  
  # Install header files
  INSTALL( FILES ${HAPI_HEADERS} 
           DESTINATION HAPI/include/HAPI
           COMPONENT HAPI_cpack_headers )

  # HAPI.cmake that goes to headers is not needed unless sources is required.
  INSTALL( FILES ${HAPI_SOURCE_DIR}/../include/HAPI/HAPI.cmake
      DESTINATION HAPI/include/HAPI
      COMPONENT HAPI_cpack_sources )

  INSTALL( FILES ${OH_HEADERS}
           DESTINATION HAPI/OpenHapticsRenderer/include/HAPI
           COMPONENT HAPI_cpack_headers )

  INSTALL( FILES ${CHAI_HEADERS}
           DESTINATION HAPI/Chai3DRenderer/include/HAPI
           COMPONENT HAPI_cpack_headers )
  
  # Install src files.
  INSTALL( FILES ${HAPI_SRCS}
           DESTINATION HAPI/src
           COMPONENT HAPI_cpack_sources )

  INSTALL( FILES ${OH_SRCS}
           DESTINATION HAPI/OpenHapticsRenderer/src
           COMPONENT HAPI_cpack_sources )

  INSTALL( FILES ${CHAI_SRCS}
           DESTINATION HAPI/Chai3DRenderer/src
           COMPONENT HAPI_cpack_sources )

  INSTALL( FILES ${HAPI_SOURCE_DIR}/../changelog
                 ${HAPI_SOURCE_DIR}/../LICENSE
                 ${HAPI_SOURCE_DIR}/../ReadMe
           DESTINATION HAPI
           COMPONENT HAPI_cpack_sources )

  INSTALL( FILES ${HAPI_SOURCE_DIR}/Chai3DRenderer.rc.cmake
                 ${HAPI_SOURCE_DIR}/CMakeLists.txt
                 ${HAPI_SOURCE_DIR}/HAPI.rc.cmake
                 ${HAPI_SOURCE_DIR}/HAPICPack.cmake
                 ${HAPI_SOURCE_DIR}/HAPISourceFiles.txt
                 ${HAPI_SOURCE_DIR}/OpenHapticsRenderer.rc.cmake
                 ${HAPI_SOURCE_DIR}/UnityBuild.cmake
                 ${HAPI_SOURCE_DIR}/UpdateResourceFile.exe
           DESTINATION HAPI/build
           COMPONENT HAPI_cpack_sources )

  INSTALL( FILES ${HAPI_SOURCE_DIR}/modules/FindChai3D.cmake
                 ${HAPI_SOURCE_DIR}/modules/FindDHD.cmake
                 ${HAPI_SOURCE_DIR}/modules/FINDDirectX.cmake
                 ${HAPI_SOURCE_DIR}/modules/FindEntactAPI.cmake
                 ${HAPI_SOURCE_DIR}/modules/FindFalconAPI.cmake
                 ${HAPI_SOURCE_DIR}/modules/FindGLUTWin.cmake
                 ${HAPI_SOURCE_DIR}/modules/FindH3Dfparser.cmake
                 ${HAPI_SOURCE_DIR}/modules/FindH3DUtil.cmake
                 ${HAPI_SOURCE_DIR}/modules/FindHAPI.cmake
                 ${HAPI_SOURCE_DIR}/modules/FindHaptik.cmake
                 ${HAPI_SOURCE_DIR}/modules/FindMd5sum.cmake
                 ${HAPI_SOURCE_DIR}/modules/FindMLHI.cmake
                 ${HAPI_SOURCE_DIR}/modules/FindNiFalconAPI.cmake
                 ${HAPI_SOURCE_DIR}/modules/FindOpenHaptics.cmake
                 ${HAPI_SOURCE_DIR}/modules/FindPTHREAD.cmake
                 ${HAPI_SOURCE_DIR}/modules/FindSimballMedical.cmake
                 ${HAPI_SOURCE_DIR}/modules/FindVirtuoseAPI.cmake
                 ${HAPI_SOURCE_DIR}/modules/FindWxWidgetsWin.cmake
                 ${HAPI_SOURCE_DIR}/modules/InstallHAPIAndExternals.cmake
                 ${HAPI_SOURCE_DIR}/modules/NSIS.InstallOptions.ini.in
                 ${HAPI_SOURCE_DIR}/modules/NSIS.template.in
                 ${HAPI_SOURCE_DIR}/modules/StripAndAddLibraryDirectories.cmake
                 ${HAPI_SOURCE_DIR}/modules/TestIfVCExpress.cmake
                 ${HAPI_SOURCE_DIR}/modules/UseDebian.cmake
           DESTINATION HAPI/build/modules
           COMPONENT HAPI_cpack_sources )

  INSTALL( FILES ${HAPI_SOURCE_DIR}/../examples/build/CMakeLists.txt
           DESTINATION HAPI/examples/build
           COMPONENT HAPI_cpack_sources )

  INSTALL( FILES ${HAPI_SOURCE_DIR}/../examples/DirectXExample/build/CMakeLists.txt
           DESTINATION HAPI/examples/DirectXExample/build
           COMPONENT HAPI_cpack_sources )

  INSTALL( FILES ${HAPI_SOURCE_DIR}/../examples/DirectXExample/src/main.cpp
           DESTINATION HAPI/examples/DirectXExample/src
           COMPONENT HAPI_cpack_sources )

  INSTALL( FILES ${HAPI_SOURCE_DIR}/../examples/FeedbackBufferCollectorExample/build/CMakeLists.txt
           DESTINATION HAPI/examples/FeedbackBufferCollectorExample/build
           COMPONENT HAPI_cpack_sources )

  INSTALL( FILES ${HAPI_SOURCE_DIR}/../examples/FeedbackBufferCollectorExample/FeedbackBufferCollectorExample.cpp
           DESTINATION HAPI/examples/FeedbackBufferCollectorExample
           COMPONENT HAPI_cpack_sources )

  INSTALL( FILES ${HAPI_SOURCE_DIR}/../examples/HAPIDemo/build/CMakeLists.txt
           DESTINATION HAPI/examples/HAPIDemo/build
           COMPONENT HAPI_cpack_sources )

  INSTALL( FILES ${HAPI_SOURCE_DIR}/../examples/HAPIDemo/ForceFieldWidgetsPage.cpp
                 ${HAPI_SOURCE_DIR}/../examples/HAPIDemo/ForceFieldWidgetsPage.h
                 ${HAPI_SOURCE_DIR}/../examples/HAPIDemo/HAPIDemo.cpp
                 ${HAPI_SOURCE_DIR}/../examples/HAPIDemo/HAPIDemo.h
                 ${HAPI_SOURCE_DIR}/../examples/HAPIDemo/HapticShapeConstraintWidgetsPage.cpp
                 ${HAPI_SOURCE_DIR}/../examples/HAPIDemo/HapticShapeConstraintWidgetsPage.h
                 ${HAPI_SOURCE_DIR}/../examples/HAPIDemo/PositionFunctionWidgetsPage.cpp
                 ${HAPI_SOURCE_DIR}/../examples/HAPIDemo/PositionFunctionWidgetsPage.h
                 ${HAPI_SOURCE_DIR}/../examples/HAPIDemo/SpringWidgetsPage.cpp
                 ${HAPI_SOURCE_DIR}/../examples/HAPIDemo/SpringWidgetsPage.h
                 ${HAPI_SOURCE_DIR}/../examples/HAPIDemo/TimeFunctionWidgetsPage.cpp
                 ${HAPI_SOURCE_DIR}/../examples/HAPIDemo/TimeFunctionWidgetsPage.h
                 ${HAPI_SOURCE_DIR}/../examples/HAPIDemo/ViscosityWidgetsPage.cpp
                 ${HAPI_SOURCE_DIR}/../examples/HAPIDemo/ViscosityWidgetsPage.h
           DESTINATION HAPI/examples/HAPIDemo
           COMPONENT HAPI_cpack_sources )

  INSTALL( FILES ${HAPI_SOURCE_DIR}/../examples/SpringExample/build/CMakeLists.txt
           DESTINATION HAPI/examples/SpringExample/build
           COMPONENT HAPI_cpack_sources )

  INSTALL( FILES ${HAPI_SOURCE_DIR}/../examples/SpringExample/SpringExample.cpp
           DESTINATION HAPI/examples/SpringExample
           COMPONENT HAPI_cpack_sources )

  INSTALL( FILES ${HAPI_SOURCE_DIR}/../examples/SurfaceExample/build/CMakeLists.txt
           DESTINATION HAPI/examples/SurfaceExample/build
           COMPONENT HAPI_cpack_sources )

  INSTALL( FILES ${HAPI_SOURCE_DIR}/../examples/SurfaceExample/SurfaceDemo.cpp
           DESTINATION HAPI/examples/SurfaceExample
           COMPONENT HAPI_cpack_sources )

  INSTALL( FILES ${HAPI_SOURCE_DIR}/../examples/ThreadExamples/build/CMakeLists.txt
           DESTINATION HAPI/examples/ThreadExamples/build
           COMPONENT HAPI_cpack_sources )

  INSTALL( FILES ${HAPI_SOURCE_DIR}/../examples/ThreadExamples/PeriodicThreadCallbacks/PeriodicThreadCallbacks.cpp
           DESTINATION HAPI/examples/ThreadExamples/PeriodicThreadCallbacks
           COMPONENT HAPI_cpack_sources )

  INSTALL( FILES ${HAPI_SOURCE_DIR}/../examples/ThreadExamples/SimpleThreadPrint/SimpleThreadPrint.cpp
           DESTINATION HAPI/examples/ThreadExamples/SimpleThreadPrint
           COMPONENT HAPI_cpack_sources )

  INSTALL( FILES ${HAPI_SOURCE_DIR}/../examples/ThreadExamples/SimpleThreadPrintLock/SimpleThreadPrintLock.cpp
           DESTINATION HAPI/examples/ThreadExamples/SimpleThreadPrintLock
           COMPONENT HAPI_cpack_sources )

  # Add a cache variable HAPI_DOCS_DIRECTORY used to indicate where the HAPI docs are.
  IF( NOT DEFINED HAPI_DOCS_DIRECTORY )
    SET( HAPI_DOCS_DIRECTORY_DEFAULT "" )
    IF( H3D_USE_DEPENDENCIES_ONLY )
      SET( HAPI_DOCS_DIRECTORY_DEFAULT "${HAPI_SOURCE_DIR}/../../doc" )
    ELSEIF( TARGET H3DAPI )
      SET( HAPI_DOCS_DIRECTORY_DEFAULT "${H3DAPI_DOCS_DIRECTORY}" )
    ENDIF( H3D_USE_DEPENDENCIES_ONLY )
    SET( HAPI_DOCS_DIRECTORY "${HAPI_DOCS_DIRECTORY_DEFAULT}" CACHE PATH "Set this to the directory containing the documentation of HAPI." )
    MARK_AS_ADVANCED(HAPI_DOCS_DIRECTORY)
  ENDIF( NOT DEFINED HAPI_DOCS_DIRECTORY )
  
  IF( EXISTS ${HAPI_DOCS_DIRECTORY} )
    INSTALL( DIRECTORY ${HAPI_DOCS_DIRECTORY}/HAPI
             DESTINATION doc
             COMPONENT HAPI_cpack_headers
             REGEX "(/.svn)|(/CVS)" EXCLUDE )
    INSTALL( FILES "${HAPI_DOCS_DIRECTORY}/HAPI Manual.pdf"
             DESTINATION doc
             COMPONENT HAPI_cpack_headers )
  ENDIF( EXISTS ${HAPI_DOCS_DIRECTORY} )

  # setting names and dependencies between components and also grouping them.
  set(CPACK_COMPONENT_HAPI_CPACK_RUNTIME_DISPLAY_NAME "Runtime")
  set(CPACK_COMPONENT_HAPI_CPACK_RUNTIME_DESCRIPTION "The runtime libraries (dlls) for HAPI.")
  set(CPACK_COMPONENT_HAPI_CPACK_RUNTIME_DEPENDS H3DUtil_cpack_runtime HAPI_cpack_external_runtime )
  set(CPACK_COMPONENT_HAPI_CPACK_RUNTIME_GROUP "HAPI_cpack_group")
  set(CPACK_COMPONENT_HAPI_CPACK_RUNTIME_INSTALL_TYPES Developer Full)
    
  set(CPACK_COMPONENT_HAPI_CPACK_LIBRARIES_DISPLAY_NAME "Libraries")
  set(CPACK_COMPONENT_HAPI_CPACK_LIBRARIES_DESCRIPTION "HAPI libraries, needed for building against HAPI.")
  set(CPACK_COMPONENT_HAPI_CPACK_LIBRARIES_DEPENDS H3DUtil_cpack_libraries HAPI_cpack_external_source HAPI_cpack_headers )
  set(CPACK_COMPONENT_HAPI_CPACK_LIBRARIES_GROUP "HAPI_cpack_group")
  set(CPACK_COMPONENT_HAPI_CPACK_LIBRARIES_INSTALL_TYPES Developer Full)
    
  set(CPACK_COMPONENT_HAPI_CPACK_HEADERS_DISPLAY_NAME "C++ Headers")
  set(CPACK_COMPONENT_HAPI_CPACK_HEADERS_DESCRIPTION "HAPI C++ headers, needed for building against HAPI.")
  set(CPACK_COMPONENT_HAPI_CPACK_HEADERS_DEPENDS H3DUtil_cpack_headers HAPI_cpack_external_source HAPI_cpack_libraries )
  set(CPACK_COMPONENT_HAPI_CPACK_HEADERS_GROUP "HAPI_cpack_group")
  set(CPACK_COMPONENT_HAPI_CPACK_HEADERS_INSTALL_TYPES Developer Full)
    
  set(CPACK_COMPONENT_HAPI_CPACK_SOURCES_DISPLAY_NAME "C++ Source")
  set(CPACK_COMPONENT_HAPI_CPACK_SOURCES_DESCRIPTION "Everything needed to build HAPI.")
  set(CPACK_COMPONENT_HAPI_CPACK_SOURCES_DEPENDS HAPI_cpack_headers H3DUtil_cpack_sources )
  set(CPACK_COMPONENT_HAPI_CPACK_SOURCES_GROUP "HAPI_cpack_group")
  set(CPACK_COMPONENT_HAPI_CPACK_SOURCES_INSTALL_TYPES Full)
  
  set(CPACK_COMPONENT_HAPI_CPACK_EXAMPLES_RUNTIME_DISPLAY_NAME "Example applications")
  set(CPACK_COMPONENT_HAPI_CPACK_EXAMPLES_RUNTIME_DESCRIPTION "The example applications for HAPI.")
  set(CPACK_COMPONENT_HAPI_CPACK_EXAMPLES_RUNTIME_DEPENDS HAPI_cpack_runtime )
  set(CPACK_COMPONENT_HAPI_CPACK_EXAMPLES_RUNTIME_GROUP "HAPI_cpack_group")
  set(CPACK_COMPONENT_HAPI_CPACK_EXAMPLES_RUNTIME_INSTALL_TYPES Developer Full)
  
  set(CPACK_COMPONENT_GROUP_HAPI_CPACK_GROUP_DISPLAY_NAME "HAPI")
  set(CPACK_COMPONENT_GROUP_HAPI_CPACK_GROUP_DESCRIPTION "An open source cross platform haptics rendering engine with support for several haptics devices. C++ interface only.")
  set(CPACK_COMPONENT_GROUP_H3DUTIL_CPACK_GROUP_PARENT_GROUP "HAPI_cpack_group")  
  
  # Add a cache variable H3D_cmake_runtime_path to point to cmake binary.
  SET (H3D_cmake_runtime_path_default "")
  IF( NOT DEFINED H3D_cmake_runtime_path )
    IF( WIN32 AND NOT UNIX )
      SET (VERSION_CMAKES "4.0" "3.9" "3.8" "3.7" "3.6" "3.5" "3.4" "3.3" "3.2" "3.1" "3.0" "2.9" "2.8" "2.7" "2.6")
      foreach (version_cmake ${VERSION_CMAKES} )
        IF (EXISTS "C:/Program Files/CMake ${version_cmake}/bin/cmake.exe")
          SET( H3D_cmake_runtime_path_default "C:/Program Files/CMake ${version_cmake}/bin/cmake.exe" )
          break()
        ENDIF (EXISTS "C:/Program Files/CMake ${version_cmake}/bin/cmake.exe")
        
        IF (EXISTS "C:/Program Files (x86)/CMake ${version_cmake}/bin/cmake.exe")
          SET( H3D_cmake_runtime_path_default "C:/Program Files (x86)/CMake ${version_cmake}/bin/cmake.exe" )
          break()
        ENDIF (EXISTS "C:/Program Files (x86)/CMake ${version_cmake}/bin/cmake.exe")
        
        IF ( EXISTS "C:/Program/CMake ${version_cmake}/bin/cmake.exe")
          SET( H3D_cmake_runtime_path_default "C:/Program/CMake ${version_cmake}/bin/cmake.exe" )
          break()
        ENDIF ( EXISTS "C:/Program/CMake ${version_cmake}/bin/cmake.exe")
      endforeach (version_cmake )
    ELSE( WIN32 AND NOT UNIX )
      SET( H3D_cmake_runtime_path_default "cmake" )
    ENDIF( WIN32 AND NOT UNIX )
    SET( H3D_cmake_runtime_path ${H3D_cmake_runtime_path_default} CACHE FILEPATH "The path to the cmake runtime." )
    MARK_AS_ADVANCED(H3D_cmake_runtime_path)
  ENDIF( NOT DEFINED H3D_cmake_runtime_path )

  IF(UNIX)
    SET(CPACK_SOURCE_INSTALLED_DIRECTORIES "${HAPI_SOURCE_DIR}/..;/" )  
    SET(CPACK_SOURCE_GENERATOR TGZ ZIP ) 
    SET(CPACK_SOURCE_PACKAGE_FILE_NAME "hapi-${HAPI_MAJOR_VERSION}.${HAPI_MINOR_VERSION}.${HAPI_BUILD_VERSION}") 


    SET( HAPI_CPACK_IGNORE_PATTERNS ${HAPI_CPACK_IGNORE_PATTERNS}
            "/CVS/;/.svn/;/.bzr/;/.hg/;/.git.*/;.swp$;.#;/#;~$")
    SET(CPACK_SOURCE_IGNORE_FILES ${HAPI_CPACK_IGNORE_PATTERNS} )
  ENDIF( UNIX )
  
  IF( H3D_cmake_runtime_path )
    SET( INSTALL_RUNTIME_AND_LIBRARIES_ONLY_POST_BUILD ${INSTALL_RUNTIME_AND_LIBRARIES_ONLY_POST_BUILD}
                                                       COMMAND ${H3D_cmake_runtime_path} 
                                                       ARGS -DBUILD_TYPE=$(Configuration) -DCOMPONENT=HAPI_cpack_runtime -P cmake_install.cmake 
                                                       COMMAND ${H3D_cmake_runtime_path} 
                                                       ARGS -DBUILD_TYPE=$(Configuration) -DCOMPONENT=HAPI_cpack_libraries -P cmake_install.cmake
                                                       COMMAND ${H3D_cmake_runtime_path} 
                                                       ARGS -DBUILD_TYPE=$(Configuration) -DCOMPONENT=HAPI_cpack_examples_runtime -P cmake_install.cmake )
    
    IF( NOT TARGET H3DAPI )
      
      ADD_CUSTOM_COMMAND( OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/DummyFile
                          COMMAND echo )
      ADD_CUSTOM_TARGET( INSTALL_RUNTIME_AND_LIBRARIES_ONLY
                         DEPENDS ${CMAKE_CURRENT_BINARY_DIR}/DummyFile )
      
      ADD_CUSTOM_COMMAND( TARGET INSTALL_RUNTIME_AND_LIBRARIES_ONLY
                          POST_BUILD
                          ${INSTALL_RUNTIME_AND_LIBRARIES_ONLY_POST_BUILD} )
      ADD_DEPENDENCIES( INSTALL_RUNTIME_AND_LIBRARIES_ONLY HAPI ${INSTALL_RUNTIME_AND_LIBRARIES_ONLY_DEPENDENCIES} )
    ENDIF( NOT TARGET H3DAPI )
  ELSE( H3D_cmake_runtime_path )
    MESSAGE (STATUS "H3D_cmake_runtime_path is not set, please set it to continue")
  ENDIF( H3D_cmake_runtime_path )
  
  IF( NOT H3D_USE_DEPENDENCIES_ONLY )
    IF (NOT TARGET H3DAPI)
      INCLUDE(CPack)
      IF(${CMAKE_SYSTEM_NAME} MATCHES "Linux")  
        INCLUDE(UseDebian)
        IF(DEBIAN_FOUND)
          ADD_DEBIAN_TARGETS(HAPI)
        ENDIF(DEBIAN_FOUND)
      ENDIF(${CMAKE_SYSTEM_NAME} MATCHES "Linux")  
    ENDIF (NOT TARGET H3DAPI)
  ENDIF( NOT H3D_USE_DEPENDENCIES_ONLY )
ENDIF( GENERATE_CPACK_PROJECT )
