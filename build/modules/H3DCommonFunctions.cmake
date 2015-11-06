# Contains common H3D functions that are used a bit here and there.

SET( H3D_MSVC_VERSION 6 )
if( MSVC )
  SET( TEMP_MSVC_VERSION 1299 )
  WHILE( ${MSVC_VERSION} GREATER ${TEMP_MSVC_VERSION} )
    MATH( EXPR H3D_MSVC_VERSION "${H3D_MSVC_VERSION} + 1" )
    MATH( EXPR TEMP_MSVC_VERSION "${TEMP_MSVC_VERSION} + 100" )
  ENDWHILE( ${MSVC_VERSION} GREATER ${TEMP_MSVC_VERSION} )

  if( ${H3D_MSVC_VERSION} GREATER 12 ) # MSVC skipped 13 in their numbering system.
    MATH( EXPR H3D_MSVC_VERSION "${H3D_MSVC_VERSION} + 1" )
  endif()
endif()

# the_target Will contain search path for the include directories.
# target_base_name Must be the base name for the target.
# Optional argument can be given which will be set to the postfix appended to the name.
function( setH3DMSVCOutputName the_target target_base_name )
  if( MSVC )
    # change the name depending on compiler to be able to tell them apart
    # since they are not compatible with each other. 
    # the_target can not link incrementally on vc8 for some reason. We shut of incremental linking for
    # all visual studio versions.
    SET_TARGET_PROPERTIES( ${the_target} PROPERTIES OUTPUT_NAME ${target_base_name}_vc${H3D_MSVC_VERSION} )
    if( ARGC GREATER 2 )
      set( ${ARGV2} _vc${H3D_MSVC_VERSION} PARENT_SCOPE )
    endif()
  endif()
endfunction()

# compile_flags_container Compile flags will be added here.
function( addCommonH3DMSVCCompileFlags compile_flags_container )
  if( MSVC )
    # Treat wchar_t as built in type for all visual studio versions.
    # This is default for every version above 7 ( so far ) but we still set it for all.
    SET( compile_flags_container_internal "/Zc:wchar_t")

    IF( MSVC80 )
      # This might be useful for visual studio 2005 users that often recompile the api.
      IF( NOT DEFINED USE_VC8_MP_FLAG )
        SET( USE_VC8_MP_FLAG "NO" CACHE BOOL "In visual studio 8 the MP flag exists but is not documented. Maybe it is unsafe to use. If you want to use it then set this flag to yes." )
      ENDIF( NOT DEFINED USE_VC8_MP_FLAG )

      IF( USE_VC8_MP_FLAG )
        SET( compile_flags_container_internal "${compile_flags_container_internal} /MP" )
      ENDIF( USE_VC8_MP_FLAG )
    ENDIF( MSVC80 )

    IF( ${MSVC_VERSION} GREATER 1399 )
      # Remove compiler warnings about deprecation for visual studio versions 8 and above.
      SET( compile_flags_container_internal "${compile_flags_container_internal} -D_CRT_SECURE_NO_DEPRECATE" )
    ENDIF( ${MSVC_VERSION} GREATER 1399 )

    IF( ${MSVC_VERSION} GREATER 1499 )
      # Build using several threads for visual studio versions 9 and above.
      SET( compile_flags_container_internal "${compile_flags_container_internal} /MP" )
    ENDIF( ${MSVC_VERSION} GREATER 1499 )
    SET( ${compile_flags_container} "${${compile_flags_container}} ${compile_flags_container_internal}" PARENT_SCOPE)
  endif()
endfunction()
