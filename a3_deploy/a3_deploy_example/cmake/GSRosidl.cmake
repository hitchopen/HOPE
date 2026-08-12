function(gs_disable_rosidl_generator_py_if_requested)
  if(NOT GS_SKIP_ROSIDL_GENERATOR_PY)
    return()
  endif()

  if(NOT DEFINED AMENT_EXTENSIONS_rosidl_generate_idl_interfaces)
    return()
  endif()

  set(_gs_rosidl_extensions "${AMENT_EXTENSIONS_rosidl_generate_idl_interfaces}")
  list(FILTER _gs_rosidl_extensions EXCLUDE REGEX "^rosidl_generator_py:")
  set(AMENT_EXTENSIONS_rosidl_generate_idl_interfaces
      "${_gs_rosidl_extensions}"
      PARENT_SCOPE)
endfunction()
