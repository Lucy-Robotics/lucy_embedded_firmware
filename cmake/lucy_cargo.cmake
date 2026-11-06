function(lucy_cargo_firmware)
  cmake_parse_arguments(ARG "UF2" "BIN;TARGET" "" ${ARGN})
  if(NOT ARG_BIN)
    message(FATAL_ERROR "lucy_cargo_firmware: BIN is required")
  endif()

  find_program(CARGO_EXECUTABLE cargo REQUIRED)

  set(target_dir ${CMAKE_CURRENT_BINARY_DIR}/target)
  set(cargo_args build --release
    --manifest-path ${CMAKE_CURRENT_SOURCE_DIR}/Cargo.toml
    --target-dir ${target_dir})
  if(ARG_TARGET)
    list(APPEND cargo_args --target ${ARG_TARGET})
    set(binary ${target_dir}/${ARG_TARGET}/release/${ARG_BIN})
  else()
    set(binary ${target_dir}/release/${ARG_BIN})
  endif()

  set(commands COMMAND ${CARGO_EXECUTABLE} ${cargo_args})
  if(ARG_UF2)
    find_program(ELF2UF2_EXECUTABLE elf2uf2-rs REQUIRED)
    list(APPEND commands COMMAND ${ELF2UF2_EXECUTABLE} ${binary} ${binary}.uf2)
  endif()

  add_custom_target(cargo_build ALL
    ${commands}
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    USES_TERMINAL
    VERBATIM)

  if(ARG_UF2)
    install(FILES ${binary} ${binary}.uf2
      DESTINATION share/${PROJECT_NAME}/firmware)
  else()
    install(PROGRAMS ${binary}
      DESTINATION lib/${PROJECT_NAME})
  endif()
endfunction()
