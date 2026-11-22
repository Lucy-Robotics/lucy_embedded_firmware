# Build a firmware crate with cargo from colcon (ament_cmake).
#
# cargo comes from the Pixi env (`pixi run firmware-setup` installs rustup under
# $CONDA_PREFIX/cargo and $CONDA_PREFIX/rustup, the layout lucy_ws's
# scripts/firmware_cargo_env.py defines) or from CARGO_HOME / PATH.
# Without cargo the package is skipped with a warning so the rest of the
# workspace still builds; it installs nothing, so `ros2 run` reports it missing.
function(lucy_cargo_firmware)
  cmake_parse_arguments(ARG "UF2" "BIN;TARGET" "" ${ARGN})
  if(NOT ARG_BIN)
    message(FATAL_ERROR "lucy_cargo_firmware: BIN is required")
  endif()
  if(WIN32)
    # The host firmwares use POSIX shared memory; real hardware is not
    # supported on Windows. Skipping here keeps colcon's package lists alike
    # on every platform.
    message(STATUS "${PROJECT_NAME} not built: firmwares are not built on Windows.")
    return()
  endif()

  set(cargo_env "")
  set(hints "")
  if(DEFINED ENV{CARGO_HOME})
    list(APPEND hints "$ENV{CARGO_HOME}/bin")
  endif()
  if(DEFINED ENV{CONDA_PREFIX} AND EXISTS "$ENV{CONDA_PREFIX}/cargo/bin")
    list(APPEND hints "$ENV{CONDA_PREFIX}/cargo/bin")
    # rustup proxies resolve their toolchain through RUSTUP_HOME.
    set(cargo_env ${CMAKE_COMMAND} -E env
      "CARGO_HOME=$ENV{CONDA_PREFIX}/cargo"
      "RUSTUP_HOME=$ENV{CONDA_PREFIX}/rustup")
  endif()

  find_program(CARGO_EXECUTABLE cargo HINTS ${hints})
  if(NOT CARGO_EXECUTABLE)
    message(WARNING "${PROJECT_NAME} not built: cargo not found. "
      "Run 'pixi run firmware-setup', then rebuild.")
    return()
  endif()
  if(ARG_UF2)
    find_program(ELF2UF2_EXECUTABLE elf2uf2-rs HINTS ${hints})
    if(NOT ELF2UF2_EXECUTABLE)
      message(WARNING "${PROJECT_NAME} not built: elf2uf2-rs not found. "
        "Run 'pixi run firmware-setup', then rebuild.")
      return()
    endif()
  endif()

  # One target dir for every firmware package (next to their colcon build dirs):
  # the crates they share (core, host, serialport...) compile once, and cargo's
  # lock on the directory serialises packages colcon builds in parallel.
  set(target_dir ${CMAKE_BINARY_DIR}/../lucy_embedded_firmware_target)
  set(cargo_args build --release
    --manifest-path ${CMAKE_CURRENT_SOURCE_DIR}/Cargo.toml
    --target-dir ${target_dir})
  if(ARG_TARGET)
    list(APPEND cargo_args --target ${ARG_TARGET})
    set(binary ${target_dir}/${ARG_TARGET}/release/${ARG_BIN})
  else()
    set(binary ${target_dir}/release/${ARG_BIN})
  endif()

  set(commands COMMAND ${cargo_env} ${CARGO_EXECUTABLE} ${cargo_args})
  if(ARG_UF2)
    list(APPEND commands COMMAND ${cargo_env} ${ELF2UF2_EXECUTABLE} ${binary} ${binary}.uf2)
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
