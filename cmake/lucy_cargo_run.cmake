# Run one build command and print a condensed log: usage
#   cmake -DLABEL=<name> "-DRUN_COMMAND=<cmd|arg|...>" -P lucy_cargo_run.cmake
# The command is "|"-separated because a ";" list is split on the build command line.
# Warnings and errors are kept, cargo's progress lines are dropped, and a failure
# prints the whole log before failing.
string(REPLACE "|" ";" run_command "${RUN_COMMAND}")
execute_process(
  COMMAND ${run_command}
  RESULT_VARIABLE status
  OUTPUT_VARIABLE log
  ERROR_VARIABLE log)

function(echo_text text)
  if(NOT text STREQUAL "")
    execute_process(COMMAND ${CMAKE_COMMAND} -E echo "${text}")
  endif()
endfunction()

if(NOT status EQUAL 0)
  echo_text("${log}")
  message(FATAL_ERROR "${LABEL}: build failed (${status})")
endif()

set(progress "Compiling|Locking|Updating|Blocking|Adding|Downloading|Downloaded|Finished|Fresh|Checking|Packaging")
string(REGEX REPLACE "(^|\n) *(${progress}) [^\n]*" "\\1" kept "${log}")
string(REGEX REPLACE "\n\n+" "\n" kept "${kept}")
string(STRIP "${kept}" kept)
echo_text("${kept}")
echo_text("${LABEL}: built")
