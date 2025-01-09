# This is an undocumented internal helper for the FindMatlab
# module ``matlab_add_unit_test`` command.

#=============================================================================
# Copyright 2014-2015 Raffi Enficiaud, Max Planck Society
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
# 
# * Redistributions of source code must retain the above copyright
#   notice, this list of conditions and the following disclaimer.
# 
# * Redistributions in binary form must reproduce the above copyright
#   notice, this list of conditions and the following disclaimer in the
#   documentation and/or other materials provided with the distribution.
# 
# * Neither the names of Kitware, Inc., the Insight Software Consortium,
#   nor the names of their contributors may be used to endorse or promote
#   products derived from this software without specific prior written
#   permission.
# 
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
# A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
# HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
# SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
# DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
# THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#=============================================================================
# 
# Modified by chrisdembia to remove "-nojvm" flag, because we need MATLAB's JVM
# to run our tests.
# TODO: Recent versions of CMake have removed the "-nojvm" flag, so we could
# remove this file from our repo if we make the minimum required CMake version
# 3.7 or above.


# Usage: cmake
#   -Dtest_timeout=180
#   -Doutput_directory=
#   -Dadditional_paths=""
#   -Dno_unittest_framework=""
#   -DMatlab_PROGRAM=matlab_exe_location
#   -DMatlab_ADDITIONNAL_STARTUP_OPTIONS=""
#   -Dtest_name=name_of_the_test
#   -Dcmd_to_run_before_test=""
#   -Dunittest_file_to_run
#   -P FindMatlab_TestsRedirect.cmake

set(Matlab_UNIT_TESTS_CMD -nosplash -nodesktop -nodisplay ${Matlab_ADDITIONNAL_STARTUP_OPTIONS})
if(WIN32)
  set(Matlab_UNIT_TESTS_CMD ${Matlab_UNIT_TESTS_CMD} -wait)
endif()

if(NOT test_timeout)
  set(test_timeout 180)
endif()

if(NOT cmd_to_run_before_test)
  set(cmd_to_run_before_test)
endif()

get_filename_component(unittest_file_directory   "${unittest_file_to_run}" DIRECTORY)
get_filename_component(unittest_file_to_run_name "${unittest_file_to_run}" NAME_WE)

set(concat_string '${unittest_file_directory}')
foreach(s IN LISTS additional_paths)
  if(NOT "${s}" STREQUAL "")
    set(concat_string "${concat_string}, '${s}'")
  endif()
endforeach()

set(unittest_to_run "runtests('${unittest_file_to_run_name}'), exit(max([ans(1,:).Failed]))")
if(no_unittest_framework)
  set(unittest_to_run "try, ${unittest_file_to_run_name}, catch err, disp('An exception has been thrown during the execution'), disp(err), disp(err.stack), exit(1), end, exit(0)")
endif()

set(Matlab_SCRIPT_TO_RUN
    "addpath(${concat_string}), path, ${cmd_to_run_before_test}, ${unittest_to_run}"
   )

set(Matlab_LOG_FILE "${output_directory}/${test_name}.log")

set(devnull)
if(UNIX)
  set(devnull INPUT_FILE /dev/null)
elseif(WIN32)
  set(devnull INPUT_FILE NUL)
endif()

execute_process(
  COMMAND "${Matlab_PROGRAM}" ${Matlab_UNIT_TESTS_CMD} -logfile "${test_name}.log" -r "${Matlab_SCRIPT_TO_RUN}"
  RESULT_VARIABLE res
  TIMEOUT ${test_timeout}
  OUTPUT_QUIET # we do not want the output twice
  WORKING_DIRECTORY "${output_directory}"
  ${devnull}
  )

if(NOT EXISTS ${Matlab_LOG_FILE})
  message( FATAL_ERROR "[MATLAB] ERROR: cannot find the log file ${Matlab_LOG_FILE}")
endif()

# print the output in any case.
file(READ ${Matlab_LOG_FILE} matlab_log_content)
message("Matlab test ${name_of_the_test} output:\n${matlab_log_content}") # if we put FATAL_ERROR here, the file is indented.


if(NOT (res EQUAL 0))
  message( FATAL_ERROR "[MATLAB] TEST FAILED" )
endif()
