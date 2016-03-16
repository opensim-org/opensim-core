
# How to set java.library.path for MATLAB:
# http://www.mathworks.com/matlabcentral/answers/102751-how-do-i-configure-the-java-run-time-library-path-java-library-path-in-matlab-with-and-without-adm

set(matlab_output_dir "${CMAKE_BINARY_DIR}/Matlab")
file(WRITE "${matlab_output_dir}/javaclasspath.txt"
    "${SWIG_JAVA_JAR_BUILD_OUTPUT_PATH}")
file(WRITE "${matlab_output_dir}/javalibrarypath.txt" "${CMAKE_BINARY_DIR}")


# Test.
# -----
macro(OpenSimAddMatlabTest MFILE)
    get_filename_component(_full_path_to_mfile "${MFILE}" ABSOLUTE)
    if(WIN32)
        # On Windows, must set ADDITIONAL_PATH to find the opensim
        # libraries.
        set(_additional_path "${CMAKE_BINARY_DIR}/${CMAKE_CFG_INTDIR}")
    endif()
    matlab_add_unit_test(NAME MATLAB_TugOfWar
        UNITTEST_FILE "${_full_path_to_mfile}"
        TEST_ARGS CONFIGURATION Release RelWithDebInfo MinSizeRel
        ADDITIONAL_PATH "${_additional_path}"
        )
endmacro()


OpenSimAddMatlabTest(../examples/OpenSimCreateTugOfWarModel.m)
