# cmake_minimum_required(VERSION 3.1.3)

# Currently this application is only supported
# on Windows.
if(NOT WIN32)
    return()
endif()

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(XSENS_SDK_DIR "" CACHE PATH "Directory containing XSENS SDK.")

if(NOT XSENS_SDK_DIR)
    return()
endif()

if(CMAKE_SIZEOF_VOID_P EQUAL 8)
    # 64-bit build.
    set(XSENS_ROOT "${XSENS_SDK_DIR}/x64")
    set(XSENS_LIBRARIES "${XSENS_ROOT}/lib/xsensdeviceapi64.lib"
                        "${XSENS_ROOT}/lib/xstypes64.lib")
    set(XSENS_DLLS "${XSENS_ROOT}/lib/xsensdeviceapi64.dll"
                   "${XSENS_ROOT}/lib/xstypes64.dll")
else()
    # 32-bit build.
    set(XSENS_ROOT "${XSENS_SDK_DIR}/Win32")
    set(XSENS_LIBRARIES "${XSENS_ROOT}/lib/xsensdeviceapi32.lib"
                        "${XSENS_ROOT}/lib/xstypes32.lib")
    set(XSENS_DLLS "${XSENS_ROOT}/lib/xsensdeviceapi32.dll"
                   "${XSENS_ROOT}/lib/xstypes32.dll")
endif()

include_directories("${XSENS_ROOT}/include")

foreach(DLL ${XSENS_DLLS})
    get_filename_component(DLLNAME ${DLL} NAME)
    set(XSENS_DLL_TARGETS ${XSENS_DLL_TARGETS} ${DLLNAME})
    add_custom_target(${DLLNAME}
        COMMAND ${CMAKE_COMMAND} -E copy ${DLL}
                "${CMAKE_BINARY_DIR}/${CMAKE_CFG_INTDIR}"
        COMMENT "Copying XSENS DLL '${DLL}' to ${CMAKE_CFG_INTDIR}")
endforeach()

add_executable(XsensImuStreaming ImuStreaming.cpp)
target_link_libraries(XsensImuStreaming 
    osimCommon osimSimulation osimTools ${XSENS_LIBRARIES})
add_dependencies(XsensImuStreaming ${XSENS_DLL_TARGETS})
