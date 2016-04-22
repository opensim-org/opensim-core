cmake_minimum_required(VERSION 3.1.3)

# Currently this application is only supported
# on Windows.
if(NOT WIN32)
    return()
endif()

set(CMAKE_CXX_STANDARD_REQUIRED 11)
set(CMAKE_CXX_STANDARD 11)

set(XSENS_SDK_DIR "" CACHE PATH
    "Directory containing XSENS SDK.")

if(NOT XSENS_SDK_DIR)
    return()
endif()

if(CMAKE_SIZEOF_VOID_P EQUAL 8)
    # 64-bit build.
    set(XSENS_ROOT "${XSENS_SDK_DIR}/x64")
    set(XSENS_LIBRARIES "${XSENS_ROOT}/lib/xsensdeviceapi64.lib"
                        "${XSENS_ROOT}/lib/xstypes64.lib")
else()
    # 32-bit build.
    set(XSENS_ROOT "${XSENS_SDK_DIR}/Win32")
    set(XSENS_LIBRARIES "${XSENS_ROOT}/lib/xsensdeviceapi32.lib"
                        "${XSENS_ROOT}/lib/xstypes32.lib")
endif()

include_directories("${XSENS_ROOT}/include")

add_executable(XsensImuStreaming ImuStreaming.cpp)
target_link_libraries(XsensImuStreaming ${XSENS_LIBRARIES})

file(GLOB XSENS_DLLS "${XSENS_ROOT}/lib/*.dll")
foreach(DLL ${XSENS_DLLS})
    get_filename_component(DLLNAME ${DLL} NAME)
    add_custom_target(CopyXsensDll_${DLLNAME}
                      COMMAND ${CMAKE_COMMAND} -E copy ${DLL}
                              "${CMAKE_BINARY_DIR}/${CMAKE_CFG_INTDIR}")
endforeach()
