project(IMUStreaming)

cmake_minimum_required(VERSION 3.1.3)

set(CMAKE_CXX_STANDARD_REQUIRED 11)
set(CMAKE_CXX_STANDARD 11)

set(XSENS_ROOT "/home/shrik/tmp/MT_Software_Suite_Linux_4.5/mtsdk")

include_directories("${XSENS_ROOT}/include")
include_directories(${CMAKE_CURRENT_SOURCE_DIR})
set(XSENS_LIBRARIES "${XSENS_ROOT}/lib/libxstypes.so" 
                    "${XSENS_ROOT}/lib/libxsensdeviceapi.so")

add_executable(IMUStreaming ImuStreaming.cpp)
target_link_libraries(IMUStreaming ${XSENS_LIBRARIES})
