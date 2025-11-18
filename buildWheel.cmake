if (APPLE)
    message(STATUS "Copy *.dylib, *.a files from ${CMAKE_INSTALL_PREFIX}/lib to ${CMAKE_INSTALL_PREFIX}/${OPENSIM_INSTALL_PYTHONDIR}/opensim")
    file(GLOB OPENSIM_ALL_LIBS "${CMAKE_INSTALL_PREFIX}/lib/*.dylib" 
                                "${CMAKE_INSTALL_PREFIX}/lib/*.a"
                                "${CMAKE_INSTALL_PREFIX}/sdk/lib/*.dylib" 
                                "${CMAKE_INSTALL_PREFIX}/sdk/lib/*.a")
    set(DESTINATION_DIR ${CMAKE_INSTALL_PREFIX}/${OPENSIM_INSTALL_PYTHONDIR}/opensim)
    foreach(FILE ${OPENSIM_ALL_LIBS})
        file(COPY ${FILE} DESTINATION ${DESTINATION_DIR})
    endforeach()
else()
    message(STATUS "Copy .so* from ${CMAKE_INSTALL_PREFIX}/lib to ${CMAKE_INSTALL_PREFIX}/${OPENSIM_INSTALL_PYTHONDIR}/opensim")
    file(GLOB OPENSIM_ALL_LIBS "${CMAKE_INSTALL_PREFIX}/sdk/lib/*.so*" "${CMAKE_INSTALL_PREFIX}/lib/*.so*")
    set(DESTINATION_DIR ${CMAKE_INSTALL_PREFIX}/${OPENSIM_INSTALL_PYTHONDIR}/opensim)
    foreach(FILE ${OPENSIM_ALL_LIBS})
        file(COPY ${FILE} DESTINATION ${DESTINATION_DIR})
    endforeach()
    # add $ORIGIN to rpath as layout has changed
    file(GLOB SIMTK_LIBS "${CMAKE_INSTALL_PREFIX}/${OPENSIM_INSTALL_PYTHONDIR}/opensim/libSimTK*.so*")
    foreach(slib ${SIMTK_LIBS})
        execute_process(COMMAND bash "-c" "patchelf --force-rpath --set-rpath '$ORIGIN' '${slib}'")
    endforeach()
endif()