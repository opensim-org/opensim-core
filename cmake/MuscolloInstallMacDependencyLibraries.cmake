# Temporary hack to package dependencies on Macs. TODO

set(bindir "${CMAKE_INSTALL_PREFIX}/bin")
set(libdir "${CMAKE_INSTALL_PREFIX}/lib")
message(STATUS "Editing RPATH of Muscollo dependency libraries")

macro(install_name_tool_change lib dep_name dep_oldpath)
    execute_process(COMMAND install_name_tool
            -change ${dep_oldpath}/lib${dep_name}.dylib
            @rpath/lib${dep_name}.dylib
            "${libdir}/lib${lib}.dylib")
endmacro()

macro(install_name_tool_id lib)
    execute_process(COMMAND install_name_tool
            -id @rpath/lib${lib}.dylib
            "${libdir}/lib${lib}.dylib")
endmacro()

macro(install_name_tool_add_rpath lib)
    execute_process(COMMAND install_name_tool
            -add_rpath "@loader_path/"
            "${libdir}/lib${lib}.dylib")
endmacro()

macro(install_name_tool_delete_rpath lib rpath)
    execute_process(COMMAND install_name_tool
            -delete_rpath ${rpath}
            "${libdir}/lib${lib}.dylib")
endmacro()




install_name_tool_change(osimMuscollo adolc.2 /usr/local/opt/adol-c/lib)



execute_process(COMMAND install_name_tool
        -change /usr/local/opt/llvm/lib/libc++.1.dylib
        /usr/lib/libc++.1.dylib
        "${bindir}/opensim-muscollo")
