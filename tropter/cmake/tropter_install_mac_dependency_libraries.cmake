# Temporary hack to package dependencies on Macs. TODO

set(libdir "${CMAKE_INSTALL_PREFIX}/lib")
message(STATUS "Editing RPATH of tropter dependency libraries")

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

# tropter
install_name_tool_change(tropter adolc.2 /usr/local/opt/adol-c/lib)
install_name_tool_change(tropter ipopt.1 /usr/local/opt/ipopt/lib)
# TODO why does tropter link directly to mumps?
install_name_tool_change(tropter dmumps /usr/local/opt/mumps/lib)
install_name_tool_change(tropter mumps_common /usr/local/opt/mumps/lib)
install_name_tool_change(tropter pord /usr/local/opt/mumps/lib)
install_name_tool_change(tropter mpiseq /usr/local/opt/mumps/lib)
# TODO will this work on other machines? link with Accelerate instead?
install_name_tool_change(tropter openblasp-r0.2.20 /usr/local/opt/openblas/lib)
install_name_tool_add_rpath(tropter)


# adol-c
install_name_tool_id(adolc.2)
install_name_tool_change(adolc.2 boost_system /usr/local/opt/boost/lib)
install_name_tool_change(adolc.2 ColPack.0 /usr/local/opt/colpack/lib)
install_name_tool_add_rpath(adolc.2)
install_name_tool_delete_rpath(adolc.2 /usr/local/opt/colpack/lib)

# ipopt
install_name_tool_id(ipopt.1)
install_name_tool_change(ipopt.1 dmumps /usr/local/opt/mumps/lib)
install_name_tool_change(ipopt.1 mumps_common /usr/local/opt/mumps/lib)
install_name_tool_change(ipopt.1 pord /usr/local/opt/mumps/lib)
install_name_tool_change(ipopt.1 mpiseq /usr/local/opt/mumps/lib)
install_name_tool_change(ipopt.1 openblasp-r0.2.20 /usr/local/opt/openblas/lib)

# dmumps
install_name_tool_id(dmumps)
install_name_tool_change(dmumps gfortran.4 /usr/local/opt/gcc/lib/gcc/7)
install_name_tool_change(dmumps gcc_s.1 /usr/local/lib/gcc/7)
install_name_tool_change(dmumps quadmath.0 /usr/local/opt/gcc/lib/gcc/7)
install_name_tool_add_rpath(dmumps)

# mumps_common
install_name_tool_id(mumps_common)
install_name_tool_change(mumps_common gfortran.4 /usr/local/opt/gcc/lib/gcc/7)
install_name_tool_change(mumps_common gcc_s.1 /usr/local/lib/gcc/7)
install_name_tool_change(mumps_common quadmath.0 /usr/local/opt/gcc/lib/gcc/7)
install_name_tool_add_rpath(mumps_common)

# pord
install_name_tool_id(pord)
install_name_tool_change(pord gfortran.4 /usr/local/opt/gcc/lib/gcc/7)
install_name_tool_change(pord gcc_s.1 /usr/local/lib/gcc/7)
install_name_tool_change(pord quadmath.0 /usr/local/opt/gcc/lib/gcc/7)
install_name_tool_add_rpath(pord)

# mpiseq
install_name_tool_id(mpiseq)
install_name_tool_change(mpiseq gfortran.4 /usr/local/opt/gcc/lib/gcc/7)
install_name_tool_change(mpiseq gcc_s.1 /usr/local/lib/gcc/7)
install_name_tool_change(mpiseq quadmath.0 /usr/local/opt/gcc/lib/gcc/7)
install_name_tool_add_rpath(mpiseq)

# openblas
install_name_tool_id(openblasp-r0.2.20)
install_name_tool_change(openblasp-r0.2.20 gfortran.4 /usr/local/opt/gcc/lib/gcc/7)
install_name_tool_change(openblasp-r0.2.20 quadmath.0 /usr/local/opt/gcc/lib/gcc/7)
install_name_tool_add_rpath(openblasp-r0.2.20)

# boost_system
install_name_tool_id(boost_system)

# Colpack.0
install_name_tool_id(ColPack.0)

# gfortran
install_name_tool_id(gfortran.4)
install_name_tool_change(gfortran.4 gcc_s.1 /usr/local/lib/gcc/7)
install_name_tool_change(gfortran.4 quadmath.0
        /usr/local/Cellar/gcc/7.2.0/lib/gcc/7)
install_name_tool_add_rpath(gfortran.4)

# gcc_s
install_name_tool_id(gcc_s.1)

# quadmath
install_name_tool_id(quadmath.0)
install_name_tool_change(quadmath.0 gcc_s.1 /usr/local/lib/gcc/7)
install_name_tool_add_rpath(quadmath.0)
