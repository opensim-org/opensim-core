
file(GLOB INCLUDES *.h gcvspl.h)
file(GLOB SOURCES *.cpp gcvspl.c)

if(NOT WITH_BTK)
    file(GLOB C3D_HEADER *C3DFileAdapter.h)
    file(GLOB C3D_SOURCE *C3DFileAdapter.cpp)
    list(REMOVE_ITEM INCLUDES ${C3D_HEADER})
    list(REMOVE_ITEM SOURCES  ${C3D_SOURCE})
    unset(BTK_LIBRARIES)
endif()

#################################################### PRECOMPILE HEADER -- Begin.

# Original header.
set(HEADER     "${CMAKE_CURRENT_SOURCE_DIR}/osimCommon.h")
# Precompiled header (PCH).
set(HEADER_PCH "${CMAKE_CURRENT_SOURCE_DIR}/common.h.gch")

# Append necessary flags for compiling PCH.
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Winvalid-pch -fPIC")

# Create define flag for each macro definition.
get_property(MACRO_DEFS DIRECTORY PROPERTY COMPILE_DEFINITIONS)
set(MACRO_DEFINITIONS "-DOSIMCOMMON_EXPORTS")
foreach(DEF ${MACRO_DEFS})
    set(MACRO_DEFINITIONS ${MACRO_DEFINITIONS} "-D${DEF}")
endforeach()

# Create include flag for each include directory.
get_property(INCLUDE_DIRECTORIES DIRECTORY PROPERTY INCLUDE_DIRECTORIES)
foreach(DIR ${INCLUDE_DIRECTORIES})
    set(INCLUDE_FLAGS ${INCLUDE_FLAGS} -I"${DIR}")
endforeach()
set(INCLUDE_FLAGS ${INCLUDE_FLAGS} -I"${CMAKE_SOURCE_DIR}")

# Lump all arguments together.
set(PCH_ARGS ${CMAKE_CXX_COMPILER_ARG1} 
             ${MACRO_DEFINITIONS}
             ${CMAKE_CXX_FLAGS} 
             ${CMAKE_CXX_FLAGS_RELWITHDEBINFO}
             ${INCLUDE_FLAGS}
             -c ${HEADER}
             -o ${HEADER_PCH})
separate_arguments(PCH_ARGS)

add_custom_command(OUTPUT ${HEADER_PCH}
    COMMAND rm -f ${HEADER_PCH}
    COMMAND ${CMAKE_CXX_COMPILER} ${PCH_ARGS} 
    DEPENDS ${INCLUDES} ${HEADER}
    COMMENT "Generating precompiled header -- common")

#################################################### PRECOMPILE HEADER -- End.

OpenSimAddLibrary(KIT Common
    LINKLIBS ${Simbody_LIBRARIES} ${BTK_LIBRARIES}
    INCLUDES ${INCLUDES}
    SOURCES ${SOURCES} ${HEADER_PCH}
    TESTDIRS "Test")
