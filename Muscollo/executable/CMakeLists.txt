
add_executable(opensim-muscollo ../executable/opensim-muscollo.cpp)
target_link_libraries(opensim-muscollo osimMuscollo)

install(TARGETS opensim-muscollo DESTINATION bin)
set_target_properties(opensim-muscollo PROPERTIES
        DEBUG_POSTFIX ${CMAKE_DEBUG_POSTFIX}
        PROJECT_LABEL "Executable - opensim-muscollo"
        FOLDER "Muscollo")
