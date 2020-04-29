OpenSimAddApplication(NAME opensim-cmd SOURCES
        cli_helpers.h
        cli_helpers.cpp
        subcommands/run-tool.h
        subcommands/run-tool.cpp
        subcommands/print-xml.h
        subcommands/print-xml.cpp
        subcommands/info.h
        subcommands/info.cpp
        subcommands/update-file.h
        subcommands/update-file.cpp)

if(BUILD_TESTING)
    subdirs(test)
endif(BUILD_TESTING)
