set(KIT acpp)

if(WIN32)
file(GLOB SOURCE_FILES alloca.c cccp.c obstack.c version.c y.tab.c)
else(WIN32)
file(GLOB SOURCE_FILES cccp.c obstack.c version.c y_linux.tab.c)
endif(WIN32)

file(GLOB INCLUDE_FILES *.h)
                
add_library(acpp ${SOURCE_FILES} ${INCLUDE_FILES}) 

set_target_properties(acpp PROPERTIES PROJECT_LABEL "Vendor Libraries - acpp")

if(WIN32)
    install_targets(/sdk/lib/ acpp)
else(WIN32)
    install_targets(/bin/ acpp)
endif(WIN32)
