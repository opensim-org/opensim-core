SET(KIT acpp)

IF(WIN32)
FILE(GLOB SOURCE_FILES alloca.c cccp.c obstack.c version.c y.tab.c)
ELSE(WIN32)
FILE(GLOB SOURCE_FILES cccp.c obstack.c version.c y_linux.tab.c)
ENDIF(WIN32)

FILE(GLOB INCLUDE_FILES *.h)
                
ADD_LIBRARY(acpp ${SOURCE_FILES} ${INCLUDE_FILES}) 

SET_TARGET_PROPERTIES(acpp PROPERTIES PROJECT_LABEL "Vendor Libraries - acpp")

IF (WIN32)
    INSTALL_TARGETS(/sdk/lib/ acpp)
ELSE(WIN32)
    INSTALL_TARGETS(/bin/ acpp)
ENDIF(WIN32)
