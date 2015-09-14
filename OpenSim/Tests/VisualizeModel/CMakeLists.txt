
set(VISUALIZE_MODEL visualizeModel)    

add_executable(${VISUALIZE_MODEL} VisualizeModel.cpp)

target_link_libraries(${VISUALIZE_MODEL} osimTools)

set_target_properties(${VISUALIZE_MODEL}        
       PROPERTIES        
       PROJECT_LABEL "Utilities - ${VISUALIZE_MODEL}"       
)
