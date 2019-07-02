## #################################################################
## Sources
## #################################################################

set(${PROJECT_NAME}_HEADERS
    base.h
    field.h
    skeleton.h
)

set(${PROJECT_NAME}_SOURCES
    field.cpp
    skeleton.cpp
)

## #################################################################
## Build rules
## #################################################################

add_library(${PROJECT_NAME} SHARED
    ${${PROJECT_NAME}_SOURCES}
    ${${PROJECT_NAME}_HEADERS})

## ###################################################################
## Link rules
## ##################################################################

target_link_libraries(${PROJECT_NAME} ${QHULL_LIBRARIES})
target_link_libraries(${PROJECT_NAME} GSL::gsl)
target_link_libraries(${PROJECT_NAME} GSL::gslcblas)



