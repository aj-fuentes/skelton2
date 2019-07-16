## #################################################################
## Sources
## #################################################################

set(${PROJECT_NAME}_HEADERS
    base.h
    field.h
    skeleton.h
    graph.h
    convex_hull.h
    scaffolder.h
    mesher.h
)

set(${PROJECT_NAME}_SOURCES
    field.cpp
    skeleton.cpp
    graph.cpp
    convex_hull.cpp
    scaffolder.cpp
    mesher.cpp
)

## #################################################################
## Build rules
## #################################################################

add_library(${PROJECT_NAME} STATIC
    ${${PROJECT_NAME}_SOURCES}
    ${${PROJECT_NAME}_HEADERS})

## ###################################################################
## Link rules
## ##################################################################

# SET_TARGET_PROPERTIES(${PROJECT_NAME} PROPERTIES COMPILE_FLAGS "-fPIC")
# target_link_libraries(${PROJECT_NAME} ${QHULL_LIBRARIES})
target_link_libraries(${PROJECT_NAME} eigen3)
target_link_libraries(${PROJECT_NAME} qhullcpp)
target_link_libraries(${PROJECT_NAME} qhull_r)
target_link_libraries(${PROJECT_NAME} gsl)
target_link_libraries(${PROJECT_NAME} gslcblas)
target_link_libraries(${PROJECT_NAME} glpk)



