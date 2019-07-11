project(skelton_tests)
enable_testing()

# Make test executable
set(TEST_SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/main.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/test_skeleton.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/test_field.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/test_graph.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/test_convex_hull.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/test_scaffolder.cpp
)

add_executable(skelton_tests ${TEST_SOURCES})

target_link_libraries(skelton_tests skelton)
target_link_libraries(skelton_tests Catch)

include(ParseAndAddCatchTests)
ParseAndAddCatchTests(skelton_tests)
