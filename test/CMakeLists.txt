project(skelton_tests)
enable_testing()

# Make test executable
set(TEST_SOURCES
    main.cpp
    test_skeleton.cpp
    test_field.cpp
    test_graph.cpp
    test_convex_hull.cpp
    test_scaffolder.cpp
    test_mesher.cpp
)

add_executable(skelton_tests ${TEST_SOURCES})

target_link_libraries(skelton_tests skelton)
target_link_libraries(skelton_tests Catch)

include(ParseAndAddCatchTests)
ParseAndAddCatchTests(skelton_tests)
