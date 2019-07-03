project(skelton_tests)
enable_testing()

# Make test executable
set(TEST_SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/main.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/test_skeleton.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/test_field.cpp
    # ${CMAKE_CURRENT_SOURCE_DIR}/test-ArcLength.cpp
    # ${CMAKE_CURRENT_SOURCE_DIR}/test-AxelVisualization.cpp
    # ${CMAKE_CURRENT_SOURCE_DIR}/test-ConstantMetric.cpp
    # ${CMAKE_CURRENT_SOURCE_DIR}/test-Field.cpp
    # ${CMAKE_CURRENT_SOURCE_DIR}/test-G1Sleeve.cpp
    # ${CMAKE_CURRENT_SOURCE_DIR}/test-LinearMetric.cpp
    # ${CMAKE_CURRENT_SOURCE_DIR}/test-Mesher.cpp
    # ${CMAKE_CURRENT_SOURCE_DIR}/test-Segment.cpp
    # ${CMAKE_CURRENT_SOURCE_DIR}/test-SkeletonGraph.cpp
    # ${CMAKE_CURRENT_SOURCE_DIR}/test-SkeletonMetric.cpp
    # ${CMAKE_CURRENT_SOURCE_DIR}/test-SkeletonNode.cpp
    # ${CMAKE_CURRENT_SOURCE_DIR}/test-Reader.cpp
    # ${CMAKE_CURRENT_SOURCE_DIR}/test-examples-segment.cpp
    # ${CMAKE_CURRENT_SOURCE_DIR}/test-examples-arc.cpp
    # ${CMAKE_CURRENT_SOURCE_DIR}/test-examples-arc-and-segment.cpp
    # ${CMAKE_CURRENT_SOURCE_DIR}/test-examples-g1-pieces.cpp
    # ${CMAKE_CURRENT_SOURCE_DIR}/test-examples-knot.cpp
)

add_executable(skelton_tests ${TEST_SOURCES})

target_link_libraries(skelton_tests skelton)
target_link_libraries(skelton_tests Catch)

# add_custom_command(TARGET skelton_tests POST_BUILD
#     COMMAND ${CMAKE_COMMAND} -E copy_directory ${CMAKE_CURRENT_SOURCE_DIR}/skels ${CMAKE_CURRENT_BINARY_DIR}/skels)

include(ParseAndAddCatchTests)
ParseAndAddCatchTests(skelton_tests)
