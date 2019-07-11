project(skelton_icesl_plugin)

#Make plugin executable
set(ICESL_PLUGIN_SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/main.cpp
)

add_executable(skelton_icesl_plugin ${ICESL_PLUGIN_SOURCES})
target_link_libraries(skelton_icesl_plugin skelton)
