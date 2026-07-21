set(_jie_max_tool_sources
    ${CMAKE_CURRENT_LIST_DIR}/convert.c
    ${CMAKE_CURRENT_LIST_DIR}/convert.h
    ${CMAKE_CURRENT_LIST_DIR}/process.h
)

target_sources(${PROJECT_NAME} PRIVATE ${_jie_max_tool_sources})

target_include_directories(${PROJECT_NAME} PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}
)
