set(_jie_max_app_sources
    ${CMAKE_CURRENT_LIST_DIR}/chassis/chassis_dynamics.c
)
target_sources(${PROJECT_NAME} PRIVATE ${_jie_max_app_sources})
target_include_directories(${PROJECT_NAME} PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}/chassis
)
