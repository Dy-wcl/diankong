set(_jie_max_calculate_sources
    ${CMAKE_CURRENT_LIST_DIR}/chassis_control/chassis_control.c
    ${CMAKE_CURRENT_LIST_DIR}/joint_control/joint_control.c
)

target_sources(${PROJECT_NAME} PRIVATE ${_jie_max_calculate_sources})

target_include_directories(${PROJECT_NAME} PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}/chassis_control
    ${CMAKE_CURRENT_LIST_DIR}/joint_control
)
