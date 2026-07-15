set(_jie_max_module_sources
    ${CMAKE_CURRENT_LIST_DIR}/DR16/dr16.c
    ${CMAKE_CURRENT_LIST_DIR}/game/game.c
    ${CMAKE_CURRENT_LIST_DIR}/LX824/lx824.c
    ${CMAKE_CURRENT_LIST_DIR}/Vofa/vofa.c
    ${CMAKE_CURRENT_LIST_DIR}/I6X/i6x.c
    ${CMAKE_CURRENT_LIST_DIR}/motor/dj_motor/dj_motor_codec.c
    ${CMAKE_CURRENT_LIST_DIR}/motor/dj_motor/dj_motor_drv.c
    ${CMAKE_CURRENT_LIST_DIR}/motor/dj_motor/dj_motor_ctrl.c
    ${CMAKE_CURRENT_LIST_DIR}/motor/dm_motor/dm_motor_codec.c
    ${CMAKE_CURRENT_LIST_DIR}/motor/dm_motor/dm_motor_drv.c
    ${CMAKE_CURRENT_LIST_DIR}/motor/dm_motor/dm_motor_param.c
    ${CMAKE_CURRENT_LIST_DIR}/motor/dm_motor/dm_motor_ctrl.c
)

target_sources(${PROJECT_NAME} PRIVATE ${_jie_max_module_sources})

target_include_directories(${PROJECT_NAME} PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}/DR16
    ${CMAKE_CURRENT_LIST_DIR}/game
    ${CMAKE_CURRENT_LIST_DIR}/LX824
    ${CMAKE_CURRENT_LIST_DIR}/Vofa
    ${CMAKE_CURRENT_LIST_DIR}/I6X
    ${CMAKE_CURRENT_LIST_DIR}/motor/dj_motor
    ${CMAKE_CURRENT_LIST_DIR}/motor/dm_motor
)
