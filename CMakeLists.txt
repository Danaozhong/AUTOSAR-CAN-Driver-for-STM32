
SET(CAN_SOURCES
    Can/Can.c
    CanIf/CanIf.c
    CanIf/CanIf_Cfg.c
    CanIf/CanIf_Lcfg.c
    config/Can_Lcfg.c
    Det/Det.c
)


add_library(can_stack STATIC ${CAN_SOURCES})

target_compile_definitions(can_stack PUBLIC
    -DHAL_CAN_MODULE_ENABLED
)

target_include_directories(can_stack PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMSIS_INCLUDE_DIRS}
    ${STM32HAL_INCLUDE_DIR}
    ../../Inc # needed for HW depending stuff, TODO find a better solution for this include
)

#target_include_directories(can_stack PUBLIC
#    
#
#)
target_include_directories(can_stack PUBLIC
    Can
    CanIf
    config
    Det
    EcuM
    ${CMAKE_CURRENT_SOURCE_DIR}/PduR
    ${CMAKE_CURRENT_SOURCE_DIR}/SchM
    ${CMAKE_CURRENT_SOURCE_DIR}/shared
)
          