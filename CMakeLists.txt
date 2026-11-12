list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_LIST_DIR}/eil-cmake-utils/cmake)

idf_component_register(
    SRCS calibration.c calibration_pchip.c calibration_grid.c
    INCLUDE_DIRS .
    REQUIRES log
)

include(eil_ci)
